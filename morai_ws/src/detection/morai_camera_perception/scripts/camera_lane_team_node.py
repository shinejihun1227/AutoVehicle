#!/usr/bin/env python3
"""CAMERA팀 lane_segmentation.onnx를 기존 주행 인터페이스로 연결하는 노드.

CAMERA팀 브랜치의 핵심 자산은 YOLOP 계열의 차선/주행영역 segmentation ONNX와
MORAI 카메라 보정·라벨 생성 규칙이다. 해당 저장소에는 이를 ROS 차선 메시지로
발행하는 완성 노드가 없으므로, 이 파일에서 다음 경계를 담당한다.

    sensor_msgs/CompressedImage
        -> lane_segmentation.onnx (cv2.dnn)
        -> lane line mask 후처리
        -> morai_perception_msgs/LaneDetection

기존 주행부가 요구하는 lateral_offset_m, heading_error_rad, confidence, valid만
발행하므로 EKF, 곡률 Pure Pursuit, camera fallback, control mux는 변경하지 않는다.
차선 디버그 영상은 원본 위에 segmentation mask와 추정 중심선을 그려 별도 토픽으로
발행한다. 이 노드는 CtrlCmd나 UDP 제어 패킷을 발행하지 않는다.
"""

from __future__ import annotations

import math
import os
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

import rospy
from morai_perception_msgs.msg import LaneDetection
from sensor_msgs.msg import CompressedImage

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


Fit = Tuple[float, ...]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def finite(value: float) -> bool:
    return math.isfinite(float(value))


def median(values: List[float]) -> float:
    if not values:
        return math.nan
    ordered = sorted(float(value) for value in values if finite(value))
    if not ordered:
        return math.nan
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


class CameraTeamLaneNode:
    def __init__(self) -> None:
        rospy.init_node("camera_lane_team_node", anonymous=False)

        self.image_topic = rospy.get_param(
            "~image_topic", "/camera/front/image/compressed"
        )
        self.output_topic = rospy.get_param("~output_topic", "/detection/lane")
        self.debug_topic = rospy.get_param(
            "~debug_topic", "/detection/lane_debug/compressed"
        )
        self.mask_topic = rospy.get_param(
            "~mask_topic", "/detection/lane_model_mask/compressed"
        )
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))
        self.publish_mask = bool(rospy.get_param("~publish_mask", True))

        self.model_path = os.path.abspath(
            os.path.expanduser(
                str(
                    rospy.get_param(
                        "~model_path", "models/lane_segmentation.onnx"
                    )
                )
            )
        )
        self.model_width = max(32, int(rospy.get_param("~model_width", 640)))
        self.model_height = max(32, int(rospy.get_param("~model_height", 360)))
        self.lane_threshold = clamp(
            float(rospy.get_param("~lane_threshold", 0.50)), 0.05, 0.95
        )
        self.roi_start_ratio = clamp(
            float(rospy.get_param("~roi_start_ratio", 0.45)), 0.0, 0.95
        )
        self.roi_top_ratio = clamp(
            float(rospy.get_param("~roi_top_ratio", 0.58)), 0.05, 0.95
        )
        self.roi_bottom_ratio = clamp(
            float(rospy.get_param("~roi_bottom_ratio", 0.98)), 0.5, 1.0
        )
        self.lane_width_m = max(
            0.5, float(rospy.get_param("~lane_width_m", 3.5))
        )
        self.default_lane_width_px_ratio = clamp(
            float(rospy.get_param("~default_lane_width_px_ratio", 0.35)),
            0.10,
            0.90,
        )
        self.filter_window = max(1, int(rospy.get_param("~filter_window", 5)))
        self.fit_degree = max(1, min(2, int(rospy.get_param("~fit_degree", 2))))
        self.row_samples = max(8, int(rospy.get_param("~row_samples", 28)))
        self.row_half_window = max(0, int(rospy.get_param("~row_half_window", 2)))
        self.min_side_points = max(
            3, int(rospy.get_param("~min_side_points", 6))
        )
        self.max_fit_residual_px = max(
            1.0, float(rospy.get_param("~max_fit_residual_px", 18.0))
        )
        self.output_sign = 1.0 if float(rospy.get_param("~output_sign", 1.0)) >= 0 else -1.0

        self.publisher = rospy.Publisher(self.output_topic, LaneDetection, queue_size=2)
        self.debug_publisher = rospy.Publisher(
            self.debug_topic, CompressedImage, queue_size=1
        )
        self.mask_publisher = rospy.Publisher(
            self.mask_topic, CompressedImage, queue_size=1
        )
        rospy.Subscriber(self.image_topic, CompressedImage, self.callback, queue_size=2)

        self.offset_history: Deque[float] = deque(maxlen=self.filter_window)
        self.heading_history: Deque[float] = deque(maxlen=self.filter_window)
        self.lane_width_history: Deque[float] = deque(maxlen=self.filter_window)
        self.last_lane_width_px: Optional[float] = None
        self.net = None
        self.output_names: List[str] = []

        if cv2 is None or np is None:
            rospy.logerr("camera_lane_team_node에 OpenCV/numpy가 없습니다.")
        elif not os.path.isfile(self.model_path):
            rospy.logerr("CAMERA팀 ONNX 모델이 없습니다: %s", self.model_path)
        else:
            try:
                self.net = cv2.dnn.readNet(self.model_path)
                self.output_names = list(self.net.getUnconnectedOutLayersNames())
                rospy.loginfo(
                    "CAMERA팀 lane model loaded: %s, outputs=%s, input=%dx%d",
                    self.model_path,
                    self.output_names,
                    self.model_width,
                    self.model_height,
                )
            except Exception as exc:  # pylint: disable=broad-except
                rospy.logerr("CAMERA팀 ONNX 모델을 읽지 못했습니다: %s", exc)
                self.net = None

        rospy.loginfo(
            "Camera team lane: image=%s output=%s debug=%s mask=%s",
            self.image_topic,
            self.output_topic,
            self.debug_topic if self.publish_debug else "disabled",
            self.mask_topic if self.publish_mask else "disabled",
        )

    @staticmethod
    def decode(message: CompressedImage):
        if cv2 is None or np is None:
            return None
        data = np.frombuffer(message.data, dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)

    def publish_lane(self, message: CompressedImage, valid: bool, values=None) -> None:
        output = LaneDetection()
        output.header = message.header
        if not output.header.frame_id:
            output.header.frame_id = "front_camera"
        if not valid or values is None:
            output.valid = False
            output.confidence = 0.0
            self.publisher.publish(output)
            return
        output.lateral_offset_m = float(values[0])
        output.heading_error_rad = float(values[1])
        output.confidence = float(clamp(values[2], 0.0, 1.0))
        output.valid = True
        self.publisher.publish(output)

    def output_candidates(self, outputs) -> Dict[str, Any]:
        """모델 출력 이름 차이를 흡수한다.

        GenerateLabels.py는 출력 이름을 `da`/`ll`로 사용하지만, ONNX export
        도구에 따라 이름이 바뀔 수 있다. 이름을 먼저 보고, 마지막으로 2채널
        segmentation tensor의 shape를 보고 lane 출력을 찾는다.
        """
        candidates: Dict[str, Any] = {}
        named = list(zip(self.output_names, outputs))
        for name, output in named:
            normalized = str(name).lower()
            if normalized in ("ll", "lane", "lane_line", "lane_line_seg"):
                candidates["lane"] = output
            elif normalized in ("da", "drive", "drivable", "drive_area_seg"):
                candidates["drive"] = output

        if "lane" not in candidates:
            tensors = []
            for output in outputs:
                shape = tuple(int(v) for v in getattr(output, "shape", ()))
                if len(shape) >= 3 and 2 in shape:
                    tensors.append(output)
            if tensors:
                # CAMERA팀 모델의 출력 순서는 da, ll이다. 이름이 제거된
                # export에서는 두 번째 2채널 tensor를 차선으로 우선한다.
                candidates["lane"] = tensors[-1]
            elif outputs:
                candidates["lane"] = outputs[-1]
        return candidates

    @staticmethod
    def binary_probability(output):
        """2채널 logits/확률 tensor를 foreground 확률 맵으로 바꾼다."""
        array = np.asarray(output)
        if array.ndim == 4:
            array = array[0]
        if array.ndim == 3 and array.shape[0] in (1, 2):
            channels = array
        elif array.ndim == 3 and array.shape[-1] in (1, 2):
            channels = np.transpose(array, (2, 0, 1))
        elif array.ndim == 2:
            channels = array[np.newaxis, ...]
        else:
            raise ValueError("지원하지 않는 segmentation 출력 shape: %s" % (array.shape,))

        channels = channels.astype(np.float32)
        if channels.shape[0] >= 2:
            difference = np.clip(channels[1] - channels[0], -40.0, 40.0)
            return 1.0 / (1.0 + np.exp(-difference))
        return 1.0 / (1.0 + np.exp(-np.clip(channels[0], -40.0, 40.0)))

    def infer_mask(self, image):
        if self.net is None:
            return None, None
        blob = cv2.dnn.blobFromImage(
            image,
            scalefactor=1.0 / 255.0,
            size=(self.model_width, self.model_height),
            mean=(0.0, 0.0, 0.0),
            swapRB=True,
            crop=False,
        )
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_names) if self.output_names else self.net.forward()
        if isinstance(outputs, np.ndarray):
            outputs = [outputs]
        candidates = self.output_candidates(outputs)
        if "lane" not in candidates:
            raise ValueError("ONNX 출력에서 lane segmentation을 찾지 못했습니다.")

        probability = self.binary_probability(candidates["lane"])
        probability = cv2.resize(
            probability,
            (image.shape[1], image.shape[0]),
            interpolation=cv2.INTER_LINEAR,
        )
        mask = probability >= self.lane_threshold
        return mask, probability

    def collect_side_points(self, mask, side: str, top_y: int, bottom_y: int, center_x: float):
        points: List[Tuple[float, float]] = []
        rows = np.linspace(top_y, bottom_y, self.row_samples).astype(np.int32)
        height, width = mask.shape[:2]
        for row in rows:
            y0 = max(0, int(row) - self.row_half_window)
            y1 = min(height, int(row) + self.row_half_window + 1)
            xs = np.where(mask[y0:y1, :].any(axis=0))[0]
            if side == "left":
                xs = xs[xs < center_x]
                if len(xs):
                    x = float(xs.max())
                else:
                    continue
            else:
                xs = xs[xs > center_x]
                if len(xs):
                    x = float(xs.min())
                else:
                    continue
            points.append((x, float(row)))
        return points

    def fit_side(self, points: List[Tuple[float, float]]) -> Tuple[Optional[Fit], float]:
        if len(points) < self.min_side_points:
            return None, math.inf
        values = np.asarray(points, dtype=np.float64)
        x_values = values[:, 0]
        y_values = values[:, 1]
        if np.ptp(y_values) < 20.0:
            return None, math.inf
        degree = min(self.fit_degree, len(points) - 1)
        coefficients = tuple(float(v) for v in np.polyfit(y_values, x_values, degree))
        predicted = np.polyval(coefficients, y_values)
        residual = float(np.median(np.abs(predicted - x_values)))
        return coefficients, residual

    @staticmethod
    def evaluate_fit(fit: Fit, y: float) -> float:
        return float(np.polyval(np.asarray(fit, dtype=np.float64), float(y)))

    def make_lane_measurement(self, mask, probability):
        height, width = mask.shape[:2]
        center_x = width * 0.5
        top_y = int(height * self.roi_top_ratio)
        bottom_y = int(height * self.roi_bottom_ratio)
        roi_y = int(height * self.roi_start_ratio)
        top_y = max(top_y, roi_y)

        left_points = self.collect_side_points(mask, "left", top_y, bottom_y, center_x)
        right_points = self.collect_side_points(mask, "right", top_y, bottom_y, center_x)
        left_fit, left_residual = self.fit_side(left_points)
        right_fit, right_residual = self.fit_side(right_points)

        both_sides = left_fit is not None and right_fit is not None
        lane_width_px: Optional[float] = None
        center_bottom: Optional[float] = None
        center_top: Optional[float] = None
        if both_sides:
            left_bottom = self.evaluate_fit(left_fit, bottom_y)
            right_bottom = self.evaluate_fit(right_fit, bottom_y)
            left_top = self.evaluate_fit(left_fit, top_y)
            right_top = self.evaluate_fit(right_fit, top_y)
            lane_width_px = right_bottom - left_bottom
            if not (0.12 * width <= lane_width_px <= 0.90 * width):
                both_sides = False
            elif max(left_residual, right_residual) > self.max_fit_residual_px:
                both_sides = False
            else:
                center_bottom = 0.5 * (left_bottom + right_bottom)
                center_top = 0.5 * (left_top + right_top)

        if not both_sides:
            fit = left_fit if left_fit is not None else right_fit
            if fit is None:
                return None
            observed = self.evaluate_fit(fit, bottom_y)
            observed_top = self.evaluate_fit(fit, top_y)
            lane_width_px = self.last_lane_width_px or width * self.default_lane_width_px_ratio
            if left_fit is not None:
                center_bottom = observed + 0.5 * lane_width_px
                center_top = observed_top + 0.5 * lane_width_px
            else:
                center_bottom = observed - 0.5 * lane_width_px
                center_top = observed_top - 0.5 * lane_width_px

        if center_bottom is None or center_top is None or lane_width_px is None:
            return None
        if both_sides:
            self.last_lane_width_px = float(lane_width_px)
            self.lane_width_history.append(float(lane_width_px))

        vertical_span = max(float(bottom_y - top_y), 1.0)
        raw_offset = (center_bottom - center_x) * self.lane_width_m / max(lane_width_px, 1.0)
        raw_heading = math.atan2(center_bottom - center_top, vertical_span)
        raw_offset *= self.output_sign
        raw_heading *= self.output_sign

        self.offset_history.append(float(clamp(raw_offset, -10.0, 10.0)))
        self.heading_history.append(float(clamp(raw_heading, -math.pi / 2.0, math.pi / 2.0)))
        offset = median(list(self.offset_history))
        heading = median(list(self.heading_history))

        selected_points = len(left_points) + len(right_points)
        coverage_score = clamp(selected_points / float(max(2 * self.row_samples, 1)), 0.0, 1.0)
        residual = (
            max(left_residual, right_residual)
            if both_sides
            else (left_residual if left_fit is not None else right_residual)
        )
        residual_score = 1.0 if not finite(residual) else clamp(
            1.0 - residual / self.max_fit_residual_px, 0.0, 1.0
        )
        probability_score = float(np.mean(probability[mask])) if np.any(mask) else 0.0
        side_score = 1.0 if both_sides else 0.55
        confidence = clamp(
            side_score
            * (0.55 + 0.45 * coverage_score)
            * (0.55 + 0.45 * residual_score)
            * (0.55 + 0.45 * probability_score),
            0.0,
            1.0,
        )
        return {
            "left_fit": left_fit,
            "right_fit": right_fit,
            "left_points": left_points,
            "right_points": right_points,
            "center_bottom": center_bottom,
            "center_top": center_top,
            "top_y": top_y,
            "bottom_y": bottom_y,
            "roi_y": roi_y,
            "offset": offset,
            "heading": heading,
            "confidence": confidence,
            "both_sides": both_sides,
        }

    def draw_fit(self, image, fit: Optional[Fit], top_y: int, bottom_y: int, color) -> None:
        if fit is None:
            return
        ys = np.linspace(top_y, bottom_y, 30)
        points = []
        width = image.shape[1]
        for y in ys:
            x = self.evaluate_fit(fit, float(y))
            if -width <= x <= 2 * width:
                points.append((int(round(x)), int(round(y))))
        if len(points) >= 2:
            cv2.polylines(image, [np.asarray(points, dtype=np.int32)], False, color, 3)

    def make_debug_image(self, image, mask, result, status: str):
        debug = image.copy()
        height, width = debug.shape[:2]
        if mask is not None:
            tint = np.zeros_like(debug)
            tint[mask] = (255, 120, 0)
            debug = cv2.addWeighted(debug, 0.78, tint, 0.35, 0.0)

        roi_y = int(height * self.roi_start_ratio)
        cv2.line(debug, (0, roi_y), (width - 1, roi_y), (255, 180, 0), 2)
        if result is not None:
            top_y = int(result["top_y"])
            bottom_y = int(result["bottom_y"])
            self.draw_fit(debug, result["left_fit"], top_y, bottom_y, (0, 255, 0))
            self.draw_fit(debug, result["right_fit"], top_y, bottom_y, (0, 255, 0))
            center_bottom = int(round(result["center_bottom"]))
            center_top = int(round(result["center_top"]))
            cv2.circle(debug, (center_bottom, bottom_y), 7, (0, 0, 255), -1)
            cv2.circle(debug, (center_top, top_y), 7, (255, 0, 0), -1)
            cv2.line(debug, (center_top, top_y), (center_bottom, bottom_y), (255, 0, 255), 3)
            cv2.line(debug, (width // 2, top_y), (width // 2, bottom_y), (0, 255, 255), 2)

        cv2.rectangle(debug, (0, 0), (width, 60), (0, 0, 0), -1)
        cv2.putText(
            debug,
            "CAMERA TEAM ONNX  " + status,
            (12, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 0) if status.startswith("VALID") else (0, 165, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug,
            "cyan=lane mask  green=boundary  magenta=centerline  yellow=vehicle center",
            (12, 49),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )
        return debug

    def publish_image(self, source: CompressedImage, image, publisher) -> None:
        if image is None or cv2 is None:
            return
        success, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not success:
            return
        output = CompressedImage()
        output.header = source.header
        output.format = "jpeg"
        output.data = encoded.tobytes()
        publisher.publish(output)

    def publish_mask_image(self, source: CompressedImage, mask) -> None:
        if not self.publish_mask or mask is None:
            return
        visual = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
        visual[mask] = (255, 255, 255)
        self.publish_image(source, visual, self.mask_publisher)

    def callback(self, message: CompressedImage) -> None:
        image = self.decode(message)
        if image is None:
            self.publish_lane(message, False)
            return

        if self.net is None:
            self.publish_lane(message, False)
            if self.publish_debug:
                self.publish_image(
                    message,
                    self.make_debug_image(image, None, None, "INVALID: model unavailable"),
                    self.debug_publisher,
                )
            return

        try:
            mask, probability = self.infer_mask(image)
            self.publish_mask_image(message, mask)
            result = self.make_lane_measurement(mask, probability)
        except Exception as exc:  # pylint: disable=broad-except
            rospy.logwarn_throttle(5.0, "CAMERA팀 lane inference 실패: %s", exc)
            self.publish_lane(message, False)
            if self.publish_debug:
                self.publish_image(
                    message,
                    self.make_debug_image(image, None, None, "INVALID: inference error"),
                    self.debug_publisher,
                )
            return

        if result is None:
            self.publish_lane(message, False)
            status = "INVALID: lane geometry unavailable"
        else:
            self.publish_lane(
                message,
                True,
                (result["offset"], result["heading"], result["confidence"]),
            )
            status = (
                "VALID  confidence=%.2f  offset=%+.2fm  heading=%+.3frad%s"
                % (
                    result["confidence"],
                    result["offset"],
                    result["heading"],
                    "  both-sides" if result["both_sides"] else "  one-side",
                )
            )

        if self.publish_debug:
            self.publish_image(
                message,
                self.make_debug_image(image, mask, result, status),
                self.debug_publisher,
            )


if __name__ == "__main__":
    try:
        CameraTeamLaneNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
