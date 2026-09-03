#!/usr/bin/env python3
"""전방 카메라 차선 검출 및 camera fallback용 오차 생성기.

입력은 MORAI의 CompressedImage이며, 색상 mask + ROI + Hough 선분을 사용한다.
양쪽 차선이 모두 보이는 결과만 높은 confidence를 부여하고, 최근 결과를 짧게
median/EMA로 완화한다. 차선 검출은 카메라 calibration 없이는 완벽한 meter
좌표를 만들 수 없으므로 lane_width_m과 meters_per_pixel을 파라미터로 둔다.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, List, Optional, Tuple

import rospy
from morai_perception_msgs.msg import LaneDetection
from sensor_msgs.msg import CompressedImage

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


Line = Tuple[int, int, int, int]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class RobustCameraLaneDetector:
    def __init__(self) -> None:
        rospy.init_node("camera_lane_detector_robust", anonymous=False)
        self.image_topic = rospy.get_param(
            "~image_topic", "/camera/front/image/compressed"
        )
        self.output_topic = rospy.get_param("~output_topic", "/detection/lane")
        self.debug_topic = rospy.get_param(
            "~debug_topic", "/detection/lane_debug/compressed"
        )
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))
        self.roi_start_ratio = clamp(
            float(rospy.get_param("~roi_start_ratio", 0.45)), 0.0, 0.95
        )
        self.roi_top_ratio = clamp(
            float(rospy.get_param("~roi_top_ratio", 0.58)), 0.05, 0.95
        )
        self.roi_bottom_ratio = clamp(
            float(rospy.get_param("~roi_bottom_ratio", 0.98)), 0.5, 1.0
        )
        self.min_line_length_ratio = max(
            0.01, float(rospy.get_param("~min_line_length_ratio", 0.04))
        )
        self.max_line_gap = max(1, int(rospy.get_param("~max_line_gap_px", 35)))
        self.hough_threshold = max(
            5, int(rospy.get_param("~hough_threshold", 25))
        )
        self.min_line_count = max(1, int(rospy.get_param("~min_line_count", 2)))
        self.smooth_alpha = clamp(
            float(rospy.get_param("~smooth_alpha", 0.35)), 0.01, 1.0
        )
        self.lane_filter_window = max(
            1, int(rospy.get_param("~lane_filter_window", 5))
        )
        self.lane_width_m = max(
            0.5, float(rospy.get_param("~lane_width_m", 3.5))
        )
        self.meters_per_pixel = max(
            1e-5, float(rospy.get_param("~meters_per_pixel", 0.01))
        )

        self.publisher = rospy.Publisher(self.output_topic, LaneDetection, queue_size=2)
        self.debug_publisher = rospy.Publisher(
            self.debug_topic, CompressedImage, queue_size=1
        )
        rospy.Subscriber(
            self.image_topic, CompressedImage, self.callback, queue_size=2
        )

        self.smoothed_bottom: Optional[float] = None
        self.smoothed_top: Optional[float] = None
        self.last_lane_width_px: Optional[float] = None
        self.offset_history: Deque[float] = deque(maxlen=self.lane_filter_window)
        self.heading_history: Deque[float] = deque(maxlen=self.lane_filter_window)

        if cv2 is None or np is None:
            rospy.logerr(
                "OpenCV/numpy가 없어 camera_lane_detector_robust가 invalid 결과만 발행한다."
            )

        rospy.loginfo(
            "Robust camera lane: image=%s output=%s debug=%s",
            self.image_topic,
            self.output_topic,
            self.debug_topic if self.publish_debug else "disabled",
        )

    @staticmethod
    def decode(message: CompressedImage):
        if cv2 is None or np is None:
            return None
        data = np.frombuffer(message.data, dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)

    def roi_polygon(self, width: int, height: int):
        top_y = int(height * self.roi_top_ratio)
        bottom_y = int(height * self.roi_bottom_ratio)
        top_half = int(width * 0.22)
        return np.array(
            [
                [max(0, width // 2 - top_half), top_y],
                [min(width - 1, width // 2 + top_half), top_y],
                [width - 1, bottom_y],
                [0, bottom_y],
            ],
            dtype=np.int32,
        )

    @staticmethod
    def fit_x_by_y(lines: List[Line]) -> Optional[Tuple[float, float]]:
        if not lines:
            return None
        xs: List[float] = []
        ys: List[float] = []
        for x1, y1, x2, y2 in lines:
            xs.extend([float(x1), float(x2)])
            ys.extend([float(y1), float(y2)])
        if len(xs) < 2 or max(ys) - min(ys) < 10.0:
            return None
        coefficients = np.polyfit(np.asarray(ys), np.asarray(xs), 1)
        return float(coefficients[0]), float(coefficients[1])

    @staticmethod
    def x_at_y(fit: Tuple[float, float], y: float) -> float:
        return fit[0] * float(y) + fit[1]

    def detect_lines(self, image) -> Tuple[List[Line], List[Line], object]:
        height, width = image.shape[:2]
        roi_y = int(height * self.roi_start_ratio)
        roi = image[roi_y:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # MORAI 장면의 밝은 흰색/황색 차선 후보를 잡고 작은 구멍을 닫는다.
        white = cv2.inRange(hsv, np.array([0, 0, 145]), np.array([180, 110, 255]))
        yellow = cv2.inRange(hsv, np.array([10, 45, 70]), np.array([45, 255, 255]))
        mask = cv2.bitwise_or(white, yellow)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        polygon = self.roi_polygon(width, height)
        roi_mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(roi_mask, [polygon], 255)
        full_mask = np.zeros((height, width), dtype=np.uint8)
        full_mask[roi_y:, :] = mask
        full_mask = cv2.bitwise_and(full_mask, roi_mask)
        edges = cv2.Canny(full_mask, 50, 150)

        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180.0,
            threshold=self.hough_threshold,
            minLineLength=max(20, int(width * self.min_line_length_ratio)),
            maxLineGap=self.max_line_gap,
        )
        left: List[Line] = []
        right: List[Line] = []
        if lines is not None:
            center_x = width * 0.5
            for raw in lines[:, 0, :]:
                x1, y1, x2, y2 = [int(value) for value in raw]
                y1 += roi_y
                y2 += roi_y
                dx = x2 - x1
                dy = y2 - y1
                if abs(dx) < 2 or abs(dy) < 8:
                    continue
                slope = dy / float(dx)
                bottom_y = height * self.roi_bottom_ratio
                bottom_x = x1 + (bottom_y - y1) * dx / float(dy)
                line = (x1, y1, x2, y2)
                if slope < -0.35 and bottom_x < center_x:
                    left.append(line)
                elif slope > 0.35 and bottom_x > center_x:
                    right.append(line)

        debug = cv2.cvtColor(full_mask, cv2.COLOR_GRAY2BGR)
        return left, right, debug

    def publish_invalid(self, message: CompressedImage, debug_image=None) -> None:
        output = LaneDetection()
        output.header = message.header
        if not output.header.frame_id:
            output.header.frame_id = "front_camera"
        output.valid = False
        output.confidence = 0.0
        self.publisher.publish(output)
        if debug_image is not None:
            self.publish_debug_image(message, debug_image)

    def publish_debug_image(self, message: CompressedImage, image) -> None:
        if not self.publish_debug or cv2 is None:
            return
        success, encoded = cv2.imencode(".jpg", image)
        if not success:
            return
        debug = CompressedImage()
        debug.header = message.header
        debug.format = "jpeg"
        debug.data = encoded.tobytes()
        self.debug_publisher.publish(debug)

    def callback(self, message: CompressedImage) -> None:
        image = self.decode(message)
        if image is None:
            self.publish_invalid(message)
            return

        height, width = image.shape[:2]
        left_lines, right_lines, debug = self.detect_lines(image)
        if len(left_lines) + len(right_lines) < self.min_line_count:
            self.publish_invalid(message, debug)
            return

        left_fit = self.fit_x_by_y(left_lines)
        right_fit = self.fit_x_by_y(right_lines)
        bottom_y = height * self.roi_bottom_ratio
        top_y = height * self.roi_top_ratio
        bottom_x: Optional[float] = None
        top_x: Optional[float] = None
        lane_width_px: Optional[float] = None
        both_sides = left_fit is not None and right_fit is not None

        if both_sides:
            left_bottom = self.x_at_y(left_fit, bottom_y)
            right_bottom = self.x_at_y(right_fit, bottom_y)
            left_top = self.x_at_y(left_fit, top_y)
            right_top = self.x_at_y(right_fit, top_y)
            lane_width_px = right_bottom - left_bottom
            if not 0.15 * width <= lane_width_px <= 0.95 * width:
                both_sides = False
            else:
                bottom_x = 0.5 * (left_bottom + right_bottom)
                top_x = 0.5 * (left_top + right_top)
                self.last_lane_width_px = lane_width_px

        if not both_sides:
            # 한쪽만 보이는 프레임은 출력은 하되 confidence를 낮게 주어
            # fallback 제어기가 단독 주행에 사용하지 않도록 한다.
            fit = left_fit if left_fit is not None else right_fit
            if fit is None:
                self.publish_invalid(message, debug)
                return
            detected_bottom = self.x_at_y(fit, bottom_y)
            detected_top = self.x_at_y(fit, top_y)
            width_guess = self.last_lane_width_px or width * 0.35
            if left_fit is not None:
                bottom_x = detected_bottom + 0.5 * width_guess
                top_x = detected_top + 0.5 * width_guess
            else:
                bottom_x = detected_bottom - 0.5 * width_guess
                top_x = detected_top - 0.5 * width_guess
            lane_width_px = width_guess

        if bottom_x is None or top_x is None or lane_width_px is None:
            self.publish_invalid(message, debug)
            return

        if self.smoothed_bottom is None:
            self.smoothed_bottom = bottom_x
            self.smoothed_top = top_x
        else:
            alpha = self.smooth_alpha
            self.smoothed_bottom = alpha * bottom_x + (1.0 - alpha) * self.smoothed_bottom
            self.smoothed_top = alpha * top_x + (1.0 - alpha) * self.smoothed_top

        center_bottom = float(self.smoothed_bottom)
        center_top = float(self.smoothed_top)
        vertical_span = max(bottom_y - top_y, 1.0)
        heading_error = math.atan2(center_bottom - center_top, vertical_span)
        if both_sides:
            lateral_offset = (center_bottom - width * 0.5) * self.lane_width_m / max(lane_width_px, 1.0)
            confidence = 0.78 + 0.04 * min(1.0, min(len(left_lines), len(right_lines)) / 3.0)
        else:
            lateral_offset = (center_bottom - width * 0.5) * self.meters_per_pixel
            confidence = 0.42

        lateral_offset = float(clamp(lateral_offset, -10.0, 10.0))
        heading_error = float(clamp(heading_error, -math.pi / 2.0, math.pi / 2.0))
        self.offset_history.append(lateral_offset)
        self.heading_history.append(heading_error)

        output = LaneDetection()
        output.header = message.header
        if not output.header.frame_id:
            output.header.frame_id = "front_camera"
        output.lateral_offset_m = float(sum(self.offset_history) / len(self.offset_history))
        output.heading_error_rad = float(sum(self.heading_history) / len(self.heading_history))
        output.confidence = float(clamp(confidence, 0.0, 1.0))
        output.valid = True
        self.publisher.publish(output)

        debug_color = cv2.cvtColor(debug, cv2.COLOR_BGR2RGB)
        for line in left_lines + right_lines:
            x1, y1, x2, y2 = line
            cv2.line(debug_color, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(debug_color, (int(center_bottom), int(bottom_y)), 6, (0, 0, 255), -1)
        cv2.circle(debug_color, (int(center_top), int(top_y)), 6, (255, 0, 0), -1)
        cv2.line(
            debug_color,
            (int(width * 0.5), int(top_y)),
            (int(width * 0.5), int(bottom_y)),
            (255, 255, 0),
            2,
        )
        self.publish_debug_image(message, debug_color)


if __name__ == "__main__":
    try:
        RobustCameraLaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
