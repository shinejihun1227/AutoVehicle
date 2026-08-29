#!/usr/bin/env python3
"""전방 CompressedImage의 고전적 차선 후보 검출.

대회용 학습 모델이 아니라 카메라·좌표·토픽 연결을 검증하기 위한 초기 구현이다.
meters_per_pixel과 ROI는 카메라 calibration 이후 다시 보정해야 한다.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rospy
from morai_perception_msgs.msg import LaneDetection
from sensor_msgs.msg import CompressedImage

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


class CameraLaneDetector:
    def __init__(self) -> None:
        rospy.init_node("camera_lane_detector", anonymous=False)
        self.topic = rospy.get_param("~image_topic", "/camera/front/image/compressed")
        self.output_topic = rospy.get_param("~output_topic", "/detection/lane")
        self.roi_start_ratio = float(rospy.get_param("~roi_start_ratio", 0.45))
        self.meters_per_pixel = float(rospy.get_param("~meters_per_pixel", 0.01))
        self.min_line_count = int(rospy.get_param("~min_line_count", 2))
        self.publisher = rospy.Publisher(self.output_topic, LaneDetection, queue_size=2)
        rospy.Subscriber(self.topic, CompressedImage, self.callback, queue_size=2)

    def decode(self, msg: CompressedImage):
        if cv2 is None or np is None:
            return None
        data = np.frombuffer(msg.data, dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)

    def line_x_at_y(self, line: Tuple[int, int, int, int], y: float) -> float:
        x1, y1, x2, y2 = line
        if abs(y2 - y1) < 1.0:
            return float((x1 + x2) / 2.0)
        return float(x1 + (y - y1) * (x2 - x1) / (y2 - y1))

    def detect_lines(self, image) -> Tuple[List[Tuple[int, int, int, int]], List[Tuple[int, int, int, int]]]:
        height, width = image.shape[:2]
        roi_y = int(height * self.roi_start_ratio)
        roi = image[roi_y:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        white = cv2.inRange(hsv, np.array([0, 0, 150]), np.array([180, 90, 255]))
        yellow = cv2.inRange(hsv, np.array([12, 50, 80]), np.array([45, 255, 255]))
        mask = cv2.bitwise_or(white, yellow)
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180.0, threshold=25,
            minLineLength=max(20, width // 12), maxLineGap=35,
        )
        left: List[Tuple[int, int, int, int]] = []
        right: List[Tuple[int, int, int, int]] = []
        if lines is None:
            return left, right
        for raw in lines[:, 0, :]:
            x1, y1, x2, y2 = [int(value) for value in raw]
            dx = x2 - x1
            dy = y2 - y1
            if abs(dx) < 2 or abs(dy) < 5:
                continue
            slope = dy / float(dx)
            line = (x1, y1 + roi_y, x2, y2 + roi_y)
            midpoint = (x1 + x2) / 2.0
            if slope < -0.35 and midpoint < width * 0.65:
                left.append(line)
            elif slope > 0.35 and midpoint > width * 0.35:
                right.append(line)
        return left, right

    def callback(self, msg: CompressedImage) -> None:
        output = LaneDetection()
        output.header = msg.header
        if not output.header.frame_id:
            output.header.frame_id = "front_camera"

        image = self.decode(msg)
        if image is None:
            self.publisher.publish(output)
            return

        height, width = image.shape[:2]
        left, right = self.detect_lines(image)
        if len(left) + len(right) < self.min_line_count:
            self.publisher.publish(output)
            return

        bottom_y = height * 0.95
        top_y = height * 0.60
        candidates = []
        if left:
            candidates.append(("left", left))
        if right:
            candidates.append(("right", right))

        bottom_x = []
        top_x = []
        for _, lines in candidates:
            bottom_x.append(sum(self.line_x_at_y(line, bottom_y) for line in lines) / len(lines))
            top_x.append(sum(self.line_x_at_y(line, top_y) for line in lines) / len(lines))

        if len(bottom_x) >= 2:
            lane_left = min(bottom_x)
            lane_right = max(bottom_x)
            center_bottom = (lane_left + lane_right) / 2.0
            center_top = (min(top_x) + max(top_x)) / 2.0
            confidence = min(1.0, 0.25 * (len(left) + len(right)))
        else:
            center_bottom = bottom_x[0]
            center_top = top_x[0]
            confidence = 0.25

        output.lateral_offset_m = float((center_bottom - width / 2.0) * self.meters_per_pixel)
        output.heading_error_rad = float(
            math.atan2(center_bottom - center_top, max(bottom_y - top_y, 1.0))
        )
        output.confidence = confidence
        output.valid = confidence > 0.0
        self.publisher.publish(output)


if __name__ == "__main__":
    try:
        CameraLaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
