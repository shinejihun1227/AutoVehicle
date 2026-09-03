#!/usr/bin/env python3
"""전방 영상의 지정 ROI에서 신호등 색상 후보를 검출한다.

실제 대회 인식기 전 단계의 연결 검증용 구현이다. ROI와 색상 threshold는
카메라 calibration 및 녹화 영상으로 다시 조정해야 한다.
"""

from __future__ import annotations

import rospy
from morai_perception_msgs.msg import TrafficLight
from sensor_msgs.msg import CompressedImage

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


class TrafficLightDetector:
    def __init__(self) -> None:
        rospy.init_node("traffic_light_detector", anonymous=False)
        self.topic = rospy.get_param("~image_topic", "/camera/front/image/compressed")
        self.output_topic = rospy.get_param("~output_topic", "/detection/traffic_light")
        self.min_pixels = int(rospy.get_param("~min_pixels", 20))
        self.publisher = rospy.Publisher(self.output_topic, TrafficLight, queue_size=2)
        rospy.Subscriber(self.topic, CompressedImage, self.callback, queue_size=2)

    def callback(self, msg: CompressedImage) -> None:
        output = TrafficLight()
        output.header = msg.header
        if not output.header.frame_id:
            output.header.frame_id = "front_camera"
        output.state = "unknown"

        if cv2 is None or np is None:
            self.publisher.publish(output)
            return
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.publisher.publish(output)
            return

        height, width = image.shape[:2]
        # 기본 ROI: 영상 상단 중앙. 실제 신호등 위치에 맞춰 조정한다.
        roi = image[int(height * 0.05):int(height * 0.45), int(width * 0.25):int(width * 0.75)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        red = cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 100, 80]), np.array([10, 255, 255])),
            cv2.inRange(hsv, np.array([170, 100, 80]), np.array([180, 255, 255])),
        )
        yellow = cv2.inRange(hsv, np.array([15, 90, 100]), np.array([40, 255, 255]))
        green = cv2.inRange(hsv, np.array([40, 70, 60]), np.array([95, 255, 255]))
        scores = {"red": int(cv2.countNonZero(red)), "yellow": int(cv2.countNonZero(yellow)), "green": int(cv2.countNonZero(green))}
        state, score = max(scores.items(), key=lambda item: item[1])
        if score >= self.min_pixels:
            output.state = state
            output.confidence = min(1.0, score / float(max(100, self.min_pixels * 10)))
            output.valid = True
        self.publisher.publish(output)


if __name__ == "__main__":
    try:
        TrafficLightDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
