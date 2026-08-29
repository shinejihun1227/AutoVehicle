#!/usr/bin/env python3
"""카메라 장애물 검출 인터페이스.

학습 모델이 연결되기 전에는 오검출로 차량을 멈추지 않도록 빈 결과를 발행한다.
후속 모델은 동일한 ObstacleArray 계약으로 교체한다.
"""

from __future__ import annotations

import rospy
from morai_perception_msgs.msg import ObstacleArray
from sensor_msgs.msg import CompressedImage


class CameraObstacleDetector:
    def __init__(self) -> None:
        rospy.init_node("camera_obstacle_detector", anonymous=False)
        self.topic = rospy.get_param("~image_topic", "/camera/front/image/compressed")
        self.output_topic = rospy.get_param("~output_topic", "/detection/camera_obstacles")
        self.publisher = rospy.Publisher(self.output_topic, ObstacleArray, queue_size=2)
        rospy.Subscriber(self.topic, CompressedImage, self.callback, queue_size=2)
        rospy.logwarn("camera_obstacle_detector is an interface stub; no camera obstacle model is enabled")

    def callback(self, msg: CompressedImage) -> None:
        output = ObstacleArray()
        output.header = msg.header
        if not output.header.frame_id:
            output.header.frame_id = "front_camera"
        self.publisher.publish(output)


if __name__ == "__main__":
    try:
        CameraObstacleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
