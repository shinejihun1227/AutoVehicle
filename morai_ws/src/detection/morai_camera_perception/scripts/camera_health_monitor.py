#!/usr/bin/env python3
"""카메라 4대의 수신 주기·timestamp·frame_id·기본 해상도를 진단한다."""

from __future__ import annotations

import time
from typing import Dict, Optional

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CompressedImage

try:
    import cv2
    import numpy as np
except ImportError:  # 상태 진단은 OpenCV 없이도 동작하도록 한다.
    cv2 = None
    np = None


class CameraHealthMonitor:
    def __init__(self) -> None:
        rospy.init_node("camera_health_monitor", anonymous=False)
        self.topics = {
            "front": rospy.get_param("~front_topic", "/camera/front/image/compressed"),
            "left": rospy.get_param("~left_topic", "/camera/left/image/compressed"),
            "right": rospy.get_param("~right_topic", "/camera/right/image/compressed"),
            "aux": rospy.get_param("~aux_topic", "/camera/aux/image/compressed"),
        }
        self.timeout_sec = float(rospy.get_param("~timeout_sec", 1.0))
        self.last_receive: Dict[str, float] = {name: 0.0 for name in self.topics}
        self.last_stamp: Dict[str, float] = {name: 0.0 for name in self.topics}
        self.count: Dict[str, int] = {name: 0 for name in self.topics}
        self.frame_id: Dict[str, str] = {name: "" for name in self.topics}
        self.size: Dict[str, str] = {name: "unknown" for name in self.topics}

        for name, topic in self.topics.items():
            rospy.Subscriber(topic, CompressedImage, self.callback, callback_args=name, queue_size=2)
        self.publisher = rospy.Publisher("/detection/camera_status", DiagnosticArray, queue_size=2)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_status)

    def callback(self, msg: CompressedImage, name: str) -> None:
        self.last_receive[name] = time.monotonic()
        self.last_stamp[name] = msg.header.stamp.to_sec()
        self.frame_id[name] = msg.header.frame_id
        self.count[name] += 1
        if cv2 is not None and np is not None:
            try:
                image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is not None:
                    self.size[name] = "%dx%d" % (image.shape[1], image.shape[0])
            except Exception as exc:  # 진단 노드는 영상 하나의 오류로 죽지 않는다.
                self.size[name] = "decode_error:%s" % exc

    def publish_status(self, _event) -> None:
        now = time.monotonic()
        report = DiagnosticArray()
        report.header.stamp = rospy.Time.now()
        for name in self.topics:
            status = DiagnosticStatus()
            status.name = "camera/%s" % name
            status.hardware_id = name
            age = now - self.last_receive[name] if self.last_receive[name] > 0.0 else float("inf")
            if age <= self.timeout_sec:
                status.level = DiagnosticStatus.OK
                status.message = "receiving"
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = "no recent frame"
            status.values = [
                KeyValue(key="topic", value=self.topics[name]),
                KeyValue(key="age_sec", value="%.3f" % age),
                KeyValue(key="frames", value=str(self.count[name])),
                KeyValue(key="frame_id", value=self.frame_id[name] or "unset"),
                KeyValue(key="resolution", value=self.size[name]),
                KeyValue(key="header_stamp", value="%.6f" % self.last_stamp[name]),
            ]
            report.status.append(status)
        self.publisher.publish(report)


if __name__ == "__main__":
    try:
        CameraHealthMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
