#!/usr/bin/env python3
"""CompressedImage 토픽을 OpenCV 창으로 표시하는 ROS 디버그 뷰어.

카메라 통합 주행을 실행할 때 원본 영상과 차선 오버레이 영상을 확인하기
위한 용도이다. 제어 토픽을 발행하지 않으며, GUI가 없는 환경에서는 창을
열지 못했다는 경고만 출력하고 ROS 노드는 계속 실행한다.
"""

from __future__ import annotations

import time

import rospy
from sensor_msgs.msg import CompressedImage

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


class CompressedImageViewer:
    def __init__(self) -> None:
        rospy.init_node("compressed_image_viewer", anonymous=False)
        self.topic = rospy.get_param("~image_topic", "/camera/front/image/compressed")
        self.window_name = rospy.get_param("~window_name", "MORAI Camera")
        self.max_width = max(320, int(rospy.get_param("~max_width", 1280)))
        self.max_height = max(240, int(rospy.get_param("~max_height", 720)))
        self.wait_key_ms = max(1, int(rospy.get_param("~wait_key_ms", 1)))
        self.frame = None
        self.frame_stamp = 0.0
        self.gui_available = cv2 is not None and np is not None
        self.warned_gui = False

        rospy.Subscriber(self.topic, CompressedImage, self.callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.03), self.render)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "Camera viewer: topic=%s window=%s",
            self.topic,
            self.window_name,
        )

    def callback(self, message: CompressedImage) -> None:
        if not self.gui_available:
            return
        try:
            data = np.frombuffer(message.data, dtype=np.uint8)
            decoded = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if decoded is not None:
                self.frame = decoded
                self.frame_stamp = time.monotonic()
        except Exception as exc:  # pylint: disable=broad-except
            rospy.logwarn_throttle(5.0, "카메라 영상 디코딩 실패: %s", exc)

    def fit_to_window(self, image):
        height, width = image.shape[:2]
        scale = min(
            1.0,
            self.max_width / float(max(width, 1)),
            self.max_height / float(max(height, 1)),
        )
        if scale >= 0.999:
            return image
        return cv2.resize(
            image,
            (max(1, int(width * scale)), max(1, int(height * scale))),
            interpolation=cv2.INTER_AREA,
        )

    def render(self, _event) -> None:
        if not self.gui_available:
            return
        if self.frame is None:
            return
        try:
            frame = self.fit_to_window(self.frame.copy())
            age = max(0.0, time.monotonic() - self.frame_stamp)
            cv2.putText(
                frame,
                "topic: %s  age: %.3fs" % (self.topic, age),
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(self.wait_key_ms) & 0xFF
            if key in (27, ord("q")):
                rospy.signal_shutdown("camera viewer closed by user")
        except cv2.error as exc:
            if not self.warned_gui:
                rospy.logwarn(
                    "카메라 창을 열 수 없습니다. DISPLAY/X11 설정을 확인하세요: %s",
                    exc,
                )
                self.warned_gui = True

    def shutdown(self) -> None:
        if self.gui_available:
            try:
                cv2.destroyWindow(self.window_name)
            except cv2.error:
                pass


if __name__ == "__main__":
    try:
        CompressedImageViewer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
