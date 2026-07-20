#!/usr/bin/env python3
import time

import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String

try:
    import cv2
except ImportError:
    cv2 = None


class CameraPerceptionNode:
    def __init__(self):
        if cv2 is None:
            raise rospy.ROSInitException("python3-opencv is required: sudo apt install -y python3-opencv")

        ns = "/camera_perception"
        self.image_topic = rospy.get_param(ns + "/image_topic", "/sensors/camera/front/compressed")
        self.lane_offset_topic = rospy.get_param(ns + "/lane_offset_topic", "/perception/camera/lane_offset_px")
        self.traffic_light_topic = rospy.get_param(ns + "/traffic_light_topic", "/perception/camera/traffic_light_state")
        self.stop_line_topic = rospy.get_param(ns + "/stop_line_topic", "/perception/camera/stop_line_detected")
        self.debug_topic = rospy.get_param(ns + "/debug_topic", "/perception/camera/debug/compressed")
        self.publish_debug = bool(rospy.get_param(ns + "/publish_debug_image", True))
        self.process_rate_hz = float(rospy.get_param(ns + "/process_rate_hz", 15.0))
        self.resize_width = int(rospy.get_param(ns + "/resize_width", 640))
        self.lane_roi_y_start_ratio = float(rospy.get_param(ns + "/lane_roi_y_start_ratio", 0.55))
        self.traffic_roi_y_end_ratio = float(rospy.get_param(ns + "/traffic_roi_y_end_ratio", 0.45))
        self.stop_line_roi_y_start_ratio = float(rospy.get_param(ns + "/stop_line_roi_y_start_ratio", 0.65))
        self.min_lane_pixels = int(rospy.get_param(ns + "/min_lane_pixels", 400))
        self.min_traffic_pixels = int(rospy.get_param(ns + "/min_traffic_pixels", 60))
        self.min_stop_line_pixels = int(rospy.get_param(ns + "/min_stop_line_pixels", 1500))

        self.last_process_time = 0.0
        self.lane_pub = rospy.Publisher(self.lane_offset_topic, Float32, queue_size=1)
        self.traffic_pub = rospy.Publisher(self.traffic_light_topic, String, queue_size=1)
        self.stop_line_pub = rospy.Publisher(self.stop_line_topic, Bool, queue_size=1)
        self.debug_pub = rospy.Publisher(self.debug_topic, CompressedImage, queue_size=1)
        rospy.Subscriber(self.image_topic, CompressedImage, self._image_cb, queue_size=1)
        rospy.loginfo("Camera perception subscribed to %s", self.image_topic)

    def _image_cb(self, msg):
        if not self._rate_limit_allows_process():
            return

        image = self._decode_image(msg)
        if image is None:
            return

        image = self._resize(image)
        lane_offset, lane_mask = self._estimate_lane_offset(image)
        traffic_state = self._estimate_traffic_light(image)
        stop_line = self._estimate_stop_line(image)

        self.lane_pub.publish(Float32(lane_offset))
        self.traffic_pub.publish(String(traffic_state))
        self.stop_line_pub.publish(Bool(stop_line))

        if self.publish_debug:
            debug = self._draw_debug(image, lane_offset, traffic_state, stop_line, lane_mask)
            self._publish_debug(debug, msg.header)

    def _decode_image(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if image is None:
            rospy.logwarn_throttle(2.0, "failed to decode compressed camera image")
        return image

    def _resize(self, image):
        if self.resize_width <= 0 or image.shape[1] == self.resize_width:
            return image
        scale = float(self.resize_width) / float(image.shape[1])
        height = max(1, int(image.shape[0] * scale))
        return cv2.resize(image, (self.resize_width, height), interpolation=cv2.INTER_AREA)

    def _estimate_lane_offset(self, image):
        height, width = image.shape[:2]
        y0 = int(height * self.lane_roi_y_start_ratio)
        roi = image[y0:height, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        white = cv2.inRange(hsv, np.array([0, 0, 180]), np.array([180, 70, 255]))
        yellow = cv2.inRange(hsv, np.array([15, 60, 80]), np.array([40, 255, 255]))
        mask = cv2.bitwise_or(white, yellow)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        ys, xs = np.where(mask > 0)
        if len(xs) < self.min_lane_pixels:
            return 0.0, mask

        left_x = xs[xs < width * 0.5]
        right_x = xs[xs >= width * 0.5]
        center_x = width * 0.5

        if len(left_x) >= self.min_lane_pixels * 0.25 and len(right_x) >= self.min_lane_pixels * 0.25:
            lane_center = (float(np.median(left_x)) + float(np.median(right_x))) * 0.5
        else:
            lane_center = float(np.median(xs))

        return lane_center - center_x, mask

    def _estimate_traffic_light(self, image):
        height, width = image.shape[:2]
        roi = image[: int(height * self.traffic_roi_y_end_ratio), :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        red2 = cv2.inRange(hsv, np.array([170, 100, 100]), np.array([180, 255, 255]))
        yellow = cv2.inRange(hsv, np.array([18, 90, 120]), np.array([38, 255, 255]))
        green = cv2.inRange(hsv, np.array([45, 70, 80]), np.array([90, 255, 255]))

        counts = {
            "red": int(cv2.countNonZero(cv2.bitwise_or(red1, red2))),
            "yellow": int(cv2.countNonZero(yellow)),
            "green": int(cv2.countNonZero(green)),
        }
        color, count = max(counts.items(), key=lambda item: item[1])
        if count < self.min_traffic_pixels:
            return "unknown"
        return color

    def _estimate_stop_line(self, image):
        height, width = image.shape[:2]
        y0 = int(height * self.stop_line_roi_y_start_ratio)
        roi = image[y0:height, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, np.array([0, 0, 190]), np.array([180, 60, 255]))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 3))
        horizontal = cv2.morphologyEx(white, cv2.MORPH_OPEN, kernel)
        return int(cv2.countNonZero(horizontal)) >= self.min_stop_line_pixels

    def _draw_debug(self, image, lane_offset, traffic_state, stop_line, lane_mask):
        debug = image.copy()
        height, width = debug.shape[:2]
        lane_center = int(width * 0.5 + lane_offset)
        cv2.line(debug, (width // 2, 0), (width // 2, height), (255, 255, 255), 1)
        cv2.line(debug, (lane_center, int(height * self.lane_roi_y_start_ratio)), (lane_center, height), (0, 255, 255), 2)
        cv2.putText(debug, "lane_offset_px=%.1f" % lane_offset, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(debug, "traffic=%s stop_line=%s" % (traffic_state, stop_line), (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        return debug

    def _publish_debug(self, image, header):
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return
        msg = CompressedImage()
        msg.header = header
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.debug_pub.publish(msg)

    def _rate_limit_allows_process(self):
        if self.process_rate_hz <= 0.0:
            return True
        now = time.time()
        if now - self.last_process_time < 1.0 / self.process_rate_hz:
            return False
        self.last_process_time = now
        return True


if __name__ == "__main__":
    rospy.init_node("camera_perception")
    CameraPerceptionNode()
    rospy.spin()
