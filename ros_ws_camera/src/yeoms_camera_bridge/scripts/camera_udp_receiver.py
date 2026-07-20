#!/usr/bin/env python3
import os
import socket
import time

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"


class CameraUdpReceiver:
    def __init__(self):
        ns = "/camera_udp_receiver"
        self.bind_ip = rospy.get_param(ns + "/bind_ip", "0.0.0.0")
        self.camera_port = int(rospy.get_param(ns + "/camera_port", 1001))
        self.image_topic = rospy.get_param(ns + "/image_topic", "/sensors/camera/front/compressed")
        self.frame_info_topic = rospy.get_param(ns + "/frame_info_topic", "/perception/camera/frame_info")
        self.frame_id = rospy.get_param(ns + "/frame_id", "front_camera")
        self.rate_limit_hz = float(rospy.get_param(ns + "/publish_rate_limit_hz", 30.0))
        self.socket_timeout = float(rospy.get_param(ns + "/socket_timeout_sec", 1.0))
        self.max_frame_bytes = int(rospy.get_param(ns + "/max_frame_bytes", 2000000))
        self.save_samples = bool(rospy.get_param(ns + "/save_samples", False))
        self.sample_dir = os.path.expanduser(rospy.get_param(ns + "/sample_dir", "/tmp/morai_camera_samples"))
        self.sample_every_n = max(1, int(rospy.get_param(ns + "/sample_every_n_frames", 30)))

        self.image_pub = rospy.Publisher(self.image_topic, CompressedImage, queue_size=1)
        self.info_pub = rospy.Publisher(self.frame_info_topic, String, queue_size=10)
        self.buffer = bytearray()
        self.frame_count = 0
        self.packet_count = 0
        self.last_publish_time = 0.0

        if self.save_samples:
            os.makedirs(self.sample_dir, exist_ok=True)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_ip, self.camera_port))
        self.sock.settimeout(self.socket_timeout)
        rospy.loginfo("Camera UDP receiver listening on %s:%d", self.bind_ip, self.camera_port)

    def run(self):
        while not rospy.is_shutdown():
            try:
                payload, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                rospy.logwarn_throttle(2.0, "waiting for camera UDP packets on port %d", self.camera_port)
                continue

            self.packet_count += 1
            frame = self._extract_jpeg(payload)
            if frame is None:
                continue
            if not self._rate_limit_allows_publish():
                continue
            self._publish_frame(frame, addr)

    def _extract_jpeg(self, payload):
        start = payload.find(JPEG_SOI)
        end = payload.find(JPEG_EOI)

        if start >= 0 and end > start:
            return payload[start:end + len(JPEG_EOI)]

        if start >= 0:
            self.buffer = bytearray(payload[start:])
            return None

        if self.buffer:
            self.buffer.extend(payload)
            if len(self.buffer) > self.max_frame_bytes:
                rospy.logwarn("camera JPEG buffer exceeded %d bytes; dropping partial frame", self.max_frame_bytes)
                self.buffer = bytearray()
                return None

            end = self.buffer.find(JPEG_EOI)
            if end >= 0:
                frame = bytes(self.buffer[:end + len(JPEG_EOI)])
                self.buffer = bytearray()
                return frame

        return None

    def _rate_limit_allows_publish(self):
        if self.rate_limit_hz <= 0.0:
            return True
        now = time.time()
        min_period = 1.0 / self.rate_limit_hz
        if now - self.last_publish_time < min_period:
            return False
        self.last_publish_time = now
        return True

    def _publish_frame(self, jpeg_bytes, addr):
        now = rospy.Time.now()
        msg = CompressedImage()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = jpeg_bytes
        self.image_pub.publish(msg)

        self.frame_count += 1
        info = "frame=%d packets=%d bytes=%d from=%s:%d topic=%s" % (
            self.frame_count,
            self.packet_count,
            len(jpeg_bytes),
            addr[0],
            addr[1],
            self.image_topic,
        )
        self.info_pub.publish(String(info))
        rospy.loginfo_throttle(1.0, info)

        if self.save_samples and self.frame_count % self.sample_every_n == 0:
            path = os.path.join(self.sample_dir, "front_camera_%06d.jpg" % self.frame_count)
            with open(path, "wb") as fp:
                fp.write(jpeg_bytes)


if __name__ == "__main__":
    rospy.init_node("camera_udp_receiver")
    CameraUdpReceiver().run()
