#!/usr/bin/env python3
import math
import socket
import struct

import rospy
import tf.transformations
from sensor_msgs.msg import Imu
from std_msgs.msg import String


HEADER = b"#IMUData$"


class UdpImuToTopic:
    def __init__(self):
        self.bind_ip = rospy.get_param("/udp_bridge/bind_ip", "0.0.0.0")
        self.port = int(rospy.get_param("/udp_bridge/imu_adapter/imu_port", 4001))
        self.frame_id = rospy.get_param("/udp_bridge/imu_adapter/frame_id", "imu_link")
        self.topic = rospy.get_param("/udp_bridge/imu_adapter/topic", "/udp_bridge/imu")
        self.debug_topic = rospy.get_param("/udp_bridge/imu_adapter/debug_topic", "/udp_bridge/imu_debug")

        self.imu_pub = rospy.Publisher(self.topic, Imu, queue_size=10)
        self.debug_pub = rospy.Publisher(self.debug_topic, String, queue_size=10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_ip, self.port))
        self.sock.settimeout(0.5)
        rospy.loginfo("IMU UDP parser listening on %s:%s", self.bind_ip, self.port)

    def run(self):
        while not rospy.is_shutdown():
            try:
                payload, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                rospy.logwarn_throttle(2.0, "waiting for IMU UDP packets on port %s", self.port)
                continue

            try:
                imu, skip = self._parse_payload(payload)
            except ValueError as exc:
                rospy.logwarn_throttle(1.0, "invalid IMU packet from %s: %s", addr[0], exc)
                continue

            self.imu_pub.publish(imu)
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
            )
            self.debug_pub.publish(
                String(
                    "skip=%d qx=%.6f qy=%.6f qz=%.6f qw=%.6f roll=%.6f pitch=%.6f yaw=%.6f "
                    "gyro_x=%.6f gyro_y=%.6f gyro_z=%.6f accel_x=%.6f accel_y=%.6f accel_z=%.6f"
                    % (
                        skip,
                        imu.orientation.x,
                        imu.orientation.y,
                        imu.orientation.z,
                        imu.orientation.w,
                        roll,
                        pitch,
                        yaw,
                        imu.angular_velocity.x,
                        imu.angular_velocity.y,
                        imu.angular_velocity.z,
                        imu.linear_acceleration.x,
                        imu.linear_acceleration.y,
                        imu.linear_acceleration.z,
                    )
                )
            )

    def _parse_payload(self, payload):
        start = payload.find(HEADER)
        if start < 0:
            raise ValueError("missing #IMUData$ header")

        size_offset = start + len(HEADER)
        if size_offset + 4 > len(payload):
            raise ValueError("missing data length")
        data_length = struct.unpack_from("<I", payload, size_offset)[0]
        body_start = size_offset + 4 + 12
        if body_start + data_length + 2 > len(payload):
            raise ValueError("packet shorter than declared data length")

        body = payload[body_start : body_start + data_length]
        values, skip = self._read_best_imu_values(body)
        qx, qy, qz, qw, avx, avy, avz, lax, lay, laz = values

        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.x = avx
        msg.angular_velocity.y = avy
        msg.angular_velocity.z = avz
        msg.linear_acceleration.x = lax
        msg.linear_acceleration.y = lay
        msg.linear_acceleration.z = laz
        return msg, skip

    def _read_best_imu_values(self, body):
        candidates = []
        for skip in (0, 8):
            if len(body) < skip + 80:
                continue
            values = struct.unpack_from("<dddddddddd", body, skip)
            q_norm = math.sqrt(sum(v * v for v in values[:4]))
            finite = all(math.isfinite(v) for v in values)
            if finite:
                score = abs(q_norm - 1.0)
                candidates.append((score, values, skip))

        if not candidates:
            raise ValueError("not enough IMU body data")

        candidates.sort(key=lambda item: item[0])
        return candidates[0][1], candidates[0][2]


if __name__ == "__main__":
    rospy.init_node("udp_imu_to_topic")
    UdpImuToTopic().run()
