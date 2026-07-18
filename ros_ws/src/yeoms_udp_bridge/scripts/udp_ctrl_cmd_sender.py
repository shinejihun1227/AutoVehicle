#!/usr/bin/env python3
import socket
import struct

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


HEADER = b"#MoraiCtrlCmd$"
TAIL = b"\r\n"
BODY_LENGTH = 23


class UdpCtrlCmdSender:
    def __init__(self):
        self.target_ip = rospy.get_param("/udp_bridge/ctrl_sender/target_ip", "192.168.0.151")
        self.target_port = int(rospy.get_param("/udp_bridge/ctrl_sender/target_port", 9093))
        self.local_port = int(rospy.get_param("/udp_bridge/ctrl_sender/local_port", 9094))
        self.ctrl_mode = int(rospy.get_param("/udp_bridge/ctrl_sender/ctrl_mode", 2))
        self.gear = int(rospy.get_param("/udp_bridge/ctrl_sender/gear", 4))
        self.long_cmd_type = int(rospy.get_param("/udp_bridge/ctrl_sender/long_cmd_type", 2))
        self.max_speed_mps = float(rospy.get_param("/udp_bridge/ctrl_sender/max_speed_mps", 1.0))
        self.max_steer_rad = float(rospy.get_param("/udp_bridge/ctrl_sender/max_steer_rad", 0.6981317008))
        self.invert_steering = bool(rospy.get_param("/udp_bridge/ctrl_sender/invert_steering", False))
        self.send_rate_limit_hz = float(rospy.get_param("/udp_bridge/ctrl_sender/send_rate_limit_hz", 20.0))
        self.topic = rospy.get_param("/udp_bridge/ctrl_sender/subscribe_topic", "/control/ctrl_cmd")
        self.last_send_time = rospy.Time(0)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", self.local_port))
        rospy.Subscriber(self.topic, Twist, self._cmd_cb, queue_size=1)
        self.debug_pub = rospy.Publisher("/udp_bridge/ctrl_cmd_debug", String, queue_size=10)
        rospy.loginfo(
            "UDP CtrlCmd sender target=%s:%s local_port=%s long_cmd_type=%s max_speed=%.2f",
            self.target_ip,
            self.target_port,
            self.local_port,
            self.long_cmd_type,
            self.max_speed_mps,
        )

    def _cmd_cb(self, msg):
        now = rospy.Time.now()
        if self.send_rate_limit_hz > 0.0 and (now - self.last_send_time).to_sec() < 1.0 / self.send_rate_limit_hz:
            return
        self.last_send_time = now

        target_speed_mps = self._clamp(float(msg.linear.x), 0.0, self.max_speed_mps)
        steering_rad = self._clamp(float(msg.angular.z), -self.max_steer_rad, self.max_steer_rad)
        steering_cmd = steering_rad / self.max_steer_rad if self.max_steer_rad > 0.0 else 0.0
        if self.invert_steering:
            steering_cmd *= -1.0

        velocity_kmh = target_speed_mps * 3.6
        acceleration = 0.0
        accel_cmd = 0.0
        brake_cmd = 0.0

        body = struct.pack(
            "<BBBfffff",
            self.ctrl_mode,
            self.gear,
            self.long_cmd_type,
            velocity_kmh,
            acceleration,
            accel_cmd,
            brake_cmd,
            steering_cmd,
        )
        payload = HEADER + struct.pack("<I", BODY_LENGTH) + (b"\x00" * 12) + body + TAIL
        self.sock.sendto(payload, (self.target_ip, self.target_port))
        self.debug_pub.publish(
            String(
                "bytes=%d mode=%d gear=%d long_cmd_type=%d velocity_kmh=%.3f steering_cmd=%.3f steering_rad=%.3f"
                % (len(payload), self.ctrl_mode, self.gear, self.long_cmd_type, velocity_kmh, steering_cmd, steering_rad)
            )
        )

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))


if __name__ == "__main__":
    rospy.init_node("udp_ctrl_cmd_sender")
    UdpCtrlCmdSender()
    rospy.spin()
