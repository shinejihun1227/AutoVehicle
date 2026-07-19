#!/usr/bin/env python3
import socket
import struct

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


HEADER = b"#MoraiCtrlCmd$"
TAIL = b"\r\n"
BODY_LENGTH = 23


class CtrlCmdSender:
    def __init__(self):
        self.topic = rospy.get_param("/ctrl_cmd/subscribe_topic", "/control/ctrl_cmd")
        self.target_ip = rospy.get_param("/ctrl_cmd/target_ip", "192.168.0.151")
        self.target_port = int(rospy.get_param("/ctrl_cmd/target_port", 9093))
        self.local_port = int(rospy.get_param("/ctrl_cmd/local_port", 9094))
        self.ctrl_mode = int(rospy.get_param("/ctrl_cmd/ctrl_mode", 2))
        self.gear = int(rospy.get_param("/ctrl_cmd/gear", 4))
        self.long_cmd_type = int(rospy.get_param("/ctrl_cmd/long_cmd_type", 2))
        self.max_speed = float(rospy.get_param("/ctrl_cmd/max_speed_mps", 1.2))
        self.max_steer = float(rospy.get_param("/ctrl_cmd/max_steer_rad", 0.35))
        self.invert_steering = bool(rospy.get_param("/ctrl_cmd/invert_steering", False))
        self.steering_packet_mode = rospy.get_param("/ctrl_cmd/steering_packet_mode", "radian")
        self.send_rate_limit_hz = float(rospy.get_param("/ctrl_cmd/send_rate_limit_hz", 20.0))
        self.last_send_time = rospy.Time(0)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", self.local_port))

        self.debug_pub = rospy.Publisher(rospy.get_param("/ctrl_cmd/debug_topic", "/control/udp_cmd_debug"), String, queue_size=10)
        rospy.Subscriber(self.topic, Twist, self._cmd_cb, queue_size=1)
        rospy.loginfo("CtrlCmd sender target=%s:%d local_port=%d steering_mode=%s", self.target_ip, self.target_port, self.local_port, self.steering_packet_mode)

    def _cmd_cb(self, msg):
        now = rospy.Time.now()
        if self.send_rate_limit_hz > 0.0 and (now - self.last_send_time).to_sec() < 1.0 / self.send_rate_limit_hz:
            return
        self.last_send_time = now

        speed_mps = self._clamp(float(msg.linear.x), 0.0, self.max_speed)
        steering_rad = self._clamp(float(msg.angular.z), -self.max_steer, self.max_steer)
        if self.invert_steering:
            steering_rad *= -1.0

        if self.steering_packet_mode == "normalized":
            steering_packet = steering_rad / self.max_steer if self.max_steer > 0.0 else 0.0
        else:
            steering_packet = steering_rad

        body = struct.pack(
            "<BBBfffff",
            self.ctrl_mode,
            self.gear,
            self.long_cmd_type,
            speed_mps * 3.6,
            0.0,
            0.0,
            0.0,
            steering_packet,
        )
        payload = HEADER + struct.pack("<I", BODY_LENGTH) + (b"\x00" * 12) + body + TAIL
        self.sock.sendto(payload, (self.target_ip, self.target_port))
        self.debug_pub.publish(String("speed_mps=%.3f steering_rad=%.3f steering_packet=%.3f mode=%s" % (speed_mps, steering_rad, steering_packet, self.steering_packet_mode)))

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))


if __name__ == "__main__":
    rospy.init_node("ctrl_cmd_sender")
    CtrlCmdSender()
    rospy.spin()
