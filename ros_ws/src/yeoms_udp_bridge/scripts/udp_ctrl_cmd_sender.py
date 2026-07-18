#!/usr/bin/env python3
import socket
import struct

import rospy
from geometry_msgs.msg import Twist


class UdpCtrlCmdSender:
    def __init__(self):
        self.target_ip = rospy.get_param("/udp_bridge/ctrl_sender/target_ip", "192.168.0.151")
        self.target_port = int(rospy.get_param("/udp_bridge/ctrl_sender/target_port", 9093))
        self.local_port = int(rospy.get_param("/udp_bridge/ctrl_sender/local_port", 9094))
        self.ctrl_mode = int(rospy.get_param("/udp_bridge/ctrl_sender/ctrl_mode", 2))
        self.gear = int(rospy.get_param("/udp_bridge/ctrl_sender/gear", 4))
        self.long_cmd_type = int(rospy.get_param("/udp_bridge/ctrl_sender/long_cmd_type", 1))
        self.topic = rospy.get_param("/udp_bridge/ctrl_sender/subscribe_topic", "/control/ctrl_cmd")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", self.local_port))
        rospy.Subscriber(self.topic, Twist, self._cmd_cb, queue_size=1)
        rospy.loginfo("UDP CtrlCmd sender target=%s:%s local_port=%s", self.target_ip, self.target_port, self.local_port)

    def _cmd_cb(self, msg):
        target_speed = max(0.0, float(msg.linear.x))
        steering = float(msg.angular.z)

        # Placeholder packet for connectivity tests. Replace with the exact MORAI Ego Ctrl Cmd
        # binary layout from the official UDP protocol/example before driving the vehicle.
        accel = 0.3 if target_speed > 0.1 else 0.0
        brake = 0.0 if target_speed > 0.1 else 0.5
        payload = struct.pack("<BBBffff", self.ctrl_mode, self.gear, self.long_cmd_type, accel, brake, steering, target_speed)
        self.sock.sendto(payload, (self.target_ip, self.target_port))


if __name__ == "__main__":
    rospy.init_node("udp_ctrl_cmd_sender")
    UdpCtrlCmdSender()
    rospy.spin()

