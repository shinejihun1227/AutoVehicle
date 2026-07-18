#!/usr/bin/env python3
import select
import socket
import time

import rospy
from std_msgs.msg import String


class UdpPacketMonitor:
    def __init__(self):
        self.bind_ip = rospy.get_param("/udp_bridge/bind_ip", "0.0.0.0")
        self.hex_bytes = int(rospy.get_param("/udp_bridge/monitor/print_payload_hex_bytes", 32))
        self.print_rate_hz = float(rospy.get_param("/udp_bridge/monitor/print_rate_hz", 1.0))
        self.ports = rospy.get_param("/udp_bridge/ports", {})
        self.pub = rospy.Publisher("/udp_bridge/packet_info", String, queue_size=20)
        self.sockets = {}
        self.last_print = {}

        for name, port in sorted(self.ports.items()):
            if not self._should_monitor(name):
                continue
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.bind_ip, int(port)))
            sock.setblocking(False)
            self.sockets[sock] = (name, int(port))
            self.last_print[name] = 0.0
            rospy.loginfo("UDP monitor listening: %s on %s:%s", name, self.bind_ip, port)

    @staticmethod
    def _should_monitor(name):
        return name not in ("ego_ctrl_cmd_morai", "ego_ctrl_cmd_local")

    def run(self):
        while not rospy.is_shutdown():
            readable, _, _ = select.select(list(self.sockets.keys()), [], [], 0.2)
            now = time.time()
            for sock in readable:
                name, port = self.sockets[sock]
                payload, addr = sock.recvfrom(65535)
                preview = payload[: self.hex_bytes].hex(" ")
                msg = f"{name} port={port} from={addr[0]}:{addr[1]} len={len(payload)} hex={preview}"
                self.pub.publish(String(msg))
                if now - self.last_print[name] >= 1.0 / max(self.print_rate_hz, 0.1):
                    rospy.loginfo(msg)
                    self.last_print[name] = now


if __name__ == "__main__":
    rospy.init_node("udp_packet_monitor")
    UdpPacketMonitor().run()

