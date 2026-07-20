#!/usr/bin/env python3
import math
import socket
import struct

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


VLP16_VERTICAL_ANGLES_DEG = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
POINT_FIELDS = [
    PointField("x", 0, PointField.FLOAT32, 1),
    PointField("y", 4, PointField.FLOAT32, 1),
    PointField("z", 8, PointField.FLOAT32, 1),
    PointField("intensity", 12, PointField.FLOAT32, 1),
    PointField("ring", 16, PointField.FLOAT32, 1),
]


class Vlp16UdpReceiver:
    def __init__(self):
        self.bind_ip = self._param("bind_ip", "0.0.0.0")
        self.lidar_port = int(self._param("lidar_port", 2001))
        self.pointcloud_topic = self._param("pointcloud_topic", "/sensors/lidar/points")
        self.packet_info_topic = self._param("packet_info_topic", "/perception/lidar/packet_info")
        self.frame_id = self._param("frame_id", "velodyne")
        self.socket_timeout = float(self._param("socket_timeout_sec", 1.0))
        self.packets_per_cloud = max(1, int(self._param("publish_packets_per_cloud", 76)))
        self.min_distance = float(self._param("min_distance_m", 0.2))
        self.max_distance = float(self._param("max_distance_m", 120.0))
        self.max_points_per_cloud = int(self._param("max_points_per_cloud", 100000))
        self.azimuth_offset = math.radians(float(self._param("azimuth_offset_deg", 0.0)))
        self.invert_y = bool(self._param("invert_y", False))
        self.invert_z = bool(self._param("invert_z", False))

        self.vertical_angles = [math.radians(deg) for deg in VLP16_VERTICAL_ANGLES_DEG]
        self.cloud_points = []
        self.packet_count = 0
        self.cloud_count = 0

        self.cloud_pub = rospy.Publisher(self.pointcloud_topic, PointCloud2, queue_size=1)
        self.info_pub = rospy.Publisher(self.packet_info_topic, String, queue_size=10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_ip, self.lidar_port))
        self.sock.settimeout(self.socket_timeout)
        rospy.loginfo("VLP16 UDP receiver listening on %s:%d -> %s", self.bind_ip, self.lidar_port, self.pointcloud_topic)

    def run(self):
        while not rospy.is_shutdown():
            try:
                payload, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                rospy.logwarn_throttle(2.0, "waiting for VLP16 UDP packets on port %d", self.lidar_port)
                continue

            self.packet_count += 1
            points = self._parse_packet(payload)
            if points:
                self.cloud_points.extend(points)

            if self.packet_count % self.packets_per_cloud == 0:
                self._publish_cloud(addr)

    def _parse_packet(self, payload):
        if len(payload) < 1200:
            rospy.logwarn_throttle(2.0, "short VLP16 packet len=%d", len(payload))
            return []

        points = []
        for block_idx in range(12):
            block_offset = block_idx * 100
            flag = struct.unpack_from("<H", payload, block_offset)[0]
            if flag != 0xEEFF:
                continue

            azimuth_raw = struct.unpack_from("<H", payload, block_offset + 2)[0]
            azimuth = math.radians(azimuth_raw / 100.0) + self.azimuth_offset

            for channel in range(32):
                data_offset = block_offset + 4 + channel * 3
                distance_raw = struct.unpack_from("<H", payload, data_offset)[0]
                intensity = float(payload[data_offset + 2])
                distance_m = distance_raw * 0.002
                if distance_m < self.min_distance or distance_m > self.max_distance:
                    continue

                ring = channel % 16
                vertical = self.vertical_angles[ring]
                x = distance_m * math.cos(vertical) * math.cos(azimuth)
                y = distance_m * math.cos(vertical) * math.sin(azimuth)
                z = distance_m * math.sin(vertical)
                if self.invert_y:
                    y = -y
                if self.invert_z:
                    z = -z
                points.append((x, y, z, intensity, float(ring)))

        return points

    def _publish_cloud(self, addr):
        if not self.cloud_points:
            return

        if len(self.cloud_points) > self.max_points_per_cloud:
            self.cloud_points = self.cloud_points[-self.max_points_per_cloud:]

        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        cloud = pc2.create_cloud(header, POINT_FIELDS, self.cloud_points)
        self.cloud_pub.publish(cloud)

        self.cloud_count += 1
        info = "cloud=%d packets=%d points=%d from=%s:%d topic=%s" % (
            self.cloud_count,
            self.packet_count,
            len(self.cloud_points),
            addr[0],
            addr[1],
            self.pointcloud_topic,
        )
        self.info_pub.publish(String(info))
        rospy.loginfo_throttle(1.0, info)
        self.cloud_points = []

    @staticmethod
    def _param(name, default):
        private_name = "~" + name
        if rospy.has_param(private_name):
            return rospy.get_param(private_name)
        return rospy.get_param("/vlp16_udp_receiver/" + name, default)


if __name__ == "__main__":
    rospy.init_node("vlp16_udp_receiver")
    Vlp16UdpReceiver().run()
