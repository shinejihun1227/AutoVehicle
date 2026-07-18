#!/usr/bin/env python3
import math
import socket

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String


EARTH_RADIUS_M = 6378137.0
KNOT_TO_MPS = 0.514444


class UdpGpsToLocalization:
    def __init__(self):
        self.bind_ip = rospy.get_param("/udp_bridge/bind_ip", "0.0.0.0")
        self.port = int(rospy.get_param("/udp_bridge/gps_adapter/gps_port", 3001))
        self.frame_id = rospy.get_param("/udp_bridge/gps_adapter/frame_id", "map")
        self.auto_origin = bool(rospy.get_param("/udp_bridge/gps_adapter/auto_origin", True))
        self.min_course_speed_mps = float(rospy.get_param("/udp_bridge/gps_adapter/min_course_speed_mps", 0.2))

        self.pose_pub = rospy.Publisher(
            rospy.get_param("/udp_bridge/gps_adapter/pose_topic", "/localization/ego_pose"),
            PoseStamped,
            queue_size=1,
        )
        self.twist_pub = rospy.Publisher(
            rospy.get_param("/udp_bridge/gps_adapter/twist_topic", "/localization/ego_twist"),
            TwistStamped,
            queue_size=1,
        )
        self.debug_pub = rospy.Publisher("/udp_bridge/gps_debug", String, queue_size=10)

        self.origin_lat = None
        self.origin_lon = None
        self.last_yaw = 0.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_ip, self.port))
        self.sock.settimeout(0.5)
        rospy.loginfo("GPS UDP localization listening on %s:%s", self.bind_ip, self.port)

    def run(self):
        while not rospy.is_shutdown():
            try:
                payload, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                rospy.logwarn_throttle(2.0, "waiting for GPS UDP packets on port %s", self.port)
                continue

            sentence = self._extract_gprmc(payload)
            if not sentence:
                self.debug_pub.publish(String("from=%s no GPRMC payload=%s" % (addr[0], payload[:32].hex(" "))))
                continue

            try:
                gps = self._parse_gprmc(sentence)
            except ValueError as exc:
                rospy.logwarn_throttle(1.0, "invalid GPRMC packet: %s sentence=%s", exc, sentence)
                continue

            if self.origin_lat is None:
                if not self.auto_origin:
                    rospy.logerr_throttle(2.0, "gps_adapter auto_origin=false but fixed origin is not configured yet")
                    continue
                self.origin_lat = gps["lat"]
                self.origin_lon = gps["lon"]
                rospy.loginfo("GPS origin set lat=%.8f lon=%.8f", self.origin_lat, self.origin_lon)

            x, y = self._latlon_to_local_xy(gps["lat"], gps["lon"])
            yaw = self._course_to_ros_yaw(gps["course_deg"], gps["speed_mps"])
            self._publish_state(x, y, yaw, gps["speed_mps"])
            self.debug_pub.publish(
                String(
                    "lat=%.8f lon=%.8f x=%.3f y=%.3f speed=%.3f course=%.3f yaw=%.3f"
                    % (gps["lat"], gps["lon"], x, y, gps["speed_mps"], gps["course_deg"], yaw)
                )
            )

    def _extract_gprmc(self, payload):
        text = payload.decode("ascii", errors="ignore")
        start = text.find("$GPRMC")
        if start < 0:
            return None
        end = text.find("\n", start)
        if end < 0:
            end = len(text)
        return text[start:end].strip().strip("\x00")

    def _parse_gprmc(self, sentence):
        main = sentence.split("*", 1)[0]
        fields = main.split(",")
        if len(fields) < 9:
            raise ValueError("not enough fields")
        if fields[2] != "A":
            raise ValueError("GPS status is not active")

        lat = self._nmea_coord_to_deg(fields[3], fields[4])
        lon = self._nmea_coord_to_deg(fields[5], fields[6])
        speed_knots = float(fields[7] or 0.0)
        course_deg = float(fields[8] or 0.0)
        return {
            "lat": lat,
            "lon": lon,
            "speed_mps": speed_knots * KNOT_TO_MPS,
            "course_deg": course_deg,
        }

    def _nmea_coord_to_deg(self, value, hemisphere):
        if not value:
            raise ValueError("empty coordinate")
        raw = float(value)
        degrees = int(raw / 100.0)
        minutes = raw - degrees * 100.0
        decimal = degrees + minutes / 60.0
        if hemisphere in ("S", "W"):
            decimal *= -1.0
        return decimal

    def _latlon_to_local_xy(self, lat, lon):
        lat0 = math.radians(self.origin_lat)
        d_lat = math.radians(lat - self.origin_lat)
        d_lon = math.radians(lon - self.origin_lon)
        x_east = EARTH_RADIUS_M * d_lon * math.cos(lat0)
        y_north = EARTH_RADIUS_M * d_lat
        return x_east, y_north

    def _course_to_ros_yaw(self, course_deg, speed_mps):
        if speed_mps < self.min_course_speed_mps:
            return self.last_yaw
        yaw = math.radians(90.0 - course_deg)
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        self.last_yaw = yaw
        return yaw

    def _publish_state(self, x, y, yaw, speed_mps):
        now = rospy.Time.now()

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        twist = TwistStamped()
        twist.header = pose.header
        twist.twist.linear.x = speed_mps

        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)


if __name__ == "__main__":
    rospy.init_node("udp_gps_to_localization")
    UdpGpsToLocalization().run()
