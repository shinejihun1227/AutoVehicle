#!/usr/bin/env python3
import math
import socket

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String


EARTH_RADIUS_M = 6378137.0
KNOT_TO_MPS = 0.514444


class GpsLocalizer:
    def __init__(self):
        self.bind_ip = rospy.get_param("/gps/bind_ip", "0.0.0.0")
        self.port = int(rospy.get_param("/gps/port", 3001))
        self.frame_id = rospy.get_param("/gps/frame_id", "map")
        self.auto_origin = bool(rospy.get_param("/gps/auto_origin", True))
        self.origin_lat = self._optional_origin("/gps/origin_lat")
        self.origin_lon = self._optional_origin("/gps/origin_lon")
        self.origin_alt = float(rospy.get_param("/gps/origin_alt", 0.0))
        self.min_yaw_distance = float(rospy.get_param("/gps/min_yaw_distance_m", 0.4))
        self.min_yaw_speed = float(rospy.get_param("/gps/min_yaw_speed_mps", 0.4))
        self.speed_alpha = self._clamp(float(rospy.get_param("/gps/speed_filter_alpha", 0.25)), 0.0, 1.0)

        self.pose_pub = rospy.Publisher(rospy.get_param("/gps/pose_topic", "/localization/ego_pose"), PoseStamped, queue_size=1)
        self.twist_pub = rospy.Publisher(rospy.get_param("/gps/twist_topic", "/localization/ego_twist"), TwistStamped, queue_size=1)
        self.debug_pub = rospy.Publisher(rospy.get_param("/gps/debug_topic", "/localization/gps_debug"), String, queue_size=10)

        self.last_x = None
        self.last_y = None
        self.last_time = None
        self.yaw = 0.0
        self.speed = 0.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_ip, self.port))
        self.sock.settimeout(0.5)
        rospy.loginfo("GPS localizer listening on %s:%d", self.bind_ip, self.port)

    def run(self):
        while not rospy.is_shutdown():
            try:
                payload, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                rospy.logwarn_throttle(2.0, "waiting for GPS UDP packets on port %d", self.port)
                continue

            sentences = self._extract_nmea(payload)
            if "GPRMC" not in sentences:
                self.debug_pub.publish(String("no_gprmc from=%s head=%s" % (addr[0], payload[:16].hex(" "))))
                continue

            try:
                gps = self._parse_gprmc(sentences["GPRMC"])
                if "GPGGA" in sentences:
                    gps.update(self._parse_gpgga(sentences["GPGGA"]))
            except ValueError as exc:
                rospy.logwarn_throttle(1.0, "bad gps packet: %s", exc)
                continue

            if self.origin_lat is None or self.origin_lon is None:
                if not self.auto_origin:
                    rospy.logerr_throttle(2.0, "GPS origin is not configured")
                    continue
                self.origin_lat = gps["lat"]
                self.origin_lon = gps["lon"]
                self.origin_alt = gps["alt"]
                rospy.loginfo("GPS origin set lat=%.8f lon=%.8f alt=%.3f", self.origin_lat, self.origin_lon, self.origin_alt)

            now = rospy.Time.now()
            x, y, z = self._latlonalt_to_enu(gps["lat"], gps["lon"], gps["alt"])
            self._update_motion(x, y, now, gps["speed_mps"], gps["course_deg"])
            self._publish(x, y, z, now)
            self.debug_pub.publish(String("x=%.3f y=%.3f z=%.3f speed=%.3f yaw=%.3f lat=%.8f lon=%.8f" % (x, y, z, self.speed, self.yaw, gps["lat"], gps["lon"])))

    @staticmethod
    def _optional_origin(param_name):
        value = float(rospy.get_param(param_name, 0.0))
        return value if abs(value) > 1.0e-12 else None

    def _extract_nmea(self, payload):
        text = payload.decode("ascii", errors="ignore")
        out = {}
        for key in ("GPRMC", "GPGGA"):
            start = text.find("$" + key)
            if start < 0:
                continue
            end = text.find("\n", start)
            if end < 0:
                end = text.find("\r", start)
            if end < 0:
                end = len(text)
            out[key] = text[start:end].strip().strip("\x00")
        return out

    def _parse_gprmc(self, sentence):
        main = sentence.split("*", 1)[0]
        fields = main.split(",")
        if len(fields) < 9:
            raise ValueError("GPRMC has too few fields")
        if fields[2] != "A":
            raise ValueError("GPS status is not active")
        return {
            "lat": self._nmea_coord_to_deg(fields[3], fields[4]),
            "lon": self._nmea_coord_to_deg(fields[5], fields[6]),
            "alt": 0.0,
            "speed_mps": float(fields[7] or 0.0) * KNOT_TO_MPS,
            "course_deg": float(fields[8] or 0.0),
        }

    def _parse_gpgga(self, sentence):
        main = sentence.split("*", 1)[0]
        fields = main.split(",")
        if len(fields) < 10:
            raise ValueError("GPGGA has too few fields")
        return {
            "lat": self._nmea_coord_to_deg(fields[2], fields[3]),
            "lon": self._nmea_coord_to_deg(fields[4], fields[5]),
            "alt": float(fields[9] or 0.0),
        }

    @staticmethod
    def _nmea_coord_to_deg(value, hemisphere):
        if not value:
            raise ValueError("empty coordinate")
        raw = float(value)
        degrees = int(raw / 100.0)
        minutes = raw - degrees * 100.0
        deg = degrees + minutes / 60.0
        if hemisphere in ("S", "W"):
            deg *= -1.0
        return deg

    def _latlonalt_to_enu(self, lat, lon, alt):
        lat0 = math.radians(self.origin_lat)
        x = EARTH_RADIUS_M * math.radians(lon - self.origin_lon) * math.cos(lat0)
        y = EARTH_RADIUS_M * math.radians(lat - self.origin_lat)
        z = alt - self.origin_alt
        return x, y, z

    def _update_motion(self, x, y, now, gprmc_speed, course_deg):
        if self.last_time is None:
            self.last_x = x
            self.last_y = y
            self.last_time = now
            return

        dt = max((now - self.last_time).to_sec(), 1.0e-3)
        dx = x - self.last_x
        dy = y - self.last_y
        distance = math.hypot(dx, dy)
        measured_speed = distance / dt

        if distance >= self.min_yaw_distance and measured_speed >= self.min_yaw_speed:
            self.yaw = math.atan2(dy, dx)
            self.last_x = x
            self.last_y = y
            self.last_time = now
        elif gprmc_speed >= self.min_yaw_speed:
            self.yaw = math.radians(90.0 - course_deg)

        source_speed = measured_speed if distance >= self.min_yaw_distance else gprmc_speed
        self.speed = self.speed_alpha * source_speed + (1.0 - self.speed_alpha) * self.speed

    def _publish(self, x, y, z, now):
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        twist = TwistStamped()
        twist.header = pose.header
        twist.twist.linear.x = self.speed * math.cos(self.yaw)
        twist.twist.linear.y = self.speed * math.sin(self.yaw)

        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))


if __name__ == "__main__":
    rospy.init_node("gps_localizer")
    GpsLocalizer().run()
