#!/usr/bin/env python3
import math
import socket

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String


EARTH_RADIUS_M = 6378137.0
KNOT_TO_MPS = 0.514444


class UdpGpsToLocalization:
    def __init__(self):
        self.bind_ip = rospy.get_param("/udp_bridge/bind_ip", "0.0.0.0")
        self.port = int(rospy.get_param("/udp_bridge/gps_adapter/gps_port", 3001))
        self.frame_id = rospy.get_param("/udp_bridge/gps_adapter/frame_id", "map")
        self.auto_origin = bool(rospy.get_param("/udp_bridge/gps_adapter/auto_origin", True))
        self.fixed_origin_lat = float(rospy.get_param("/udp_bridge/gps_adapter/origin_lat", 0.0))
        self.fixed_origin_lon = float(rospy.get_param("/udp_bridge/gps_adapter/origin_lon", 0.0))
        self.fixed_origin_alt = float(rospy.get_param("/udp_bridge/gps_adapter/origin_alt", 0.0))
        self.min_course_speed_mps = float(rospy.get_param("/udp_bridge/gps_adapter/min_course_speed_mps", 0.2))
        self.motion_source = rospy.get_param("/udp_bridge/gps_adapter/motion_source", "position")
        self.min_position_delta_m = float(rospy.get_param("/udp_bridge/gps_adapter/min_position_delta_m", 0.03))
        self.max_reasonable_speed_mps = float(rospy.get_param("/udp_bridge/gps_adapter/max_reasonable_speed_mps", 50.0))
        self.speed_filter_alpha = float(rospy.get_param("/udp_bridge/gps_adapter/speed_filter_alpha", 0.25))
        self.position_filter_alpha = float(rospy.get_param("/udp_bridge/gps_adapter/position_filter_alpha", 0.45))
        self.stationary_speed_decay = float(rospy.get_param("/udp_bridge/gps_adapter/stationary_speed_decay", 0.55))
        self.zero_speed_threshold_mps = float(rospy.get_param("/udp_bridge/gps_adapter/zero_speed_threshold_mps", 0.08))
        self.min_yaw_update_speed_mps = float(rospy.get_param("/udp_bridge/gps_adapter/min_yaw_update_speed_mps", 0.35))
        self.max_yaw_jump_rad = float(rospy.get_param("/udp_bridge/gps_adapter/max_yaw_jump_rad", 0.85))
        self.yaw_filter_alpha = float(rospy.get_param("/udp_bridge/gps_adapter/yaw_filter_alpha", 0.12))
        self.max_yaw_rate_radps = float(rospy.get_param("/udp_bridge/gps_adapter/max_yaw_rate_radps", 0.35))
        self.yaw_consistency_count = int(rospy.get_param("/udp_bridge/gps_adapter/yaw_consistency_count", 3))
        self.use_imu_yaw = bool(rospy.get_param("/udp_bridge/gps_adapter/use_imu_yaw", True))
        self.imu_topic = rospy.get_param("/udp_bridge/gps_adapter/imu_topic", "/udp_bridge/imu")
        self.imu_timeout_s = float(rospy.get_param("/udp_bridge/gps_adapter/imu_timeout_s", 0.5))
        self.imu_yaw_offset = float(rospy.get_param("/udp_bridge/gps_adapter/imu_yaw_offset_rad", 0.0))
        self.auto_align_imu_yaw = bool(rospy.get_param("/udp_bridge/gps_adapter/auto_align_imu_yaw", True))
        self.imu_yaw_filter_alpha = float(rospy.get_param("/udp_bridge/gps_adapter/imu_yaw_filter_alpha", 0.65))
        self.imu_yaw_alignment_alpha = float(rospy.get_param("/udp_bridge/gps_adapter/imu_yaw_alignment_alpha", 0.02))
        self.imu_align_min_speed_mps = float(rospy.get_param("/udp_bridge/gps_adapter/imu_align_min_speed_mps", 1.0))
        self.imu_yaw_sign = float(rospy.get_param("/udp_bridge/gps_adapter/imu_yaw_sign", 1.0))

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
        self.origin_alt = None
        self.last_yaw = 0.0
        self.last_x = None
        self.last_y = None
        self.filtered_x = None
        self.filtered_y = None
        self.last_time = None
        self.last_speed = 0.0
        self.last_motion_yaw = None
        self.consistent_motion_count = 0
        self.last_yaw_update_time = None
        self.yaw_initialized = False
        self.latest_imu_yaw = None
        self.latest_imu_time = None
        self.imu_yaw_offset_initialized = not self.auto_align_imu_yaw

        if self.use_imu_yaw:
            rospy.Subscriber(self.imu_topic, Imu, self._imu_cb, queue_size=1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_ip, self.port))
        self.sock.settimeout(0.5)
        if self.fixed_origin_lat != 0.0 or self.fixed_origin_lon != 0.0:
            self.origin_lat = self.fixed_origin_lat
            self.origin_lon = self.fixed_origin_lon
            self.origin_alt = self.fixed_origin_alt
            rospy.loginfo(
                "GPS fixed origin loaded lat=%.8f lon=%.8f alt=%.3f",
                self.origin_lat,
                self.origin_lon,
                self.origin_alt,
            )
        rospy.loginfo("GPS UDP localization listening on %s:%s", self.bind_ip, self.port)

    def run(self):
        while not rospy.is_shutdown():
            try:
                payload, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                rospy.logwarn_throttle(2.0, "waiting for GPS UDP packets on port %s", self.port)
                continue

            sentences = self._extract_nmea_sentences(payload)
            if "GPRMC" not in sentences:
                self.debug_pub.publish(String("from=%s no GPRMC payload=%s" % (addr[0], payload[:32].hex(" "))))
                continue

            try:
                gps = self._parse_gprmc(sentences["GPRMC"])
                if "GPGGA" in sentences:
                    gps.update(self._parse_gpgga(sentences["GPGGA"]))
            except ValueError as exc:
                rospy.logwarn_throttle(1.0, "invalid GPS packet: %s sentences=%s", exc, sentences)
                continue

            if self.origin_lat is None:
                if not self.auto_origin:
                    rospy.logerr_throttle(2.0, "gps_adapter auto_origin=false but fixed origin is not configured yet")
                    continue
                self.origin_lat = gps["lat"]
                self.origin_lon = gps["lon"]
                self.origin_alt = gps["alt"]
                rospy.loginfo(
                    "GPS origin set lat=%.8f lon=%.8f alt=%.3f",
                    self.origin_lat,
                    self.origin_lon,
                    self.origin_alt,
                )

            now = rospy.Time.now()
            raw_x, raw_y, z = self._latlonalt_to_local_xyz(gps["lat"], gps["lon"], gps["alt"])
            x, y = self._filter_position(raw_x, raw_y)
            speed, yaw, motion_source = self._estimate_motion(x, y, now, gps["speed_mps"], gps["course_deg"])
            self._publish_state(x, y, z, yaw, speed, now)
            self.debug_pub.publish(
                String(
                    "lat=%.8f lon=%.8f alt=%.3f raw_x=%.3f raw_y=%.3f z=%.3f x=%.3f y=%.3f speed=%.3f yaw=%.3f source=%s gprmc_speed=%.3f gprmc_course=%.3f yaw_initialized=%s imu_yaw=%s imu_offset=%.3f imu_aligned=%s"
                    % (
                        gps["lat"],
                        gps["lon"],
                        gps["alt"],
                        raw_x,
                        raw_y,
                        z,
                        x,
                        y,
                        speed,
                        yaw,
                        motion_source,
                        gps["speed_mps"],
                        gps["course_deg"],
                        self.yaw_initialized,
                        "%.3f" % self.latest_imu_yaw if self.latest_imu_yaw is not None else "none",
                        self.imu_yaw_offset,
                        self.imu_yaw_offset_initialized,
                    )
                )
            )

    def _extract_nmea_sentences(self, payload):
        text = payload.decode("ascii", errors="ignore")
        sentences = {}
        for sentence_type in ("GPRMC", "GPGGA"):
            start = text.find("$" + sentence_type)
            if start < 0:
                continue
            end = text.find("\n", start)
            if end < 0:
                end = text.find("\r", start)
            if end < 0:
                end = len(text)
            sentences[sentence_type] = text[start:end].strip().strip("\x00")
        return sentences

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
            "alt": 0.0,
            "speed_mps": speed_knots * KNOT_TO_MPS,
            "course_deg": course_deg,
        }

    def _parse_gpgga(self, sentence):
        main = sentence.split("*", 1)[0]
        fields = main.split(",")
        if len(fields) < 10:
            raise ValueError("not enough GPGGA fields")
        return {
            "lat": self._nmea_coord_to_deg(fields[2], fields[3]),
            "lon": self._nmea_coord_to_deg(fields[4], fields[5]),
            "alt": float(fields[9] or 0.0),
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

    def _latlonalt_to_local_xyz(self, lat, lon, alt):
        lat0 = math.radians(self.origin_lat)
        d_lat = math.radians(lat - self.origin_lat)
        d_lon = math.radians(lon - self.origin_lon)
        x_east = EARTH_RADIUS_M * d_lon * math.cos(lat0)
        y_north = EARTH_RADIUS_M * d_lat
        z_up = alt - self.origin_alt if self.origin_alt is not None else 0.0
        return x_east, y_north, z_up

    def _course_to_ros_yaw(self, course_deg, speed_mps):
        if speed_mps < self.min_course_speed_mps:
            return self.last_yaw
        yaw = math.radians(90.0 - course_deg)
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        self.last_yaw = yaw
        return yaw

    def _filter_position(self, raw_x, raw_y):
        if self.filtered_x is None:
            self.filtered_x = raw_x
            self.filtered_y = raw_y
            return self.filtered_x, self.filtered_y

        alpha = self._clamp(self.position_filter_alpha, 0.0, 1.0)
        self.filtered_x = alpha * raw_x + (1.0 - alpha) * self.filtered_x
        self.filtered_y = alpha * raw_y + (1.0 - alpha) * self.filtered_y
        return self.filtered_x, self.filtered_y

    def _imu_cb(self, msg):
        q = msg.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1.0e-6:
            return
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.latest_imu_yaw = self._normalize_angle(self.imu_yaw_sign * yaw)
        self.latest_imu_time = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()

    def _estimate_motion(self, x, y, now, gprmc_speed_mps, gprmc_course_deg):
        if self.motion_source == "gprmc":
            yaw = self._course_to_ros_yaw(gprmc_course_deg, gprmc_speed_mps)
            imu_source = self._apply_imu_yaw_if_available(now, gprmc_speed_mps)
            return gprmc_speed_mps, self.last_yaw, "gprmc" + (("+" + imu_source) if imu_source else "")

        if self.last_time is None:
            self.last_x = x
            self.last_y = y
            self.last_time = now
            return 0.0, self.last_yaw, "position:init"

        dt = (now - self.last_time).to_sec()
        dx = x - self.last_x
        dy = y - self.last_y
        distance = math.hypot(dx, dy)

        if dt <= 0.0 or distance < self.min_position_delta_m:
            return self._decay_speed(), self.last_yaw, "position:hold"

        measured_speed = distance / dt
        if not math.isfinite(measured_speed) or measured_speed > self.max_reasonable_speed_mps:
            rospy.logwarn_throttle(1.0, "GPS position speed rejected: %.3f m/s", measured_speed)
            self.last_x = x
            self.last_y = y
            self.last_time = now
            return self._decay_speed(), self.last_yaw, "position:rejected"

        measured_yaw = math.atan2(dy, dx)
        yaw_source = self._update_yaw_if_consistent(measured_yaw, measured_speed, now)
        alpha = self._clamp(self.speed_filter_alpha, 0.0, 1.0)
        self.last_speed = alpha * measured_speed + (1.0 - alpha) * self.last_speed
        imu_source = self._apply_imu_yaw_if_available(now, self.last_speed)
        self.last_x = x
        self.last_y = y
        self.last_time = now
        return self.last_speed, self.last_yaw, yaw_source + (("+" + imu_source) if imu_source else "")

    def _update_yaw_if_consistent(self, measured_yaw, measured_speed, now):
        if measured_speed < self.min_yaw_update_speed_mps:
            return "position:yaw_hold"

        if self.last_motion_yaw is None:
            self.last_motion_yaw = measured_yaw
            self.consistent_motion_count = 1
            return "position:yaw_seed"

        yaw_delta = abs(self._normalize_angle(measured_yaw - self.last_motion_yaw))
        if yaw_delta <= self.max_yaw_jump_rad:
            self.consistent_motion_count += 1
            self.last_motion_yaw = measured_yaw
        else:
            self.consistent_motion_count = 1
            self.last_motion_yaw = measured_yaw
            return "position:yaw_jump_hold"

        if self.consistent_motion_count >= max(1, self.yaw_consistency_count):
            return self._apply_filtered_yaw(measured_yaw, now)
        return "position:yaw_warmup"

    def _apply_filtered_yaw(self, measured_yaw, now):
        if not self.yaw_initialized:
            self.last_yaw = measured_yaw
            self.last_yaw_update_time = now
            self.yaw_initialized = True
            return "position:yaw_init"

        yaw_error = self._normalize_angle(measured_yaw - self.last_yaw)
        alpha = self._clamp(self.yaw_filter_alpha, 0.0, 1.0)
        filtered_delta = alpha * yaw_error

        if self.last_yaw_update_time is None:
            dt = 1.0
        else:
            dt = max((now - self.last_yaw_update_time).to_sec(), 1.0e-3)
        self.last_yaw_update_time = now

        if self.max_yaw_rate_radps > 0.0:
            max_delta = self.max_yaw_rate_radps * dt
            filtered_delta = self._clamp(filtered_delta, -max_delta, max_delta)

        self.last_yaw = self._normalize_angle(self.last_yaw + filtered_delta)
        return "position:yaw_filtered"

    def _apply_imu_yaw_if_available(self, now, speed_mps):
        if not self.use_imu_yaw or self.latest_imu_yaw is None or self.latest_imu_time is None:
            return None

        if (now - self.latest_imu_time).to_sec() > self.imu_timeout_s:
            return "imu:stale"

        if self.auto_align_imu_yaw:
            if self.yaw_initialized and speed_mps >= self.imu_align_min_speed_mps:
                target_offset = self._normalize_angle(self.last_yaw - self.latest_imu_yaw)
                if not self.imu_yaw_offset_initialized:
                    self.imu_yaw_offset = target_offset
                    self.imu_yaw_offset_initialized = True
                    return "imu:aligned"

                alpha = self._clamp(self.imu_yaw_alignment_alpha, 0.0, 1.0)
                offset_error = self._normalize_angle(target_offset - self.imu_yaw_offset)
                self.imu_yaw_offset = self._normalize_angle(self.imu_yaw_offset + alpha * offset_error)

        if not self.imu_yaw_offset_initialized:
            return "imu:wait_align"

        imu_map_yaw = self._normalize_angle(self.latest_imu_yaw + self.imu_yaw_offset)
        if not self.yaw_initialized:
            self.last_yaw = imu_map_yaw
            self.yaw_initialized = True
            self.last_yaw_update_time = now
            return "imu:yaw_init"

        yaw_error = self._normalize_angle(imu_map_yaw - self.last_yaw)
        alpha = self._clamp(self.imu_yaw_filter_alpha, 0.0, 1.0)
        filtered_delta = alpha * yaw_error

        if self.last_yaw_update_time is None:
            dt = 1.0
        else:
            dt = max((now - self.last_yaw_update_time).to_sec(), 1.0e-3)
        self.last_yaw_update_time = now

        if self.max_yaw_rate_radps > 0.0:
            max_delta = self.max_yaw_rate_radps * dt
            filtered_delta = self._clamp(filtered_delta, -max_delta, max_delta)

        self.last_yaw = self._normalize_angle(self.last_yaw + filtered_delta)
        return "imu:yaw"

    def _decay_speed(self):
        self.last_speed *= self._clamp(self.stationary_speed_decay, 0.0, 1.0)
        if self.last_speed < self.zero_speed_threshold_mps:
            self.last_speed = 0.0
        return self.last_speed

    def _publish_state(self, x, y, z, yaw, speed_mps, now):

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
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

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))


if __name__ == "__main__":
    rospy.init_node("udp_gps_to_localization")
    UdpGpsToLocalization().run()
