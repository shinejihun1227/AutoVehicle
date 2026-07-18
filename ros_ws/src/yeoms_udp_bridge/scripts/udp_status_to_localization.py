#!/usr/bin/env python3
import math
import socket
import struct

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String


class UdpStatusToLocalization:
    def __init__(self):
        self.bind_ip = rospy.get_param("/udp_bridge/bind_ip", "0.0.0.0")
        self.port = int(rospy.get_param("/udp_bridge/status_adapter/vehicle_status_port", 909))
        self.parser_mode = rospy.get_param("/udp_bridge/status_adapter/parser_mode", "raw")
        self.frame_id = rospy.get_param("/udp_bridge/status_adapter/frame_id", "map")
        self.pose_pub = rospy.Publisher(
            rospy.get_param("/udp_bridge/status_adapter/pose_topic", "/localization/ego_pose"),
            PoseStamped,
            queue_size=1,
        )
        self.twist_pub = rospy.Publisher(
            rospy.get_param("/udp_bridge/status_adapter/twist_topic", "/localization/ego_twist"),
            TwistStamped,
            queue_size=1,
        )
        self.raw_pub = rospy.Publisher("/udp_bridge/vehicle_status_raw", String, queue_size=10)
        self.debug_pub = rospy.Publisher("/udp_bridge/status_debug", String, queue_size=10)
        self.offsets = self._load_offsets()
        self.origin_x = None
        self.origin_y = None
        self.last_x = None
        self.last_y = None
        self.last_time = None
        self.last_speed = 0.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_ip, self.port))
        self.sock.settimeout(0.5)
        rospy.loginfo("Vehicle status UDP listening on %s:%s parser_mode=%s", self.bind_ip, self.port, self.parser_mode)

    def _load_offsets(self):
        prefix = "/udp_bridge/status_adapter"
        return {
            "x": int(rospy.get_param(f"{prefix}/x_offset", -1)),
            "y": int(rospy.get_param(f"{prefix}/y_offset", -1)),
            "z": int(rospy.get_param(f"{prefix}/z_offset", -1)),
            "yaw": int(rospy.get_param(f"{prefix}/yaw_offset", -1)),
            "vx": int(rospy.get_param(f"{prefix}/velocity_x_offset", -1)),
            "vy": int(rospy.get_param(f"{prefix}/velocity_y_offset", -1)),
            "vz": int(rospy.get_param(f"{prefix}/velocity_z_offset", -1)),
            "yaw_unit": rospy.get_param(f"{prefix}/yaw_unit", "deg"),
            "little_endian": bool(rospy.get_param(f"{prefix}/little_endian", True)),
            "numeric_type": rospy.get_param(f"{prefix}/numeric_type", "float64"),
            "auto_origin": bool(rospy.get_param(f"{prefix}/auto_origin", False)),
            "max_reasonable_speed_mps": float(rospy.get_param(f"{prefix}/max_reasonable_speed_mps", 50.0)),
        }

    def run(self):
        while not rospy.is_shutdown():
            try:
                payload, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                rospy.logwarn_throttle(2.0, "waiting for vehicle status UDP packets on port %s", self.port)
                continue

            self.raw_pub.publish(String(f"from={addr[0]}:{addr[1]} len={len(payload)} hex={payload[:64].hex(' ')}"))

            if self.parser_mode == "raw":
                rospy.logwarn_throttle(
                    2.0,
                    "vehicle status packets are arriving, but parser_mode=raw. Set byte offsets after packet layout is confirmed.",
                )
                continue

            try:
                if self.parser_mode == "photo_guess":
                    x, y, z, yaw, vx, vy, vz = self._parse_photo_guess(payload)
                else:
                    x = self._read_number(payload, self.offsets["x"])
                    y = self._read_number(payload, self.offsets["y"])
                    z = self._read_number(payload, self.offsets["z"], default=0.0)
                    yaw = self._read_number(payload, self.offsets["yaw"])
                    vx = self._read_number(payload, self.offsets["vx"], default=0.0)
                    vy = self._read_number(payload, self.offsets["vy"], default=0.0)
                    vz = self._read_number(payload, self.offsets["vz"], default=0.0)
            except ValueError as exc:
                rospy.logerr_throttle(1.0, "failed to parse vehicle status packet: %s", exc)
                continue

            if self.offsets["yaw_unit"] == "deg":
                yaw = math.radians(yaw)
            yaw = self._normalize_angle(yaw)
            self._publish_state(x, y, z, yaw, vx, vy, vz)

    def _parse_photo_guess(self, payload):
        # Experimental layout inferred from the user's 181-byte #MoraiInfo$ screenshots.
        # Keep this as a disposable bridge until the official UDP packet layout is confirmed.
        raw_yaw_deg = self._read_number(payload, self.offsets["yaw"])
        raw_x = self._read_number(payload, self.offsets["x"])
        raw_y = self._read_number(payload, self.offsets["y"])
        z = self._read_number(payload, self.offsets["z"], default=0.0)

        if self.offsets["auto_origin"]:
            if self.origin_x is None:
                self.origin_x = raw_x
                self.origin_y = raw_y
                rospy.loginfo("photo_guess origin set raw_x=%.3f raw_y=%.3f", self.origin_x, self.origin_y)
            x = raw_x - self.origin_x
            y = raw_y - self.origin_y
        else:
            x = raw_x
            y = raw_y

        speed = self._estimate_speed_from_position(x, y)
        candidates = self._format_float_candidates(payload)
        self.debug_pub.publish(
            String(
                "photo_guess raw_x=%.3f raw_y=%.3f rel_x=%.3f rel_y=%.3f raw_yaw=%.3f speed=%.3f candidates=%s"
                % (raw_x, raw_y, x, y, raw_yaw_deg, speed, candidates)
            )
        )
        return x, y, z, raw_yaw_deg, speed, 0.0, 0.0

    def _estimate_speed_from_position(self, x, y):
        now = rospy.Time.now()
        if self.last_time is None:
            self.last_x = x
            self.last_y = y
            self.last_time = now
            return 0.0

        dt = (now - self.last_time).to_sec()
        if dt <= 0.0:
            return self.last_speed

        distance = math.hypot(x - self.last_x, y - self.last_y)
        speed = distance / dt
        if not math.isfinite(speed) or speed > self.offsets["max_reasonable_speed_mps"]:
            rospy.logwarn_throttle(1.0, "photo_guess speed rejected: %.3f m/s", speed)
            speed = self.last_speed

        self.last_x = x
        self.last_y = y
        self.last_time = now
        self.last_speed = speed
        return speed

    def _format_float_candidates(self, payload):
        interesting_offsets = [53, 57, 61, 65, 73, 86, 90, 94, 98, 110, 114, 118]
        values = []
        for offset in interesting_offsets:
            try:
                values.append("%d:%.3f" % (offset, self._read_number(payload, offset, default=0.0)))
            except ValueError:
                pass
        return ",".join(values)

    def _read_number(self, payload, offset, default=None):
        if offset < 0:
            if default is not None:
                return default
            raise ValueError("required offset is not configured")

        fmt_type = "d" if self.offsets["numeric_type"] == "float64" else "f"
        size = struct.calcsize(fmt_type)
        if offset + size > len(payload):
            raise ValueError(f"offset {offset} out of packet range len={len(payload)}")
        endian = "<" if self.offsets["little_endian"] else ">"
        return struct.unpack_from(endian + fmt_type, payload, offset)[0]

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _publish_state(self, x, y, z, yaw, vx, vy, vz):
        now = rospy.Time.now()
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
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = vz

        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)


if __name__ == "__main__":
    rospy.init_node("udp_status_to_localization")
    UdpStatusToLocalization().run()
