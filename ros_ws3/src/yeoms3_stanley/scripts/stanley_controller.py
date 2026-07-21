#!/usr/bin/env python3
import csv
import math
import os
from dataclasses import dataclass

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Int32


@dataclass
class Waypoint:
    x: float
    y: float
    target_speed: float


class StanleyController:
    def __init__(self):
        ns = "/stanley"
        self.pose_topic = rospy.get_param(ns + "/pose_topic", "/localization/ego_pose")
        self.twist_topic = rospy.get_param(ns + "/twist_topic", "/localization/ego_twist")
        self.command_topic = rospy.get_param(ns + "/command_topic", "/control/ctrl_cmd")
        self.frame_id = rospy.get_param(ns + "/frame_id", "map")
        self.rate_hz = float(rospy.get_param(ns + "/control_rate_hz", 20.0))
        self.wheelbase = float(rospy.get_param(ns + "/wheelbase_m", 3.0))

        self.target_speed = float(rospy.get_param(ns + "/target_speed_mps", 1.0))
        self.max_speed = float(rospy.get_param(ns + "/max_speed_mps", 1.0))
        self.use_csv_target_speed = bool(rospy.get_param(ns + "/use_csv_target_speed", False))
        self.curve_slowdown_enabled = bool(rospy.get_param(ns + "/curve_slowdown_enabled", True))
        self.curve_heading_slowdown = float(rospy.get_param(ns + "/curve_heading_slowdown_rad", 0.45))
        self.min_curve_speed = float(rospy.get_param(ns + "/min_curve_speed_mps", 2.5))

        self.k = float(rospy.get_param(ns + "/stanley_gain", 0.35))
        self.softening_gain = float(rospy.get_param(ns + "/softening_gain", 2.0))
        self.heading_error_gain = float(rospy.get_param(ns + "/heading_error_gain", 0.8))
        self.crosstrack_error_gain = float(rospy.get_param(ns + "/crosstrack_error_gain", 0.7))
        self.cross_track_deadband = float(rospy.get_param(ns + "/cross_track_deadband_m", 0.05))

        self.max_steer = float(rospy.get_param(ns + "/max_steer_rad", 0.25))
        self.steering_alpha = self._clamp(float(rospy.get_param(ns + "/steering_filter_alpha", 0.18)), 0.0, 1.0)
        self.max_steer_rate = float(rospy.get_param(ns + "/max_steer_rate_radps", 0.35))

        self.search_back_window = int(rospy.get_param(ns + "/target_search_back_window", 8))
        self.search_forward_window = int(rospy.get_param(ns + "/target_search_forward_window", 120))
        self.path_yaw_preview = float(rospy.get_param(ns + "/path_yaw_preview_m", 4.0))
        self.finish_radius = float(rospy.get_param(ns + "/finish_radius_m", 2.0))
        self.stop_at_final = bool(rospy.get_param(ns + "/stop_at_final_waypoint", True))

        waypoint_file = os.path.expanduser(rospy.get_param(ns + "/waypoint_file", ""))
        self.waypoints = self._load_waypoints(waypoint_file)

        self.x = None
        self.y = None
        self.yaw = None
        self.speed = 0.0
        self.target_idx = 0
        self.localized_on_path = False
        self.last_steer = 0.0
        self.last_time = None
        self.finished = False

        rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=1)
        rospy.Subscriber(self.twist_topic, TwistStamped, self._twist_cb, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.command_topic, Twist, queue_size=1)
        self.path_pub = rospy.Publisher("/planning/stanley_path", Path, queue_size=1, latch=True)
        self.target_idx_pub = rospy.Publisher("/control/stanley_target_index", Int32, queue_size=1)
        self.steer_pub = rospy.Publisher("/control/stanley_steering_rad", Float32, queue_size=1)
        self.raw_steer_pub = rospy.Publisher("/control/stanley_raw_steering_rad", Float32, queue_size=1)
        self.heading_error_pub = rospy.Publisher("/control/stanley_heading_error_rad", Float32, queue_size=1)
        self.cte_pub = rospy.Publisher("/control/stanley_cross_track_error_m", Float32, queue_size=1)
        self.cte_term_pub = rospy.Publisher("/control/stanley_crosstrack_term_rad", Float32, queue_size=1)
        self.path_yaw_pub = rospy.Publisher("/control/stanley_path_yaw_rad", Float32, queue_size=1)
        self.speed_limit_pub = rospy.Publisher("/control/stanley_speed_limit_mps", Float32, queue_size=1)

        self._publish_path()
        rospy.loginfo("Stanley controller ready: %d waypoints from %s", len(self.waypoints), waypoint_file)

    def _load_waypoints(self, path):
        if not path or not os.path.exists(path):
            raise rospy.ROSInitException("waypoint file not found: %s" % path)

        if path.lower().endswith(".csv"):
            points = self._load_csv_waypoints(path)
        else:
            points = self._load_xyz_waypoints(path)

        if len(points) < 2:
            raise rospy.ROSInitException("at least two waypoint rows are required")
        return points

    def _load_csv_waypoints(self, path):
        points = []
        with open(path, newline="") as fp:
            reader = csv.DictReader(fp)
            for row in reader:
                if not row.get("x") or not row.get("y"):
                    continue
                speed = float(row.get("target_speed") or self.target_speed)
                points.append(Waypoint(float(row["x"]), float(row["y"]), speed))
        return points

    def _load_xyz_waypoints(self, path):
        points = []
        with open(path) as fp:
            for line in fp:
                stripped = line.strip()
                if not stripped or stripped.startswith("#"):
                    continue
                cols = stripped.replace(",", " ").split()
                if len(cols) < 2:
                    continue
                points.append(Waypoint(float(cols[0]), float(cols[1]), self.target_speed))
        return points

    def _pose_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

    def _twist_cb(self, msg):
        self.speed = math.hypot(msg.twist.linear.x, msg.twist.linear.y)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.x is None or self.y is None or self.yaw is None:
                rospy.logwarn_throttle(2.0, "waiting for localization")
                rate.sleep()
                continue

            if self.finished:
                self._publish_cmd(0.0, 0.0)
                rate.sleep()
                continue

            result = self._compute_control()
            steer = self._filter_steer(result["raw_steer"])
            self._publish_cmd(result["speed"], steer)
            self.target_idx_pub.publish(Int32(result["target_idx"]))
            self.steer_pub.publish(Float32(steer))
            self.raw_steer_pub.publish(Float32(result["raw_steer"]))
            self.heading_error_pub.publish(Float32(result["heading_error"]))
            self.cte_pub.publish(Float32(result["cte"]))
            self.cte_term_pub.publish(Float32(result["cte_term"]))
            self.path_yaw_pub.publish(Float32(result["path_yaw"]))
            self.speed_limit_pub.publish(Float32(result["speed_limit"]))
            rate.sleep()

    def _compute_control(self):
        front_x = self.x + self.wheelbase * math.cos(self.yaw)
        front_y = self.y + self.wheelbase * math.sin(self.yaw)
        target_idx, proj_x, proj_y, segment_yaw = self._nearest_path_projection(front_x, front_y)
        self.localized_on_path = True
        self.target_idx = max(self.target_idx, target_idx)

        yaw_idx = self._advance_index(target_idx, self.path_yaw_preview)
        yaw_wp = self.waypoints[yaw_idx]
        if math.hypot(yaw_wp.x - proj_x, yaw_wp.y - proj_y) > 1.0e-6:
            path_yaw = math.atan2(yaw_wp.y - proj_y, yaw_wp.x - proj_x)
        else:
            path_yaw = segment_yaw

        heading_error = self._normalize_angle(path_yaw - self.yaw)
        dx = front_x - proj_x
        dy = front_y - proj_y
        cte = dx * math.sin(segment_yaw) - dy * math.cos(segment_yaw)
        cte = self._deadband(cte, self.cross_track_deadband)

        cte_term = math.atan2(self.k * cte, self.speed + self.softening_gain)
        raw_steer = self.heading_error_gain * heading_error + self.crosstrack_error_gain * cte_term
        raw_steer = self._clamp(raw_steer, -self.max_steer, self.max_steer)

        target = self.waypoints[target_idx]
        requested_speed = target.target_speed if self.use_csv_target_speed else self.target_speed
        requested_speed = min(requested_speed, self.max_speed)
        speed_limit = self._curve_speed_limit(requested_speed, abs(heading_error))
        speed = min(requested_speed, speed_limit)

        if self.stop_at_final and target_idx >= len(self.waypoints) - 1:
            final = self.waypoints[-1]
            if math.hypot(self.x - final.x, self.y - final.y) <= self.finish_radius:
                self.finished = True
                return self._result(0.0, 0.0, target_idx, heading_error, cte, cte_term, path_yaw, 0.0)

        return self._result(raw_steer, speed, target_idx, heading_error, cte, cte_term, path_yaw, speed_limit)

    def _nearest_path_projection(self, x, y):
        if not self.localized_on_path:
            start = 0
            end = len(self.waypoints) - 1
        else:
            start = max(0, self.target_idx - self.search_back_window)
            end = min(len(self.waypoints) - 1, self.target_idx + self.search_forward_window)
        if end <= start:
            start = max(0, len(self.waypoints) - 2)
            end = len(self.waypoints) - 1

        best_idx = start
        best_x = self.waypoints[start].x
        best_y = self.waypoints[start].y
        best_yaw = 0.0
        best_dist = float("inf")

        for idx in range(start, end):
            p0 = self.waypoints[idx]
            p1 = self.waypoints[idx + 1]
            seg_x = p1.x - p0.x
            seg_y = p1.y - p0.y
            seg_len_sq = seg_x * seg_x + seg_y * seg_y
            if seg_len_sq <= 1.0e-9:
                continue

            t = self._clamp(((x - p0.x) * seg_x + (y - p0.y) * seg_y) / seg_len_sq, 0.0, 1.0)
            proj_x = p0.x + t * seg_x
            proj_y = p0.y + t * seg_y
            dist = (x - proj_x) ** 2 + (y - proj_y) ** 2
            if dist < best_dist:
                best_idx = idx
                best_x = proj_x
                best_y = proj_y
                best_yaw = math.atan2(seg_y, seg_x)
                best_dist = dist

        return best_idx, best_x, best_y, best_yaw

    def _advance_index(self, start_idx, distance_m):
        traveled = 0.0
        prev = self.waypoints[start_idx]
        for idx in range(start_idx + 1, len(self.waypoints)):
            cur = self.waypoints[idx]
            traveled += math.hypot(cur.x - prev.x, cur.y - prev.y)
            if traveled >= distance_m:
                return idx
            prev = cur
        return len(self.waypoints) - 1

    def _curve_speed_limit(self, base_speed, abs_heading_error):
        if not self.curve_slowdown_enabled or self.curve_heading_slowdown <= 0.0:
            return base_speed
        ratio = self._clamp(abs_heading_error / self.curve_heading_slowdown, 0.0, 1.0)
        min_speed = self._clamp(self.min_curve_speed, 0.0, base_speed)
        return base_speed - ratio * (base_speed - min_speed)

    def _filter_steer(self, raw):
        now = rospy.Time.now()
        if self.last_time is None:
            self.last_time = now
            self.last_steer = 0.0
            return 0.0

        dt = max((now - self.last_time).to_sec(), 1.0 / max(self.rate_hz, 1.0))
        self.last_time = now
        filtered = self.steering_alpha * raw + (1.0 - self.steering_alpha) * self.last_steer
        max_delta = self.max_steer_rate * dt
        delta = self._clamp(filtered - self.last_steer, -max_delta, max_delta)
        self.last_steer = self._clamp(self.last_steer + delta, -self.max_steer, self.max_steer)
        return self.last_steer

    def _publish_cmd(self, speed, steer):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = steer
        self.cmd_pub.publish(msg)

    def _publish_path(self):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time.now()
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    @staticmethod
    def _result(raw_steer, speed, target_idx, heading_error, cte, cte_term, path_yaw, speed_limit):
        return {
            "raw_steer": raw_steer,
            "speed": speed,
            "target_idx": target_idx,
            "heading_error": heading_error,
            "cte": cte,
            "cte_term": cte_term,
            "path_yaw": path_yaw,
            "speed_limit": speed_limit,
        }

    @staticmethod
    def _deadband(value, deadband):
        if deadband <= 0.0:
            return value
        if abs(value) <= deadband:
            return 0.0
        return math.copysign(abs(value) - deadband, value)

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))


if __name__ == "__main__":
    rospy.init_node("stanley_controller")
    StanleyController().run()
