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


class HybridController:
    def __init__(self):
        ns = "/hybrid_controller"
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
        self.min_curve_speed = float(rospy.get_param(ns + "/min_curve_speed_mps", 2.5))

        self.base_lookahead = float(rospy.get_param(ns + "/base_lookahead_m", 6.0))
        self.min_lookahead = float(rospy.get_param(ns + "/min_lookahead_m", 3.0))
        self.max_lookahead = float(rospy.get_param(ns + "/max_lookahead_m", 12.0))
        self.lookahead_speed_gain = float(rospy.get_param(ns + "/lookahead_speed_gain", 0.8))
        self.curvature_preview = float(rospy.get_param(ns + "/curvature_preview_m", 12.0))
        self.curvature_threshold = float(rospy.get_param(ns + "/curvature_threshold_radpm", 0.12))
        self.heading_threshold = float(rospy.get_param(ns + "/heading_error_threshold_rad", 0.35))
        self.cte_threshold = float(rospy.get_param(ns + "/cross_track_threshold_m", 1.5))

        self.hybrid_stanley_weight = float(rospy.get_param(ns + "/hybrid_stanley_weight", 0.45))
        self.adaptive_blend_enabled = bool(rospy.get_param(ns + "/adaptive_blend_enabled", True))
        self.min_stanley_weight = float(rospy.get_param(ns + "/min_stanley_weight", 0.30))
        self.max_stanley_weight = float(rospy.get_param(ns + "/max_stanley_weight", 0.75))

        self.k = float(rospy.get_param(ns + "/stanley_gain", 0.35))
        self.softening_gain = float(rospy.get_param(ns + "/softening_gain", 2.0))
        self.heading_error_gain = float(rospy.get_param(ns + "/heading_error_gain", 0.8))
        self.crosstrack_error_gain = float(rospy.get_param(ns + "/crosstrack_error_gain", 0.7))
        self.cross_track_deadband = float(rospy.get_param(ns + "/cross_track_deadband_m", 0.05))
        self.pure_pursuit_gain = float(rospy.get_param(ns + "/pure_pursuit_gain", 1.0))

        self.max_steer = float(rospy.get_param(ns + "/max_steer_rad", 0.25))
        self.steering_alpha = self._clamp(float(rospy.get_param(ns + "/steering_filter_alpha", 0.18)), 0.0, 1.0)
        self.max_steer_rate = float(rospy.get_param(ns + "/max_steer_rate_radps", 0.35))

        self.search_back_window = int(rospy.get_param(ns + "/target_search_back_window", 8))
        self.search_forward_window = int(rospy.get_param(ns + "/target_search_forward_window", 120))
        self.path_yaw_preview = float(rospy.get_param(ns + "/path_yaw_preview_m", 4.0))
        self.finish_radius = float(rospy.get_param(ns + "/finish_radius_m", 2.0))
        self.stop_at_final = bool(rospy.get_param(ns + "/stop_at_final_waypoint", True))
        self.start_from_first_waypoint = bool(rospy.get_param(ns + "/start_from_first_waypoint", True))
        self.require_start_pose = bool(rospy.get_param(ns + "/require_start_pose", False))
        self.start_position_tolerance = float(rospy.get_param(ns + "/start_position_tolerance_m", 3.0))
        self.start_yaw_tolerance = float(rospy.get_param(ns + "/start_yaw_tolerance_rad", 0.7))

        waypoint_file = os.path.expanduser(rospy.get_param(ns + "/waypoint_file", ""))
        self.waypoints = self._load_waypoints(waypoint_file)
        self.start_yaw = self._path_start_yaw()

        self.x = None
        self.y = None
        self.yaw = None
        self.speed = 0.0
        self.target_idx = 0
        self.localized_on_path = self.start_from_first_waypoint
        self.last_steer = 0.0
        self.last_time = None
        self.finished = False

        rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=1)
        rospy.Subscriber(self.twist_topic, TwistStamped, self._twist_cb, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.command_topic, Twist, queue_size=1)
        self.path_pub = rospy.Publisher("/planning/hybrid_path", Path, queue_size=1, latch=True)
        self.target_idx_pub = rospy.Publisher("/control/hybrid_target_index", Int32, queue_size=1)
        self.steer_pub = rospy.Publisher("/control/hybrid_steering_rad", Float32, queue_size=1)
        self.raw_steer_pub = rospy.Publisher("/control/hybrid_raw_steering_rad", Float32, queue_size=1)
        self.pp_steer_pub = rospy.Publisher("/control/hybrid_pure_pursuit_raw_steering_rad", Float32, queue_size=1)
        self.stanley_steer_pub = rospy.Publisher("/control/hybrid_stanley_raw_steering_rad", Float32, queue_size=1)
        self.weight_pub = rospy.Publisher("/control/hybrid_stanley_weight", Float32, queue_size=1)
        self.heading_error_pub = rospy.Publisher("/control/hybrid_heading_error_rad", Float32, queue_size=1)
        self.cte_pub = rospy.Publisher("/control/hybrid_cross_track_error_m", Float32, queue_size=1)
        self.lookahead_pub = rospy.Publisher("/control/hybrid_lookahead_distance_m", Float32, queue_size=1)
        self.curvature_pub = rospy.Publisher("/control/hybrid_path_curvature_radpm", Float32, queue_size=1)
        self.speed_limit_pub = rospy.Publisher("/control/hybrid_speed_limit_mps", Float32, queue_size=1)

        self._publish_path()
        rospy.loginfo("Hybrid controller ready: %d waypoints from %s", len(self.waypoints), waypoint_file)
        self._log_required_start_pose()

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

            if self.require_start_pose and not self._start_pose_ready():
                self._publish_cmd(0.0, 0.0)
                self._warn_start_pose()
                rate.sleep()
                continue

            if self.finished:
                self._publish_cmd(0.0, 0.0)
                rate.sleep()
                continue

            result = self._compute_control()
            steer = self._filter_steer(result["raw_steer"])
            self._publish_cmd(result["speed"], steer)
            self._publish_diagnostics(result, steer)
            rate.sleep()

    def _compute_control(self):
        nearest_idx, proj_x, proj_y, segment_yaw = self._nearest_path_projection(self.x, self.y)
        self.localized_on_path = True
        self.target_idx = max(self.target_idx, nearest_idx)
        curvature = self._estimate_path_curvature(nearest_idx, self.curvature_preview)

        cte = self._cross_track_error(self.x, self.y, proj_x, proj_y, segment_yaw)
        cte_db = self._deadband(cte, self.cross_track_deadband)
        path_yaw = self._path_yaw_from_projection(nearest_idx, proj_x, proj_y, segment_yaw)
        heading_error = self._normalize_angle(path_yaw - self.yaw)

        adaptive_ratio = self._adaptive_ratio(abs(heading_error), abs(cte), abs(curvature))
        lookahead = self._lookahead_distance(adaptive_ratio)
        target_idx = self._advance_index(nearest_idx, lookahead)
        target = self.waypoints[target_idx]

        pp_steer = self._pure_pursuit_steer(target)
        stanley_steer = self._stanley_steer(cte_db, heading_error)
        stanley_weight = self._stanley_weight(adaptive_ratio)
        raw_steer = (1.0 - stanley_weight) * pp_steer + stanley_weight * stanley_steer
        raw_steer = self._clamp(raw_steer, -self.max_steer, self.max_steer)

        requested_speed = target.target_speed if self.use_csv_target_speed else self.target_speed
        requested_speed = min(requested_speed, self.max_speed)
        speed_limit = self._adaptive_speed_limit(requested_speed, adaptive_ratio)
        speed = min(requested_speed, speed_limit)

        if self.stop_at_final and target_idx >= len(self.waypoints) - 1:
            final = self.waypoints[-1]
            if math.hypot(self.x - final.x, self.y - final.y) <= self.finish_radius:
                self.finished = True
                return self._result(0.0, 0.0, target_idx, pp_steer, stanley_steer, stanley_weight,
                                    heading_error, cte, lookahead, curvature, 0.0)

        return self._result(raw_steer, speed, target_idx, pp_steer, stanley_steer, stanley_weight,
                            heading_error, cte, lookahead, curvature, speed_limit)

    def _pure_pursuit_steer(self, target):
        dx = target.x - self.x
        dy = target.y - self.y
        distance = max(math.hypot(dx, dy), 1.0e-3)
        heading_error = self._normalize_angle(math.atan2(dy, dx) - self.yaw)
        steer = self.pure_pursuit_gain * math.atan2(2.0 * self.wheelbase * math.sin(heading_error), distance)
        return self._clamp(steer, -self.max_steer, self.max_steer)

    def _stanley_steer(self, cte, heading_error):
        cte_term = math.atan2(self.k * cte, self.speed + self.softening_gain)
        steer = self.heading_error_gain * heading_error + self.crosstrack_error_gain * cte_term
        return self._clamp(steer, -self.max_steer, self.max_steer)

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

    def _path_start_yaw(self):
        first = self.waypoints[0]
        for idx in range(1, len(self.waypoints)):
            wp = self.waypoints[idx]
            if math.hypot(wp.x - first.x, wp.y - first.y) > 1.0e-6:
                return math.atan2(wp.y - first.y, wp.x - first.x)
        return 0.0

    def _start_pose_ready(self):
        pos_err, yaw_err = self._start_pose_error()
        return pos_err <= self.start_position_tolerance and yaw_err <= self.start_yaw_tolerance

    def _start_pose_error(self):
        first = self.waypoints[0]
        pos_err = math.hypot(self.x - first.x, self.y - first.y)
        yaw_err = abs(self._normalize_angle(self.start_yaw - self.yaw))
        return pos_err, yaw_err

    def _log_required_start_pose(self):
        first = self.waypoints[0]
        rospy.loginfo(
            "Required MORAI start pose: x=%.3f y=%.3f yaw=%.3f rad %.1f deg",
            first.x,
            first.y,
            self.start_yaw,
            math.degrees(self.start_yaw),
        )

    def _warn_start_pose(self):
        first = self.waypoints[0]
        pos_err, yaw_err = self._start_pose_error()
        rospy.logwarn_throttle(
            2.0,
            "holding: set MORAI ego near x=%.3f y=%.3f yaw=%.1f deg; current pos_err=%.2f yaw_err=%.1f deg",
            first.x,
            first.y,
            math.degrees(self.start_yaw),
            pos_err,
            math.degrees(yaw_err),
        )

    def _path_yaw_from_projection(self, start_idx, proj_x, proj_y, fallback_yaw):
        yaw_idx = self._advance_index(start_idx, self.path_yaw_preview)
        yaw_wp = self.waypoints[yaw_idx]
        if math.hypot(yaw_wp.x - proj_x, yaw_wp.y - proj_y) <= 1.0e-6:
            return fallback_yaw
        return math.atan2(yaw_wp.y - proj_y, yaw_wp.x - proj_x)

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

    def _estimate_path_curvature(self, start_idx, preview_m):
        mid_idx = self._advance_index(start_idx, max(preview_m * 0.5, 0.1))
        end_idx = self._advance_index(start_idx, max(preview_m, 0.1))
        if mid_idx <= start_idx or end_idx <= mid_idx:
            return 0.0

        p0 = self.waypoints[start_idx]
        p1 = self.waypoints[mid_idx]
        p2 = self.waypoints[end_idx]
        yaw_a = math.atan2(p1.y - p0.y, p1.x - p0.x)
        yaw_b = math.atan2(p2.y - p1.y, p2.x - p1.x)
        dist = max(math.hypot(p2.x - p0.x, p2.y - p0.y), 1.0e-3)
        return self._normalize_angle(yaw_b - yaw_a) / dist

    @staticmethod
    def _cross_track_error(x, y, proj_x, proj_y, segment_yaw):
        dx = x - proj_x
        dy = y - proj_y
        return dx * math.sin(segment_yaw) - dy * math.cos(segment_yaw)

    def _adaptive_ratio(self, abs_heading_error, abs_cte, abs_curvature):
        heading_ratio = self._ratio(abs_heading_error, self.heading_threshold)
        cte_ratio = self._ratio(abs_cte, self.cte_threshold)
        curvature_ratio = self._ratio(abs_curvature, self.curvature_threshold)
        return self._clamp(max(heading_ratio, cte_ratio, curvature_ratio), 0.0, 1.0)

    def _lookahead_distance(self, adaptive_ratio):
        speed_lookahead = self.base_lookahead + self.lookahead_speed_gain * self.speed
        base = self._clamp(speed_lookahead, self.min_lookahead, self.max_lookahead)
        return base - adaptive_ratio * (base - self.min_lookahead)

    def _stanley_weight(self, adaptive_ratio):
        base = self._clamp(self.hybrid_stanley_weight, 0.0, 1.0)
        if not self.adaptive_blend_enabled:
            return base
        low = self._clamp(self.min_stanley_weight, 0.0, 1.0)
        high = self._clamp(self.max_stanley_weight, low, 1.0)
        return self._clamp(base + adaptive_ratio * (high - base), low, high)

    def _adaptive_speed_limit(self, base_speed, adaptive_ratio):
        if not self.curve_slowdown_enabled:
            return base_speed
        min_speed = self._clamp(self.min_curve_speed, 0.0, base_speed)
        return base_speed - adaptive_ratio * (base_speed - min_speed)

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

    def _publish_diagnostics(self, result, steer):
        self.target_idx_pub.publish(Int32(result["target_idx"]))
        self.steer_pub.publish(Float32(steer))
        self.raw_steer_pub.publish(Float32(result["raw_steer"]))
        self.pp_steer_pub.publish(Float32(result["pp_steer"]))
        self.stanley_steer_pub.publish(Float32(result["stanley_steer"]))
        self.weight_pub.publish(Float32(result["stanley_weight"]))
        self.heading_error_pub.publish(Float32(result["heading_error"]))
        self.cte_pub.publish(Float32(result["cte"]))
        self.lookahead_pub.publish(Float32(result["lookahead"]))
        self.curvature_pub.publish(Float32(result["curvature"]))
        self.speed_limit_pub.publish(Float32(result["speed_limit"]))

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
    def _result(raw_steer, speed, target_idx, pp_steer, stanley_steer, stanley_weight,
                heading_error, cte, lookahead, curvature, speed_limit):
        return {
            "raw_steer": raw_steer,
            "speed": speed,
            "target_idx": target_idx,
            "pp_steer": pp_steer,
            "stanley_steer": stanley_steer,
            "stanley_weight": stanley_weight,
            "heading_error": heading_error,
            "cte": cte,
            "lookahead": lookahead,
            "curvature": curvature,
            "speed_limit": speed_limit,
        }

    @staticmethod
    def _ratio(value, threshold):
        if threshold <= 0.0:
            return 0.0
        return value / threshold

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
    rospy.init_node("hybrid_controller")
    HybridController().run()
