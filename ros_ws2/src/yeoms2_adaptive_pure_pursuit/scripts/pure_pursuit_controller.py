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


class AdaptivePurePursuitController:
    def __init__(self):
        ns = "/adaptive_pure_pursuit"
        self.pose_topic = rospy.get_param(ns + "/pose_topic", "/localization/ego_pose")
        self.twist_topic = rospy.get_param(ns + "/twist_topic", "/localization/ego_twist")
        self.command_topic = rospy.get_param(ns + "/command_topic", "/control/ctrl_cmd")
        self.frame_id = rospy.get_param(ns + "/frame_id", "map")
        self.rate_hz = float(rospy.get_param(ns + "/control_rate_hz", 20.0))
        self.wheelbase = float(rospy.get_param(ns + "/wheelbase_m", 3.0))

        self.base_lookahead = float(rospy.get_param(ns + "/base_lookahead_m", 7.0))
        self.min_lookahead = float(rospy.get_param(ns + "/min_lookahead_m", 3.0))
        self.max_lookahead = float(rospy.get_param(ns + "/max_lookahead_m", 14.0))
        self.lookahead_speed_gain = float(rospy.get_param(ns + "/lookahead_speed_gain", 0.8))
        self.heading_threshold = float(rospy.get_param(ns + "/heading_error_threshold_rad", 0.35))
        self.cte_threshold = float(rospy.get_param(ns + "/cross_track_threshold_m", 1.5))
        self.curvature_threshold = float(rospy.get_param(ns + "/curvature_threshold_radpm", 0.12))
        self.curvature_preview = float(rospy.get_param(ns + "/curvature_preview_m", 12.0))

        self.target_speed = float(rospy.get_param(ns + "/target_speed_mps", 1.0))
        self.max_speed = float(rospy.get_param(ns + "/max_speed_mps", 1.0))
        self.min_curve_speed = float(rospy.get_param(ns + "/min_curve_speed_mps", 2.5))
        self.use_csv_target_speed = bool(rospy.get_param(ns + "/use_csv_target_speed", False))
        self.curve_slowdown_enabled = bool(rospy.get_param(ns + "/curve_slowdown_enabled", True))

        self.max_steer = float(rospy.get_param(ns + "/max_steer_rad", 0.25))
        self.crosstrack_steer_gain = float(rospy.get_param(ns + "/crosstrack_steer_gain", 0.25))
        self.max_crosstrack_steer = float(rospy.get_param(ns + "/max_crosstrack_steer_rad", 0.08))
        self.steering_alpha = self._clamp(float(rospy.get_param(ns + "/steering_filter_alpha", 0.18)), 0.0, 1.0)
        self.max_steer_rate = float(rospy.get_param(ns + "/max_steer_rate_radps", 0.35))

        self.search_back_window = int(rospy.get_param(ns + "/target_search_back_window", 8))
        self.search_forward_window = int(rospy.get_param(ns + "/target_search_forward_window", 120))
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
        self.path_pub = rospy.Publisher("/planning/adaptive_path", Path, queue_size=1, latch=True)
        self.target_idx_pub = rospy.Publisher("/control/adaptive_target_index", Int32, queue_size=1)
        self.steer_pub = rospy.Publisher("/control/adaptive_pp_steering_rad", Float32, queue_size=1)
        self.raw_steer_pub = rospy.Publisher("/control/adaptive_pp_raw_steering_rad", Float32, queue_size=1)
        self.heading_error_pub = rospy.Publisher("/control/adaptive_pp_heading_error_rad", Float32, queue_size=1)
        self.cte_pub = rospy.Publisher("/control/adaptive_pp_cross_track_error_m", Float32, queue_size=1)
        self.lookahead_pub = rospy.Publisher("/control/adaptive_lookahead_distance_m", Float32, queue_size=1)
        self.curvature_pub = rospy.Publisher("/control/adaptive_path_curvature_radpm", Float32, queue_size=1)
        self.adaptive_ratio_pub = rospy.Publisher("/control/adaptive_curve_ratio", Float32, queue_size=1)
        self.speed_limit_pub = rospy.Publisher("/control/adaptive_speed_limit_mps", Float32, queue_size=1)

        self._publish_path()
        rospy.loginfo("Adaptive Pure Pursuit ready: %d waypoints from %s", len(self.waypoints), waypoint_file)

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
            self.lookahead_pub.publish(Float32(result["lookahead"]))
            self.curvature_pub.publish(Float32(result["curvature"]))
            self.adaptive_ratio_pub.publish(Float32(result["adaptive_ratio"]))
            self.speed_limit_pub.publish(Float32(result["speed_limit"]))
            rate.sleep()

    def _compute_control(self):
        nearest = self._nearest_index()
        self.localized_on_path = True
        curvature = self._estimate_path_curvature(nearest, self.curvature_preview)
        cte = self._cross_track_error(nearest)

        base_lookahead = self._clamp(
            self.base_lookahead + self.lookahead_speed_gain * self.speed,
            self.min_lookahead,
            self.max_lookahead,
        )
        probe_idx = self._advance_index(nearest, base_lookahead)
        probe_error = abs(self._target_heading_error(self.waypoints[probe_idx]))
        adaptive_ratio = self._adaptive_ratio(probe_error, abs(cte), abs(curvature))
        lookahead = self._adaptive_lookahead(base_lookahead, adaptive_ratio)

        target_idx = self._advance_index(nearest, lookahead)
        self.target_idx = max(self.target_idx, nearest)
        target = self.waypoints[target_idx]

        distance, heading_error = self._target_distance_and_heading_error(target)
        pure_pursuit_steer = math.atan2(2.0 * self.wheelbase * math.sin(heading_error), distance)
        cte_steer = self._clamp(
            math.atan2(self.crosstrack_steer_gain * cte, max(self.speed, 0.5)),
            -self.max_crosstrack_steer,
            self.max_crosstrack_steer,
        )
        raw_steer = self._clamp(pure_pursuit_steer + cte_steer, -self.max_steer, self.max_steer)

        requested_speed = target.target_speed if self.use_csv_target_speed else self.target_speed
        requested_speed = min(requested_speed, self.max_speed)
        speed_limit = self._adaptive_speed_limit(requested_speed, adaptive_ratio)
        speed = min(requested_speed, speed_limit)

        if self.stop_at_final and target_idx >= len(self.waypoints) - 1:
            final = self.waypoints[-1]
            if math.hypot(self.x - final.x, self.y - final.y) <= self.finish_radius:
                self.finished = True
                return self._result(0.0, 0.0, target_idx, heading_error, cte, lookahead, curvature, adaptive_ratio, 0.0)

        return self._result(raw_steer, speed, target_idx, heading_error, cte, lookahead, curvature, adaptive_ratio, speed_limit)

    def _result(self, raw_steer, speed, target_idx, heading_error, cte, lookahead, curvature, adaptive_ratio, speed_limit):
        return {
            "raw_steer": raw_steer,
            "speed": speed,
            "target_idx": target_idx,
            "heading_error": heading_error,
            "cte": cte,
            "lookahead": lookahead,
            "curvature": curvature,
            "adaptive_ratio": adaptive_ratio,
            "speed_limit": speed_limit,
        }

    def _nearest_index(self):
        if not self.localized_on_path:
            start = 0
            end = len(self.waypoints)
        else:
            start = max(0, self.target_idx - self.search_back_window)
            end = min(len(self.waypoints), self.target_idx + self.search_forward_window)
        best_idx = start
        best_dist = float("inf")
        for idx in range(start, end):
            wp = self.waypoints[idx]
            dist = (self.x - wp.x) ** 2 + (self.y - wp.y) ** 2
            if dist < best_dist:
                best_idx = idx
                best_dist = dist
        return best_idx

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

    def _cross_track_error(self, nearest_idx):
        idx0 = min(nearest_idx, len(self.waypoints) - 2)
        idx1 = idx0 + 1
        p0 = self.waypoints[idx0]
        p1 = self.waypoints[idx1]
        seg_x = p1.x - p0.x
        seg_y = p1.y - p0.y
        seg_len_sq = seg_x * seg_x + seg_y * seg_y
        if seg_len_sq <= 1.0e-9:
            return 0.0

        t = self._clamp(((self.x - p0.x) * seg_x + (self.y - p0.y) * seg_y) / seg_len_sq, 0.0, 1.0)
        proj_x = p0.x + t * seg_x
        proj_y = p0.y + t * seg_y
        seg_yaw = math.atan2(seg_y, seg_x)
        dx = self.x - proj_x
        dy = self.y - proj_y
        return dx * math.sin(seg_yaw) - dy * math.cos(seg_yaw)

    def _adaptive_ratio(self, abs_heading_error, abs_cte, abs_curvature):
        heading_ratio = self._ratio(abs_heading_error, self.heading_threshold)
        cte_ratio = self._ratio(abs_cte, self.cte_threshold)
        curvature_ratio = self._ratio(abs_curvature, self.curvature_threshold)
        return self._clamp(max(heading_ratio, cte_ratio, curvature_ratio), 0.0, 1.0)

    def _adaptive_lookahead(self, base_lookahead, adaptive_ratio):
        return base_lookahead - adaptive_ratio * (base_lookahead - self.min_lookahead)

    def _adaptive_speed_limit(self, base_speed, adaptive_ratio):
        if not self.curve_slowdown_enabled:
            return base_speed
        min_speed = self._clamp(self.min_curve_speed, 0.0, base_speed)
        return base_speed - adaptive_ratio * (base_speed - min_speed)

    def _target_distance_and_heading_error(self, target):
        dx = target.x - self.x
        dy = target.y - self.y
        distance = max(math.hypot(dx, dy), 1.0e-3)
        heading_error = self._normalize_angle(math.atan2(dy, dx) - self.yaw)
        return distance, heading_error

    def _target_heading_error(self, target):
        _, heading_error = self._target_distance_and_heading_error(target)
        return heading_error

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
    def _ratio(value, threshold):
        if threshold <= 0.0:
            return 0.0
        return value / threshold

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))


if __name__ == "__main__":
    rospy.init_node("adaptive_pure_pursuit_controller")
    AdaptivePurePursuitController().run()
