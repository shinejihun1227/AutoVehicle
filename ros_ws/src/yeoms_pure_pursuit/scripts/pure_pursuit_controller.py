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


class PurePursuitController:
    def __init__(self):
        ns = "/pure_pursuit"
        self.pose_topic = rospy.get_param(ns + "/pose_topic", "/localization/ego_pose")
        self.twist_topic = rospy.get_param(ns + "/twist_topic", "/localization/ego_twist")
        self.command_topic = rospy.get_param(ns + "/command_topic", "/control/ctrl_cmd")
        self.frame_id = rospy.get_param(ns + "/frame_id", "map")
        self.rate_hz = float(rospy.get_param(ns + "/control_rate_hz", 20.0))
        self.wheelbase = float(rospy.get_param(ns + "/wheelbase_m", 3.0))
        self.lookahead = float(rospy.get_param(ns + "/lookahead_m", 6.0))
        self.min_lookahead = float(rospy.get_param(ns + "/min_lookahead_m", 4.0))
        self.max_lookahead = float(rospy.get_param(ns + "/max_lookahead_m", 10.0))
        self.lookahead_speed_gain = float(rospy.get_param(ns + "/lookahead_speed_gain", 1.0))
        self.curve_lookahead_enabled = bool(rospy.get_param(ns + "/curve_lookahead_enabled", True))
        self.curve_lookahead_heading = float(rospy.get_param(ns + "/curve_lookahead_heading_rad", 0.35))
        self.curve_min_lookahead = float(rospy.get_param(ns + "/curve_min_lookahead_m", 3.0))
        self.target_speed = float(rospy.get_param(ns + "/target_speed_mps", 1.0))
        self.max_speed = float(rospy.get_param(ns + "/max_speed_mps", 1.0))
        self.use_csv_target_speed = bool(rospy.get_param(ns + "/use_csv_target_speed", False))
        self.curve_slowdown_enabled = bool(rospy.get_param(ns + "/curve_slowdown_enabled", True))
        self.curve_heading_slowdown = float(rospy.get_param(ns + "/curve_heading_slowdown_rad", 0.35))
        self.min_curve_speed = float(rospy.get_param(ns + "/min_curve_speed_mps", 2.5))
        self.max_steer = float(rospy.get_param(ns + "/max_steer_rad", 0.25))
        self.steering_alpha = self._clamp(float(rospy.get_param(ns + "/steering_filter_alpha", 0.2)), 0.0, 1.0)
        self.max_steer_rate = float(rospy.get_param(ns + "/max_steer_rate_radps", 0.3))
        self.search_window = int(rospy.get_param(ns + "/target_search_window", 80))
        self.finish_radius = float(rospy.get_param(ns + "/finish_radius_m", 2.0))
        self.stop_at_final = bool(rospy.get_param(ns + "/stop_at_final_waypoint", True))
        self.start_from_first_waypoint = bool(rospy.get_param(ns + "/start_from_first_waypoint", True))
        self.require_start_pose = bool(rospy.get_param(ns + "/require_start_pose", False))
        self.start_position_tolerance = float(rospy.get_param(ns + "/start_position_tolerance_m", 3.0))
        self.start_yaw_tolerance = float(rospy.get_param(ns + "/start_yaw_tolerance_rad", 0.7))

        waypoint_file = os.path.expanduser(rospy.get_param(ns + "/waypoint_file", ""))
        self.align_path_to_ego_start = bool(rospy.get_param(ns + "/align_path_to_ego_start", True))
        self.rotate_path_to_ego_yaw = bool(rospy.get_param(ns + "/rotate_path_to_ego_yaw", False))
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
        self.path_aligned = False

        rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=1)
        rospy.Subscriber(self.twist_topic, TwistStamped, self._twist_cb, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.command_topic, Twist, queue_size=1)
        self.path_pub = rospy.Publisher("/planning/path", Path, queue_size=1, latch=True)
        self.target_idx_pub = rospy.Publisher("/control/target_index", Int32, queue_size=1)
        self.steer_pub = rospy.Publisher("/control/pure_pursuit_steering_rad", Float32, queue_size=1)
        self.heading_error_pub = rospy.Publisher("/control/pure_pursuit_heading_error_rad", Float32, queue_size=1)
        self.lookahead_pub = rospy.Publisher("/control/lookahead_distance_m", Float32, queue_size=1)
        self.curve_speed_limit_pub = rospy.Publisher("/control/curve_speed_limit_mps", Float32, queue_size=1)
        self.raw_steer_pub = rospy.Publisher("/control/pure_pursuit_raw_steering_rad", Float32, queue_size=1)
        self._publish_path()
        rospy.loginfo("Pure Pursuit ready: %d waypoints from %s", len(self.waypoints), waypoint_file)
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

            self._align_path_to_ego_start_if_needed()

            if self.require_start_pose and not self._start_pose_ready():
                self._publish_cmd(0.0, 0.0)
                self._warn_start_pose()
                rate.sleep()
                continue

            if self.finished:
                self._publish_cmd(0.0, 0.0)
                rate.sleep()
                continue

            steer, speed, target_idx, heading_error, lookahead, curve_speed_limit = self._compute_control()
            raw_steer = steer
            steer = self._filter_steer(steer)
            self._publish_cmd(speed, steer)
            self.target_idx_pub.publish(Int32(target_idx))
            self.steer_pub.publish(Float32(steer))
            self.heading_error_pub.publish(Float32(heading_error))
            self.lookahead_pub.publish(Float32(lookahead))
            self.curve_speed_limit_pub.publish(Float32(curve_speed_limit))
            self.raw_steer_pub.publish(Float32(raw_steer))
            rate.sleep()

    def _compute_control(self):
        nearest = self._nearest_index()
        self.localized_on_path = True
        lookahead = self._clamp(self.lookahead + self.lookahead_speed_gain * self.speed, self.min_lookahead, self.max_lookahead)
        heading_probe_idx = self._advance_index(nearest, lookahead)
        heading_probe = self.waypoints[heading_probe_idx]
        probe_heading_error = self._target_heading_error(heading_probe)
        lookahead = self._apply_curve_lookahead(lookahead, abs(probe_heading_error))
        target_idx = self._advance_index(nearest, lookahead)
        self.target_idx = max(self.target_idx, nearest)
        target = self.waypoints[target_idx]

        distance, heading_error = self._target_distance_and_heading_error(target)
        steer = math.atan2(2.0 * self.wheelbase * math.sin(heading_error), distance)
        steer = self._clamp(steer, -self.max_steer, self.max_steer)

        speed = target.target_speed if self.use_csv_target_speed else self.target_speed
        speed = min(speed, self.max_speed)
        curve_speed_limit = self._curve_speed_limit(speed, abs(heading_error))
        speed = min(speed, curve_speed_limit)
        if self.stop_at_final and target_idx >= len(self.waypoints) - 1:
            final = self.waypoints[-1]
            if math.hypot(self.x - final.x, self.y - final.y) <= self.finish_radius:
                self.finished = True
                return 0.0, 0.0, target_idx, heading_error, lookahead, 0.0
        return steer, speed, target_idx, heading_error, lookahead, curve_speed_limit

    def _target_distance_and_heading_error(self, target):
        dx = target.x - self.x
        dy = target.y - self.y
        distance = max(math.hypot(dx, dy), 1.0e-3)
        heading_error = self._normalize_angle(math.atan2(dy, dx) - self.yaw)
        return distance, heading_error

    def _target_heading_error(self, target):
        _, heading_error = self._target_distance_and_heading_error(target)
        return heading_error

    def _apply_curve_lookahead(self, base_lookahead, abs_heading_error):
        if not self.curve_lookahead_enabled or self.curve_lookahead_heading <= 0.0:
            return base_lookahead
        ratio = self._clamp(abs_heading_error / self.curve_lookahead_heading, 0.0, 1.0)
        curve_min = max(0.1, min(self.curve_min_lookahead, self.max_lookahead))
        return base_lookahead - ratio * (base_lookahead - curve_min)

    def _curve_speed_limit(self, base_speed, abs_heading_error):
        if not self.curve_slowdown_enabled or self.curve_heading_slowdown <= 0.0:
            return base_speed
        ratio = self._clamp(abs_heading_error / self.curve_heading_slowdown, 0.0, 1.0)
        min_speed = self._clamp(self.min_curve_speed, 0.0, base_speed)
        return base_speed - ratio * (base_speed - min_speed)

    def _nearest_index(self):
        if not self.localized_on_path:
            start = 0
            end = len(self.waypoints)
        else:
            start = max(0, self.target_idx - 5)
            end = min(len(self.waypoints), self.target_idx + self.search_window)
        best_idx = start
        best_dist = float("inf")
        for idx in range(start, end):
            wp = self.waypoints[idx]
            dist = (self.x - wp.x) ** 2 + (self.y - wp.y) ** 2
            if dist < best_dist:
                best_idx = idx
                best_dist = dist
        return best_idx

    def _path_start_yaw(self):
        first = self.waypoints[0]
        for idx in range(1, len(self.waypoints)):
            wp = self.waypoints[idx]
            if math.hypot(wp.x - first.x, wp.y - first.y) > 1.0e-6:
                return math.atan2(wp.y - first.y, wp.x - first.x)
        return 0.0

    def _align_path_to_ego_start_if_needed(self):
        if self.path_aligned or not self.align_path_to_ego_start:
            return

        origin = self.waypoints[0]
        origin_x = origin.x
        origin_y = origin.y
        original_start_yaw = self.start_yaw
        yaw_delta = self._normalize_angle(self.yaw - original_start_yaw) if self.rotate_path_to_ego_yaw else 0.0
        cos_yaw = math.cos(yaw_delta)
        sin_yaw = math.sin(yaw_delta)

        for wp in self.waypoints:
            dx = wp.x - origin_x
            dy = wp.y - origin_y
            wp.x = self.x + cos_yaw * dx - sin_yaw * dy
            wp.y = self.y + sin_yaw * dx + cos_yaw * dy

        self.start_yaw = self.yaw
        self.target_idx = 0
        self.localized_on_path = True
        self.path_aligned = True
        self._publish_path()
        rospy.logwarn(
            "Aligned path to ego start pose: original_start=(%.3f, %.3f, %.1f deg), ego_start=(%.3f, %.3f, %.1f deg), rotate=%s",
            origin_x,
            origin_y,
            math.degrees(original_start_yaw),
            self.x,
            self.y,
            math.degrees(self.yaw),
            self.rotate_path_to_ego_yaw,
        )

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
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))


if __name__ == "__main__":
    rospy.init_node("pure_pursuit_controller")
    PurePursuitController().run()
