#!/usr/bin/env python3
import csv
import math
import os
from dataclasses import dataclass

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, Int32


@dataclass
class Waypoint:
    x: float
    y: float
    target_speed: float


class StanleyController:
    def __init__(self):
        ns = "stanley"
        self.pose_topic = rospy.get_param("~pose_topic", rospy.get_param(f"/{ns}/pose_topic", "/localization/ego_pose"))
        self.twist_topic = rospy.get_param("~twist_topic", rospy.get_param(f"/{ns}/twist_topic", "/localization/ego_twist"))
        self.odom_topic = rospy.get_param("~odom_topic", rospy.get_param(f"/{ns}/odom_topic", "/localization/ego_odom"))
        self.use_odometry = rospy.get_param("~use_odometry", rospy.get_param(f"/{ns}/use_odometry", False))
        self.command_topic = rospy.get_param("~command_topic", rospy.get_param(f"/{ns}/command_topic", "/control/ctrl_cmd"))
        self.command_type = rospy.get_param("~command_type", rospy.get_param(f"/{ns}/command_type", "twist"))
        self.frame_id = rospy.get_param("~frame_id", rospy.get_param(f"/{ns}/frame_id", "map"))

        waypoint_file = rospy.get_param(
            "~waypoint_file",
            rospy.get_param(f"/{ns}/waypoint_file", ""),
        )
        self.waypoints = self._load_waypoints(waypoint_file)

        self.rate_hz = float(rospy.get_param("~control_rate_hz", rospy.get_param(f"/{ns}/control_rate_hz", 20.0)))
        self.wheelbase = float(rospy.get_param("~wheelbase_m", rospy.get_param(f"/{ns}/wheelbase_m", 3.0)))
        self.max_steer = float(rospy.get_param("~max_steer_rad", rospy.get_param(f"/{ns}/max_steer_rad", 0.6981317008)))
        self.k = float(rospy.get_param("~stanley_gain", rospy.get_param(f"/{ns}/stanley_gain", 0.7)))
        self.softening_gain = float(rospy.get_param("~softening_gain", rospy.get_param(f"/{ns}/softening_gain", 1.0)))
        self.heading_error_gain = float(rospy.get_param("~heading_error_gain", rospy.get_param(f"/{ns}/heading_error_gain", 1.0)))
        self.crosstrack_error_gain = float(rospy.get_param("~crosstrack_error_gain", rospy.get_param(f"/{ns}/crosstrack_error_gain", 1.0)))
        self.steering_filter_alpha = float(
            rospy.get_param("~steering_filter_alpha", rospy.get_param(f"/{ns}/steering_filter_alpha", 0.35))
        )
        self.max_steer_rate = float(
            rospy.get_param("~max_steer_rate_radps", rospy.get_param(f"/{ns}/max_steer_rate_radps", 0.6))
        )
        self.default_target_speed = float(rospy.get_param("~default_target_speed_mps", rospy.get_param(f"/{ns}/default_target_speed_mps", 4.0)))
        self.speed_kp = float(rospy.get_param("~speed_kp", rospy.get_param(f"/{ns}/speed_kp", 0.35)))
        self.max_accel = float(rospy.get_param("~max_accel_cmd", rospy.get_param(f"/{ns}/max_accel_cmd", 1.0)))
        self.max_brake = float(rospy.get_param("~max_brake_cmd", rospy.get_param(f"/{ns}/max_brake_cmd", 1.0)))
        self.speed_deadband = float(rospy.get_param("~speed_deadband_mps", rospy.get_param(f"/{ns}/speed_deadband_mps", 0.15)))
        self.search_window = int(rospy.get_param("~target_search_window", rospy.get_param(f"/{ns}/target_search_window", 50)))
        self.reached_radius = float(rospy.get_param("~waypoint_reached_radius_m", rospy.get_param(f"/{ns}/waypoint_reached_radius_m", 2.0)))
        self.stop_at_final = bool(rospy.get_param("~stop_at_final_waypoint", rospy.get_param(f"/{ns}/stop_at_final_waypoint", True)))

        self.x = None
        self.y = None
        self.yaw = None
        self.speed = 0.0
        self.target_idx = 0
        self.last_steering = 0.0
        self.last_control_time = None

        if self.use_odometry:
            rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)
        else:
            rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=1)
            rospy.Subscriber(self.twist_topic, TwistStamped, self._twist_cb, queue_size=1)

        self.cmd_pub = self._make_command_publisher()
        self.path_pub = rospy.Publisher("/planning/target_path", Path, queue_size=1, latch=True)
        self.target_idx_pub = rospy.Publisher("/control/stanley_target_index", Int32, queue_size=1)
        self.cte_pub = rospy.Publisher("/control/stanley_cross_track_error", Float32, queue_size=1)
        self.steer_pub = rospy.Publisher("/control/stanley_steering_rad", Float32, queue_size=1)

        self._publish_path()
        rospy.loginfo("Stanley controller ready: %d waypoints, command_type=%s", len(self.waypoints), self.command_type)

    def _load_waypoints(self, path):
        if not path:
            raise rospy.ROSInitException("waypoint_file parameter is required")

        expanded = os.path.expanduser(path)
        if not os.path.exists(expanded):
            raise rospy.ROSInitException(f"waypoint file not found: {expanded}")

        waypoints = []
        with open(expanded, newline="") as fp:
            reader = csv.DictReader(fp)
            for row in reader:
                waypoints.append(
                    Waypoint(
                        x=float(row["x"]),
                        y=float(row["y"]),
                        target_speed=float(row.get("target_speed") or self.default_target_speed),
                    )
                )

        if len(waypoints) < 2:
            raise rospy.ROSInitException("at least two waypoints are required")
        return waypoints

    def _make_command_publisher(self):
        if self.command_type == "morai":
            try:
                from morai_msgs.msg import CtrlCmd
            except ImportError as exc:
                raise rospy.ROSInitException(
                    "command_type=morai requires morai_msgs. Use command_type=twist until MORAI messages are installed."
                ) from exc
            self.CtrlCmd = CtrlCmd
            return rospy.Publisher(self.command_topic, CtrlCmd, queue_size=1)

        self.CtrlCmd = None
        return rospy.Publisher(self.command_topic, Twist, queue_size=1)

    def _pose_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

    def _twist_cb(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.speed = math.hypot(vx, vy)

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed = math.hypot(vx, vy)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.x is None or self.y is None or self.yaw is None:
                rospy.logwarn_throttle(2.0, "waiting for ego pose")
                rate.sleep()
                continue

            steering, target_speed, cte, target_idx = self._compute_control()
            steering = self._filter_steering(steering)
            accel, brake = self._compute_speed_cmd(target_speed)
            self._publish_command(steering, target_speed, accel, brake)
            self.target_idx_pub.publish(Int32(target_idx))
            self.cte_pub.publish(Float32(cte))
            self.steer_pub.publish(Float32(steering))
            rate.sleep()

    def _compute_control(self):
        front_x = self.x + self.wheelbase * math.cos(self.yaw)
        front_y = self.y + self.wheelbase * math.sin(self.yaw)
        target_idx = self._nearest_waypoint_index(front_x, front_y)
        self.target_idx = target_idx

        current_wp = self.waypoints[target_idx]
        next_wp = self.waypoints[min(target_idx + 1, len(self.waypoints) - 1)]
        path_yaw = math.atan2(next_wp.y - current_wp.y, next_wp.x - current_wp.x)

        heading_error = self._normalize_angle(path_yaw - self.yaw)

        dx = front_x - current_wp.x
        dy = front_y - current_wp.y
        # Positive cross-track error means the path is left of the vehicle heading.
        cte = dy * math.cos(path_yaw) - dx * math.sin(path_yaw)

        cte_term = math.atan2(self.k * cte, self.speed + self.softening_gain)
        steering = self.heading_error_gain * heading_error + self.crosstrack_error_gain * cte_term
        steering = self._clamp(steering, -self.max_steer, self.max_steer)

        target_speed = current_wp.target_speed or self.default_target_speed
        if self.stop_at_final and target_idx >= len(self.waypoints) - 1:
            if math.hypot(self.x - current_wp.x, self.y - current_wp.y) <= self.reached_radius:
                target_speed = 0.0

        return steering, target_speed, cte, target_idx

    def _filter_steering(self, raw_steering):
        now = rospy.Time.now()
        if self.last_control_time is None:
            self.last_control_time = now
            self.last_steering = self._clamp(raw_steering, -self.max_steer, self.max_steer)
            return self.last_steering

        dt = max((now - self.last_control_time).to_sec(), 1.0 / max(self.rate_hz, 1.0))
        self.last_control_time = now

        alpha = self._clamp(self.steering_filter_alpha, 0.0, 1.0)
        filtered = alpha * raw_steering + (1.0 - alpha) * self.last_steering

        max_delta = self.max_steer_rate * dt
        delta = self._clamp(filtered - self.last_steering, -max_delta, max_delta)
        self.last_steering = self._clamp(self.last_steering + delta, -self.max_steer, self.max_steer)
        return self.last_steering

    def _nearest_waypoint_index(self, x, y):
        start = max(0, self.target_idx - 5)
        end = min(len(self.waypoints), self.target_idx + self.search_window)
        best_idx = start
        best_dist = float("inf")

        for idx in range(start, end):
            wp = self.waypoints[idx]
            dist = (x - wp.x) ** 2 + (y - wp.y) ** 2
            if dist < best_dist:
                best_idx = idx
                best_dist = dist
        return best_idx

    def _compute_speed_cmd(self, target_speed):
        error = target_speed - self.speed
        if abs(error) < self.speed_deadband:
            return 0.0, 0.0
        if error > 0:
            return self._clamp(self.speed_kp * error, 0.0, self.max_accel), 0.0
        return 0.0, self._clamp(-self.speed_kp * error, 0.0, self.max_brake)

    def _publish_command(self, steering, target_speed, accel, brake):
        if self.command_type == "morai":
            msg = self.CtrlCmd()
            # MORAI CtrlCmd commonly uses longlCmdType. Keep this isolated until message fields are confirmed.
            if hasattr(msg, "longlCmdType"):
                msg.longlCmdType = 1
            if hasattr(msg, "steering"):
                msg.steering = steering
            if hasattr(msg, "accel"):
                msg.accel = accel
            if hasattr(msg, "brake"):
                msg.brake = brake
            if hasattr(msg, "velocity"):
                msg.velocity = target_speed
            self.cmd_pub.publish(msg)
            return

        msg = Twist()
        msg.linear.x = target_speed
        msg.angular.z = steering
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
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))


if __name__ == "__main__":
    rospy.init_node("stanley_controller")
    controller = StanleyController()
    controller.run()
