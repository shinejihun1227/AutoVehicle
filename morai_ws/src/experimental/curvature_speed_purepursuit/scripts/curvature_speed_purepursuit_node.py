#!/usr/bin/env python3
"""곡률 기반 목표속도 계획과 accel/brake PI를 사용하는 Pure Pursuit.

기본 동작은 /experimental/* 토픽으로 결과를 미리보기만 하는 것이다.
publish_command=true이면 command_topic으로 nominal CtrlCmd를 발행하며,
통합 주행에서는 control_mux의 입력으로 연결한다.
"""

from __future__ import annotations

import math
import os
import time
from typing import Optional, Tuple

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float64

from purepursuit_mgeo.longitudinal_controller import (
    LongitudinalCommand,
    MPS_TO_KPH,
    SpeedPIController,
)
from curvature_speed_purepursuit.planner import (
    PathPoint,
    build_speed_profile,
    clean_consecutive_duplicates,
    cumulative_arc_lengths,
    curvature_profile,
    interpolate_by_s,
    load_path_file,
    nearest_projection,
    profile_value_at_s,
)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def yaw_to_quaternion(yaw_rad: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


class CurvatureSpeedPurePursuitNode:
    def __init__(self) -> None:
        rospy.init_node("curvature_speed_purepursuit", anonymous=False)

        default_path = os.path.join(
            os.environ.get("HOME", "/home"),
            "morai_ws",
            "data",
            "routes",
            "2026_molit_comp_global_path.txt",
        )
        path_file = rospy.get_param("~path_file", default_path)
        raw_points = load_path_file(path_file)
        self.points = clean_consecutive_duplicates(
            raw_points,
            float(rospy.get_param("~duplicate_epsilon_m", 1e-6)),
        )
        self.s_values = cumulative_arc_lengths(self.points)
        self.total_length_m = self.s_values[-1]

        half_window = int(rospy.get_param("~curvature_half_window_points", 1))
        smoothing_window = int(rospy.get_param("~curvature_smoothing_window", 5))
        self.curvatures = curvature_profile(
            self.points,
            half_window_points=max(1, half_window),
            smoothing_window=max(1, smoothing_window),
        )
        legacy_max_speed_mps = float(rospy.get_param("~max_speed_mps", 2.0))
        max_speed_kph = rospy.get_param("~max_speed_kph", None)
        if max_speed_kph is None:
            max_speed_kph = legacy_max_speed_mps * MPS_TO_KPH
        self.max_speed_kph = max(0.0, float(max_speed_kph))

        legacy_initial_speed_mps = float(
            rospy.get_param("~initial_speed_mps", 0.0)
        )
        initial_speed_kph = rospy.get_param("~initial_speed_kph", None)
        if initial_speed_kph is None:
            initial_speed_kph = legacy_initial_speed_mps * MPS_TO_KPH
        initial_speed_mps = float(initial_speed_kph) / MPS_TO_KPH
        if not math.isfinite(initial_speed_mps) or initial_speed_mps < 0.0:
            raise ValueError("initial_speed_kph must be finite and nonnegative")

        legacy_final_speed_mps = float(rospy.get_param("~final_speed_mps", 0.0))
        final_speed_kph = rospy.get_param("~final_speed_kph", None)
        if final_speed_kph is None:
            final_speed_kph = legacy_final_speed_mps * MPS_TO_KPH

        self.speed_profile = build_speed_profile(
            self.s_values,
            self.curvatures,
            max_speed_mps=self.max_speed_kph / MPS_TO_KPH,
            lateral_accel_limit_mps2=float(
                rospy.get_param("~lateral_accel_limit_mps2", 1.0)
            ),
            max_accel_mps2=float(rospy.get_param("~max_accel_mps2", 1.0)),
            max_decel_mps2=float(rospy.get_param("~max_decel_mps2", 1.5)),
            # At rest, a fixed v(s=0)=0 ceiling prevents any departure from s=0.
            # The command starts at zero and apply_speed_rate_limit handles
            # acceleration in time. Keep curve/goal limits in the spatial
            # profile, and preserve an explicitly positive initial-speed cap.
            initial_speed_mps=initial_speed_mps if initial_speed_mps > 0.0 else None,
            final_speed_mps=max(0.0, float(final_speed_kph)) / MPS_TO_KPH,
        )

        self.wheelbase_m = float(rospy.get_param("~wheelbase_m", 3.0))
        self.lookahead_min_m = float(rospy.get_param("~lookahead_min_m", 4.0))
        self.lookahead_gain = float(rospy.get_param("~lookahead_gain", 0.35))
        self.goal_tolerance_m = float(rospy.get_param("~goal_tolerance_m", 1.5))
        self.max_steering_rad = float(
            rospy.get_param("~max_steering_rad", math.radians(40.0))
        )
        self.steering_sign = 1.0 if float(rospy.get_param("~steering_sign", 1.0)) >= 0.0 else -1.0
        self.max_accel_mps2 = max(1e-6, float(rospy.get_param("~max_accel_mps2", 1.0)))
        self.max_decel_mps2 = max(1e-6, float(rospy.get_param("~max_decel_mps2", 1.5)))
        self.rate_hz = max(1.0, float(rospy.get_param("~control_rate_hz", 20.0)))
        self.progress_search_window_points = max(
            10, int(rospy.get_param("~progress_search_window_points", 250))
        )
        self.progress_backtrack_m = max(
            0.0, float(rospy.get_param("~progress_backtrack_tolerance_m", 2.0))
        )

        self.pose_topic = rospy.get_param("~pose_topic", "/localization/odometry")
        self.pose_timeout_sec = max(
            0.0, float(rospy.get_param("~pose_timeout_sec", 0.5))
        )
        self.command_topic = rospy.get_param(
            "~command_topic", "/experimental/curvature_ctrl_cmd"
        )
        self.publish_command = bool(rospy.get_param("~publish_command", False))
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.longl_cmd_type = int(rospy.get_param("~longl_cmd_type", 1))
        self.speed_kp = max(0.0, float(rospy.get_param("~speed_kp", 0.8)))
        self.speed_ki = max(0.0, float(rospy.get_param("~speed_ki", 0.05)))
        self.speed_controller = SpeedPIController(
            kp=self.speed_kp,
            ki=self.speed_ki,
            max_accel_mps2=self.max_accel_mps2,
            max_decel_mps2=self.max_decel_mps2,
            integral_limit_kph_s=max(
                0.0, float(rospy.get_param("~speed_integral_limit_kph_s", 10.8))
            ),
            speed_error_deadband_kph=max(
                0.0, float(rospy.get_param("~speed_error_deadband_kph", 0.1))
            ),
        )

        self.latest_odom: Optional[Odometry] = None
        self.latest_odom_wall_time: Optional[float] = None
        self.last_segment_index: Optional[int] = None
        self.last_progress_s: Optional[float] = None
        self.last_control_time: Optional[float] = None
        self.command_speed_mps = 0.0

        self.command_pub = rospy.Publisher(self.command_topic, CtrlCmd, queue_size=1)
        self.target_pub = rospy.Publisher(
            "/experimental/curvature_lookahead_point", PointStamped, queue_size=1
        )
        self.reference_path_pub = rospy.Publisher(
            "/experimental/curvature_reference_path", Path, queue_size=1, latch=True
        )
        self.curvature_pub = rospy.Publisher(
            "/experimental/curvature_value", Float64, queue_size=1
        )
        self.speed_limit_pub = rospy.Publisher(
            "/experimental/curvature_speed_limit", Float64, queue_size=1
        )
        self.speed_command_pub = rospy.Publisher(
            "/experimental/curvature_speed_command", Float64, queue_size=1
        )
        self.accel_command_pub = rospy.Publisher(
            "/experimental/curvature_accel_command", Float64, queue_size=1
        )
        self.brake_command_pub = rospy.Publisher(
            "/experimental/curvature_brake_command", Float64, queue_size=1
        )
        self.steering_pub = rospy.Publisher(
            "/experimental/curvature_steering", Float64, queue_size=1
        )
        self.progress_pub = rospy.Publisher(
            "/experimental/curvature_progress", Float64, queue_size=1
        )
        self.goal_pub = rospy.Publisher(
            "/experimental/curvature_goal_reached", Bool, queue_size=1, latch=True
        )

        rospy.Subscriber(self.pose_topic, Odometry, self.odom_callback, queue_size=10)
        self.publish_reference_path()
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.rate_hz), self.control_callback
        )

        rospy.logwarn(
            "Curvature PP isolated mode: path=%s raw_points=%d cleaned_points=%d "
            "length=%.2fm command=%s publish_command=%s",
            path_file,
            len(raw_points),
            len(self.points),
            self.total_length_m,
            self.command_topic,
            self.publish_command,
        )

    def publish_reference_path(self) -> None:
        message = Path()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.map_frame
        for point in self.points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = point.z
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.reference_path_pub.publish(message)

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message
        self.latest_odom_wall_time = time.monotonic()

    def search_projection(self, x: float, y: float):
        if self.last_segment_index is None:
            return nearest_projection(self.points, self.s_values, x, y)

        start = max(0, self.last_segment_index - self.progress_search_window_points)
        end = min(
            len(self.points) - 2,
            self.last_segment_index + self.progress_search_window_points,
        )
        projection = nearest_projection(
            self.points,
            self.s_values,
            x,
            y,
            start_segment=start,
            end_segment=end,
        )
        if (
            self.last_progress_s is not None
            and projection.progress_s < self.last_progress_s - self.progress_backtrack_m
        ):
            return nearest_projection(
                self.points,
                self.s_values,
                x,
                y,
                start_segment=self.last_segment_index,
                end_segment=end,
            )
        return projection

    def apply_speed_rate_limit(self, target_speed: float, dt: float) -> float:
        target = max(0.0, float(target_speed))
        delta = target - self.command_speed_mps
        if delta >= 0.0:
            allowed = self.max_accel_mps2 * max(dt, 1e-3)
        else:
            allowed = self.max_decel_mps2 * max(dt, 1e-3)
        self.command_speed_mps += clamp(delta, -allowed, allowed)
        self.command_speed_mps = max(0.0, self.command_speed_mps)
        return self.command_speed_mps

    def compute_steering(
        self, x: float, y: float, yaw: float, speed_mps: float, progress_s: float
    ) -> Tuple[float, PathPoint, float]:
        lookahead = max(
            self.lookahead_min_m,
            self.lookahead_min_m + self.lookahead_gain * max(0.0, speed_mps),
        )
        target, _ = interpolate_by_s(
            self.points,
            self.s_values,
            min(self.total_length_m, progress_s + lookahead),
        )
        dx = target.x - x
        dy = target.y - y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        target_x_body = cos_yaw * dx + sin_yaw * dy
        target_y_body = -sin_yaw * dx + cos_yaw * dy
        actual_lookahead = max(math.hypot(target_x_body, target_y_body), 1e-3)
        alpha = math.atan2(target_y_body, target_x_body)
        curvature = 2.0 * math.sin(alpha) / actual_lookahead
        steering = math.atan(self.wheelbase_m * curvature) * self.steering_sign
        return clamp(steering, -self.max_steering_rad, self.max_steering_rad), target, actual_lookahead

    def make_command(
        self,
        steering: float,
        target_speed_kph: float,
        measured_speed_kph: float,
        stop: bool,
        dt: float,
        pedal: Optional[LongitudinalCommand] = None,
    ) -> CtrlCmd:
        command = CtrlCmd()
        if hasattr(command, "longlCmdType"):
            command.longlCmdType = self.longl_cmd_type
        if hasattr(command, "steering"):
            command.steering = 0.0 if stop else steering

        if pedal is None:
            pedal = self.speed_controller.update(
                target_speed_kph,
                measured_speed_kph,
                dt,
                stop=stop,
            )
        if self.longl_cmd_type == 1:
            if hasattr(command, "brake"):
                command.brake = pedal.brake
            if hasattr(command, "accel"):
                command.accel = pedal.accel
            if hasattr(command, "acceleration"):
                command.acceleration = 0.0
            if hasattr(command, "velocity"):
                command.velocity = 0.0
        else:
            self.speed_controller.reset()
            if hasattr(command, "brake"):
                command.brake = 1.0 if stop else 0.0
            if hasattr(command, "accel"):
                command.accel = 0.0
            if hasattr(command, "acceleration"):
                command.acceleration = 0.0
            if hasattr(command, "velocity"):
                command.velocity = (
                    0.0
                    if stop
                    else max(0.0, target_speed_kph) / MPS_TO_KPH
                )
        return command

    def control_callback(self, _event: rospy.timer.TimerEvent) -> None:
        if self.latest_odom is None:
            rospy.logwarn_throttle(
                5.0,
                "곡률 기반 Pure Pursuit가 %s를 기다리는 중이다.",
                self.pose_topic,
            )
            return

        if (
            self.latest_odom_wall_time is None
            or time.monotonic() - self.latest_odom_wall_time > self.pose_timeout_sec
        ):
            self.command_speed_mps = 0.0
            self.speed_controller.reset()
            self.speed_command_pub.publish(Float64(0.0))
            self.goal_pub.publish(Bool(False))
            if self.publish_command:
                self.command_pub.publish(
                    self.make_command(0.0, 0.0, 0.0, stop=True, dt=1.0 / self.rate_hz)
                )
            rospy.logwarn_throttle(
                2.0,
                "곡률 기반 Pure Pursuit가 %s의 최신 pose를 받지 못해 정지 명령을 발행한다.",
                self.pose_topic,
            )
            return

        now = rospy.Time.now().to_sec()
        dt = 1.0 / self.rate_hz if self.last_control_time is None else now - self.last_control_time
        self.last_control_time = now
        dt = clamp(dt, 1e-3, 0.25)

        pose = self.latest_odom.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        measured_speed_mps = math.hypot(
            self.latest_odom.twist.twist.linear.x,
            self.latest_odom.twist.twist.linear.y,
        )
        measured_speed_kph = measured_speed_mps * MPS_TO_KPH

        projection = self.search_projection(x, y)
        self.last_segment_index = projection.segment_index
        progress_s = projection.progress_s
        if self.last_progress_s is not None:
            progress_s = max(progress_s, self.last_progress_s)
        progress_s = min(progress_s, self.total_length_m)
        self.last_progress_s = progress_s

        remaining_m = max(0.0, self.total_length_m - progress_s)
        stop = remaining_m <= self.goal_tolerance_m
        speed_limit = profile_value_at_s(self.s_values, self.speed_profile, progress_s)
        command_speed = 0.0 if stop else self.apply_speed_rate_limit(speed_limit, dt)
        curvature = profile_value_at_s(self.s_values, self.curvatures, progress_s)

        if stop:
            steering = 0.0
            target = self.points[-1]
            actual_lookahead = 0.0
        else:
            steering, target, actual_lookahead = self.compute_steering(
                x, y, yaw, measured_speed_mps, progress_s
            )

        target_message = PointStamped()
        target_message.header.stamp = rospy.Time.now()
        target_message.header.frame_id = self.map_frame
        target_message.point.x = target.x
        target_message.point.y = target.y
        target_message.point.z = target.z
        self.target_pub.publish(target_message)

        self.curvature_pub.publish(Float64(curvature))
        speed_limit_kph = speed_limit * MPS_TO_KPH
        command_speed_kph = command_speed * MPS_TO_KPH
        pedal = self.speed_controller.update(
            command_speed_kph,
            measured_speed_kph,
            dt,
            stop=stop,
        )
        self.speed_limit_pub.publish(Float64(speed_limit_kph))
        self.speed_command_pub.publish(Float64(command_speed_kph))
        self.accel_command_pub.publish(Float64(pedal.accel))
        self.brake_command_pub.publish(Float64(pedal.brake))
        self.steering_pub.publish(Float64(steering))
        self.progress_pub.publish(Float64(progress_s))
        self.goal_pub.publish(Bool(stop))

        if self.publish_command:
            self.command_pub.publish(
                self.make_command(
                    steering,
                    command_speed_kph,
                    measured_speed_kph,
                    stop,
                    dt,
                    pedal=pedal,
                )
            )

        rospy.loginfo_throttle(
            2.0,
            "Curvature PP progress=%.1f/%.1fm kappa=%.4f speed_limit=%.2f "
            "speed_cmd=%.2f measured=%.2f km/h accel=%.2f brake=%.2f "
            "lookahead=%.2f steering=%.4f stop=%s",
            progress_s,
            self.total_length_m,
            curvature,
            speed_limit_kph,
            command_speed_kph,
            measured_speed_kph,
            pedal.accel,
            pedal.brake,
            actual_lookahead,
            steering,
            stop,
        )


if __name__ == "__main__":
    try:
        CurvatureSpeedPurePursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
