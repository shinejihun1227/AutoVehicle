#!/usr/bin/env python3
"""곡률 기반 목표속도 계획과 accel/brake PI를 사용하는 Pure Pursuit.

기본 동작은 /experimental/* 토픽으로 결과를 미리보기만 하는 것이다.
publish_command=true이면 command_topic으로 nominal CtrlCmd를 발행하며,
통합 주행에서는 control_mux의 입력으로 연결한다.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from bisect import bisect_left, bisect_right
from collections import deque
from typing import Optional, Tuple

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from morai_perception_msgs.msg import StopLineDetection
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float64, String

from purepursuit_mgeo.longitudinal_controller import (
    LongitudinalCommand,
    MPS_TO_KPH,
    SpeedPIController,
    VehicleSpeedSource,
)
from curvature_speed_purepursuit.planner import (
    PathPoint,
    build_speed_profile,
    clean_consecutive_duplicates,
    cumulative_arc_lengths,
    curvature_profile,
    conservative_speed_curvatures,
    steering_curvature_profile,
    bridge_reversing_curve_gaps,
    adaptive_lookahead_m,
    interpolate_by_s,
    max_abs_curvature_ahead,
    minimum_speed_ahead,
    load_path_file,
    nearest_projection,
    profile_value_at_s,
)
from curvature_speed_purepursuit.lane_centering import (
    LaneCenteringAssist, ideal_lane_reference, lane_quality, lane_measurement,
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
        self.curvatures = (
            steering_curvature_profile(self.points, self.s_values)
            if bool(rospy.get_param("~metre_sampled_steering_curvature", False))
            else curvature_profile(
                self.points,
                half_window_points=max(1, half_window),
                smoothing_window=max(1, smoothing_window),
            )
        )
        self.speed_curvatures = (conservative_speed_curvatures(self.points, self.s_values)
                                 if bool(rospy.get_param("~conservative_curve_speed_enabled", False))
                                 else self.curvatures)
        if bool(rospy.get_param("~bridge_reversing_curves", False)):
            self.speed_curvatures = bridge_reversing_curve_gaps(
                self.s_values, self.curvatures, self.speed_curvatures)
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

        max_decel_mps2 = float(rospy.get_param("~max_decel_mps2", 1.5))
        self.speed_profile = build_speed_profile(
            self.s_values,
            self.speed_curvatures,
            max_speed_mps=self.max_speed_kph / MPS_TO_KPH,
            lateral_accel_limit_mps2=float(
                rospy.get_param("~lateral_accel_limit_mps2", 1.0)
            ),
            max_accel_mps2=float(rospy.get_param("~max_accel_mps2", 1.0)),
            max_decel_mps2=min(max_decel_mps2, float(rospy.get_param(
                "~curve_planning_decel_mps2", max_decel_mps2))),
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
        self.lookahead_curvature_gain = max(
            0.0, float(rospy.get_param("~lookahead_curvature_gain", 6.0))
        )
        self.lookahead_tight_min_m = max(
            0.5, float(rospy.get_param("~lookahead_tight_min_m", 2.2))
        )
        self.lookahead_max_m = max(
            self.lookahead_tight_min_m,
            float(rospy.get_param("~lookahead_max_m", 12.0)),
        )
        self.curvature_preview_distance_m = max(
            0.0,
            float(rospy.get_param("~curvature_preview_distance_m", 8.0)),
        )
        self.curvature_preview_step_m = max(
            0.1,
            float(rospy.get_param("~curvature_preview_step_m", 0.5)),
        )
        self.steering_feedforward_weight = clamp(
            float(rospy.get_param("~steering_feedforward_weight", 0.35)),
            0.0,
            1.0,
        )
        self.path_lateral_feedback_gain = max(
            0.0, float(rospy.get_param("~path_lateral_feedback_gain", 0.0))
        )
        self.path_lateral_feedback_max_rad = clamp(
            float(rospy.get_param("~path_lateral_feedback_max_rad", 0.0)),
            0.0, 0.12,
        )
        self.goal_tolerance_m = float(rospy.get_param("~goal_tolerance_m", 1.5))
        self.max_steering_rad = float(
            rospy.get_param("~max_steering_rad", math.radians(40.0))
        )
        self.steering_sign = 1.0 if float(rospy.get_param("~steering_sign", 1.0)) >= 0.0 else -1.0
        self.max_steering_rate_rad_s = max(
            0.0,
            float(rospy.get_param("~max_steering_rate_rad_s", 2.5)),
        )
        self.enable_lane_centering = bool(rospy.get_param("~enable_lane_centering", False))
        self.lane_info_topic = str(rospy.get_param("~lane_info_topic", "/perception/camera/lane_info"))
        self.lane_centering_weight = float(rospy.get_param("~lane_centering_weight", 0.25))
        self.lane_centering_max_correction_rad = float(
            rospy.get_param("~lane_centering_max_correction_rad", 0.045)
        )
        self.lane_centering_min_confidence = float(
            rospy.get_param("~lane_centering_min_confidence", 0.65)
        )
        self.lane_centering_timeout_sec = float(
            rospy.get_param("~lane_centering_timeout_sec", 0.4)
        )
        if (not all(math.isfinite(value) for value in (
                self.lane_centering_weight,
                self.lane_centering_max_correction_rad,
                self.lane_centering_min_confidence,
                self.lane_centering_timeout_sec,
            ))
                or not 0.0 <= self.lane_centering_weight <= 1.0
                or not 0.0 <= self.lane_centering_max_correction_rad <= 0.15
                or not 0.0 <= self.lane_centering_min_confidence <= 1.0
                or not 0.0 < self.lane_centering_timeout_sec <= 1.0):
            raise ValueError("Lane centering settings must be finite and in range")
        self.lane_lock = threading.RLock()
        self.lane_assist = LaneCenteringAssist(
            weight=self.lane_centering_weight,
            max_correction_rad=self.lane_centering_max_correction_rad,
            timeout_sec=self.lane_centering_timeout_sec,
            min_frames=int(rospy.get_param("~lane_centering_min_frames", 4)),
            filter_tau_sec=float(rospy.get_param("~lane_centering_filter_tau_sec", 0.35)),
            correction_rate_rad_s=float(rospy.get_param("~lane_centering_correction_rate_rad_s", 0.06)),
            release_rate_rad_s=float(rospy.get_param("~lane_centering_release_rate_rad_s", 0.10)),
        )
        self.lane_observations = deque(maxlen=10)
        self.lane_pose_history = deque(maxlen=200)
        self.last_lane_stamp = 0.0
        self.max_accel_mps2 = max(1e-6, float(rospy.get_param("~max_accel_mps2", 1.0)))
        self.max_decel_mps2 = max(1e-6, float(rospy.get_param("~max_decel_mps2", 1.5)))
        self.rate_hz = max(1.0, float(rospy.get_param("~control_rate_hz", 20.0)))
        self.progress_search_window_points = max(
            10, int(rospy.get_param("~progress_search_window_points", 250))
        )
        self.progress_backtrack_m = max(
            0.0, float(rospy.get_param("~progress_backtrack_tolerance_m", 2.0))
        )
        self.stopline_speed_cap_enabled = bool(
            rospy.get_param("~stopline_speed_cap_enabled", False)
        )
        self.stopline_approach_speed_kph = float(
            rospy.get_param("~stopline_approach_speed_kph", 30.0)
        )
        self.stopline_cap_release_after_m = float(
            rospy.get_param("~stopline_cap_release_after_m", 5.0)
        )
        self.stopline_cap_max_detection_range_m = float(
            rospy.get_param("~stopline_cap_max_detection_range_m", 60.0)
        )
        self.stopline_cap_min_confidence = float(
            rospy.get_param("~stopline_cap_min_confidence", 0.55)
        )
        if (not math.isfinite(self.stopline_approach_speed_kph)
                or self.stopline_approach_speed_kph <= 0
                or not math.isfinite(self.stopline_cap_release_after_m)
                or self.stopline_cap_release_after_m < 0
                or not math.isfinite(self.stopline_cap_max_detection_range_m)
                or self.stopline_cap_max_detection_range_m <= 0
                or not 0 <= self.stopline_cap_min_confidence <= 1):
            raise ValueError("Stopline speed-cap settings must be finite and in range")
        self.stopline_lock = threading.RLock()
        self.progress_history = deque(maxlen=200)
        self.latest_stopline = None
        self.last_stopline_stamp = 0.0
        self.active_stopline_s = None

        self.pose_topic = rospy.get_param("~pose_topic", "/localization/odometry")
        self.pose_timeout_sec = max(
            0.0, float(rospy.get_param("~pose_timeout_sec", 0.5))
        )
        self.vehicle_speed_topic = str(rospy.get_param("~vehicle_speed_topic", "")).strip()
        self.require_vehicle_speed = bool(rospy.get_param("~require_vehicle_speed", False))
        if self.require_vehicle_speed and not self.vehicle_speed_topic:
            raise ValueError("require_vehicle_speed needs vehicle_speed_topic")
        self.vehicle_speed_source = VehicleSpeedSource(
            timeout_sec=float(rospy.get_param("~vehicle_speed_timeout_sec", 0.5)))
        self.speed_preview_time_sec = float(rospy.get_param("~speed_preview_time_sec", 0.8))
        if not math.isfinite(self.speed_preview_time_sec) or not 0.0 <= self.speed_preview_time_sec <= 2.0:
            raise ValueError("speed_preview_time_sec must be between 0 and 2 seconds")
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
            accel_rise_rate_per_sec=float(rospy.get_param("~pedal_accel_rise_rate_per_sec", 0.0)),
            brake_rise_rate_per_sec=float(rospy.get_param("~pedal_brake_rise_rate_per_sec", 0.0)),
            pedal_release_rate_per_sec=float(rospy.get_param("~pedal_release_rate_per_sec", 0.0)),
        )

        self.latest_odom: Optional[Odometry] = None
        self.latest_odom_wall_time: Optional[float] = None
        self.last_segment_index: Optional[int] = None
        self.last_progress_s: Optional[float] = None
        self.last_control_time: Optional[float] = None
        self.command_speed_mps = 0.0
        self.last_steering_rad = 0.0

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
        self.measured_speed_pub = rospy.Publisher(
            "/experimental/curvature_measured_speed_kph", Float64, queue_size=1
        )
        self.odom_speed_pub = rospy.Publisher(
            "/experimental/curvature_odom_speed_kph", Float64, queue_size=1
        )
        self.vehicle_speed_fresh_pub = rospy.Publisher(
            "/experimental/curvature_vehicle_speed_fresh", Bool, queue_size=1
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
        self.path_lateral_error_pub = rospy.Publisher(
            "/experimental/curvature_path_lateral_error_m", Float64, queue_size=1
        )
        self.lane_correction_pub = rospy.Publisher(
            "/experimental/curvature_lane_correction_rad", Float64, queue_size=1
        )
        self.lane_correction_active_pub = rospy.Publisher(
            "/experimental/curvature_lane_correction_active", Bool, queue_size=1
        )
        self.lane_centering_status_pub = rospy.Publisher(
            "/experimental/curvature_lane_centering_status", String, queue_size=1
        )
        self.progress_pub = rospy.Publisher(
            "/experimental/curvature_progress", Float64, queue_size=1
        )
        self.stopline_cap_active_pub = rospy.Publisher(
            "/experimental/stopline_speed_cap_active", Bool, queue_size=1
        )
        self.stopline_cap_target_pub = rospy.Publisher(
            "/experimental/stopline_speed_cap_target", Float64, queue_size=1
        )
        self.goal_pub = rospy.Publisher(
            "/experimental/curvature_goal_reached", Bool, queue_size=1, latch=True
        )

        rospy.Subscriber(self.pose_topic, Odometry, self.odom_callback, queue_size=10)
        if self.vehicle_speed_topic:
            rospy.Subscriber(self.vehicle_speed_topic, EgoVehicleStatus,
                             self.vehicle_speed_callback, queue_size=10)
        if self.enable_lane_centering:
            rospy.Subscriber(
                self.lane_info_topic,
                String, self.lane_info_callback, queue_size=10,
            )
        if self.stopline_speed_cap_enabled:
            rospy.Subscriber(
                rospy.get_param("~stopline_topic", "/perception/camera/stopline"),
                StopLineDetection,
                self.stopline_callback,
                queue_size=10,
            )
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
        if self.enable_lane_centering:
            stamp = message.header.stamp.to_sec()
            with self.lane_lock:
                if (self.lane_pose_history
                        and rospy.Time.now().to_sec() < self.lane_pose_history[-1][0] - 0.05):
                    self.lane_pose_history.clear()
                    self.lane_observations.clear()
                    self.last_lane_stamp = 0.0
                    self.lane_assist.invalidate("pose_unsynchronized")
                if (math.isfinite(stamp) and stamp > 0
                        and (not self.lane_pose_history or stamp > self.lane_pose_history[-1][0])):
                    self.lane_pose_history.append((stamp, message))

    def vehicle_speed_callback(self, message: EgoVehicleStatus) -> None:
        self.vehicle_speed_source.observe(
            math.hypot(float(message.velocity.x), float(message.velocity.y)),
            message.header.stamp.to_sec(), rospy.Time.now().to_sec(), time.monotonic())

    def lane_info_callback(self, message: String) -> None:
        """Use actual two-sided CAM1 geometry and its original observation time.

        Legacy LaneDetection omits observed/coasted/guide flags and rejects
        curve cross-section widths as though they were perpendicular widths.
        This assist independently checks normal widths and measured boundaries.
        """
        with self.lane_lock:
            try:
                info = json.loads(message.data, parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v)))
                stamp = float(info["timestamp"])
                now = rospy.Time.now().to_sec()
                if now < self.last_lane_stamp - 0.05:
                    self.last_lane_stamp = 0.0
                    self.lane_observations.clear()
                    self.lane_pose_history.clear()
                    self.lane_assist.invalidate("pose_unsynchronized")
                if (not math.isfinite(stamp) or stamp <= 0
                        or not -0.05 <= now - stamp <= self.lane_centering_timeout_sec):
                    self.lane_assist.invalidate("stale")
                    self.lane_observations.clear()
                    return
                if stamp <= self.last_lane_stamp:
                    return
                self.last_lane_stamp = stamp
                valid, quality, reason, confidence = lane_quality(info, self.lane_centering_min_confidence)
                measurement = lane_measurement(info) if valid else None
                if measurement is None:
                    self.lane_assist.invalidate(reason if not valid else "plausibility_failed")
                    self.lane_observations.clear()
                    return
                if len(self.lane_observations) == self.lane_observations.maxlen:
                    self.lane_assist.invalidate("unstable")
                    self.lane_observations.clear()
                self.lane_observations.append((stamp, time.monotonic(), measurement[0],
                                               measurement[1], confidence, quality))
            except (TypeError, KeyError, ValueError, AttributeError, OverflowError):
                self.lane_observations.clear()
                self.lane_assist.invalidate("plausibility_failed")

    def stopline_callback(self, message: StopLineDetection) -> None:
        """Cache only the newest camera observation; the control timer matches its pose."""
        if not self.stopline_speed_cap_enabled:
            return
        stamp = message.header.stamp.to_sec()
        if not math.isfinite(stamp) or stamp <= 0:
            return
        with self.stopline_lock:
            if self.latest_stopline is None or stamp > self.latest_stopline[0]:
                self.latest_stopline = (stamp, message)

    def update_stopline_speed_cap(self, progress_s: float, pose_stamp: float, ros_now: float) -> bool:
        """Cap approach speed on a measured line, then release after the vehicle clears it."""
        if not self.stopline_speed_cap_enabled:
            return False
        with self.stopline_lock:
            if math.isfinite(pose_stamp) and pose_stamp > 0:
                self.progress_history.append((pose_stamp, progress_s))
            observation = self.latest_stopline
            if observation is not None and observation[0] > self.last_stopline_stamp:
                stamp, message = observation
                self.last_stopline_stamp = stamp
                age = ros_now - stamp
                line = message
                if (-0.05 <= age <= 0.8 and line.valid
                        and line.header.frame_id == "base_link"
                        and math.isfinite(line.distance_m)
                        and 0 <= line.distance_m <= self.stopline_cap_max_detection_range_m
                        and math.isfinite(line.confidence)
                        and self.stopline_cap_min_confidence <= line.confidence <= 1.0):
                    pose = min(self.progress_history, key=lambda item: abs(item[0] - stamp), default=None)
                    if pose is not None and abs(pose[0] - stamp) <= 0.1:
                        line_s = pose[1] + line.distance_m
                        if line_s >= progress_s - self.stopline_cap_release_after_m:
                            if self.active_stopline_s is None:
                                self.active_stopline_s = line_s
                            elif line_s > self.active_stopline_s + 6.0:
                                # Keep the cap across closely spaced lines by moving the target forward.
                                self.active_stopline_s = line_s
                            elif line_s >= self.active_stopline_s - 2.0:
                                self.active_stopline_s = min(self.active_stopline_s, line_s)
            if (self.active_stopline_s is not None
                    and progress_s > self.active_stopline_s + self.stopline_cap_release_after_m):
                self.active_stopline_s = None
            return self.active_stopline_s is not None

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
        target = max(0.0, min(float(target_speed), self.max_speed_kph / MPS_TO_KPH))
        delta = target - self.command_speed_mps
        if delta >= 0.0:
            allowed = self.max_accel_mps2 * max(dt, 1e-3)
        else:
            # The spatial profile already includes a braking-distance pass.
            # A second descending target ramp would command a speed ABOVE
            # that ceiling precisely when entering the bend. Pedal slew below
            # provides smooth actuation without delaying the speed constraint.
            self.command_speed_mps = target
            return target
        self.command_speed_mps += clamp(delta, -allowed, allowed)
        self.command_speed_mps = max(0.0, self.command_speed_mps)
        return self.command_speed_mps

    def compute_steering(
        self, x: float, y: float, yaw: float, speed_mps: float, progress_s: float,
        path_lateral_error_m: float = 0.0,
    ) -> Tuple[float, PathPoint, float]:
        # Keep this method usable by lightweight offline turn tests and tools
        # that construct the controller with ``__new__`` and only provide the
        # original Pure Pursuit fields.
        curvatures = getattr(self, "curvatures", [0.0] * len(self.points))
        curvature_preview_distance = getattr(
            self, "curvature_preview_distance_m", 0.0
        )
        curvature_preview_step = getattr(self, "curvature_preview_step_m", 0.5)
        curvature_gain = getattr(self, "lookahead_curvature_gain", 0.0)
        tight_min_lookahead = getattr(self, "lookahead_tight_min_m", self.lookahead_min_m)
        max_lookahead = getattr(
            self,
            "lookahead_max_m",
            1000.0,
        )
        feedforward_weight = getattr(self, "steering_feedforward_weight", 0.0)
        nominal_lookahead = max(
            self.lookahead_min_m,
            self.lookahead_min_m + self.lookahead_gain * max(0.0, speed_mps),
        )
        preview_curvature = max_abs_curvature_ahead(
            self.s_values,
            curvatures,
            progress_s,
            max(curvature_preview_distance, nominal_lookahead),
            curvature_preview_step,
        )
        lookahead = adaptive_lookahead_m(
            speed_mps=speed_mps,
            base_lookahead_m=self.lookahead_min_m,
            speed_gain_s=self.lookahead_gain,
            preview_curvature_abs_m_inv=preview_curvature,
            curvature_gain_m=curvature_gain,
            tight_min_lookahead_m=tight_min_lookahead,
            max_lookahead_m=max_lookahead,
        )
        target_s = min(self.total_length_m, progress_s + lookahead)
        target, _ = interpolate_by_s(
            self.points,
            self.s_values,
            target_s,
        )
        dx = target.x - x
        dy = target.y - y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        target_x_body = cos_yaw * dx + sin_yaw * dy
        target_y_body = -sin_yaw * dx + cos_yaw * dy
        actual_lookahead = max(math.hypot(target_x_body, target_y_body), 1e-3)
        alpha = math.atan2(target_y_body, target_x_body)
        pp_curvature = 2.0 * math.sin(alpha) / actual_lookahead

        # Feed forward the path curvature at the target corridor.  Pure
        # Pursuit alone reacts after the vehicle has entered a tight bend;
        # blending this term starts the turn earlier without removing the
        # lateral-error feedback that recentres the vehicle on the path.
        feedforward_s = min(
            self.total_length_m,
            progress_s + 0.75 * lookahead,
        )
        path_curvature = profile_value_at_s(
            self.s_values, curvatures, feedforward_s
        )
        feedback_steering = math.atan(self.wheelbase_m * pp_curvature)
        # Preserve the full lateral/heading feedback. Averaging a curvature
        # feedforward with PP scales all corrective steering by (1-weight),
        # which weakens recovery on the outside of a corner. Instead add only
        # the feedforward correction to the PP command for an on-path vehicle.
        reference, reference_index = interpolate_by_s(self.points, self.s_values, progress_s)
        reference_next = self.points[reference_index + 1]
        reference_previous = self.points[reference_index]
        reference_yaw = math.atan2(reference_next.y - reference_previous.y,
                                   reference_next.x - reference_previous.x)
        reference_dx, reference_dy = target.x - reference.x, target.y - reference.y
        reference_distance_sq = max(reference_dx ** 2 + reference_dy ** 2, 1e-6)
        reference_y = (-math.sin(reference_yaw) * reference_dx
                       + math.cos(reference_yaw) * reference_dy)
        reference_curvature = 2.0 * reference_y / reference_distance_sq
        # At an S transition, do not inject the next bend's opposite steering
        # while the current target chord still belongs to the previous bend.
        if reference_curvature * path_curvature < 0.0:
            feedforward_weight = 0.0
        correction = (math.atan(self.wheelbase_m * path_curvature)
                      - math.atan(self.wheelbase_m * reference_curvature))
        steering = (feedback_steering + feedforward_weight * correction) * self.steering_sign
        # Pure Pursuit can cut across the centre line during a left-to-right
        # transition. Use the measured displacement from the route as bounded
        # near-field feedback; positive error is left of the route.
        lateral_gain = getattr(self, "path_lateral_feedback_gain", 0.0)
        lateral_limit = getattr(self, "path_lateral_feedback_max_rad", 0.0)
        if lateral_gain > 0.0 and lateral_limit > 0.0:
            steering -= self.steering_sign * clamp(
                math.atan2(lateral_gain * path_lateral_error_m,
                           max(0.0, speed_mps) + 2.0),
                -lateral_limit, lateral_limit,
            )
        return clamp(steering, -self.max_steering_rad, self.max_steering_rad), target, actual_lookahead

    def limit_steering_rate(self, steering: float, dt: float) -> float:
        """Limit steering slew so a noisy bend cannot create a jerk."""

        target = clamp(float(steering), -self.max_steering_rad, self.max_steering_rad)
        if self.max_steering_rate_rad_s <= 0.0:
            self.last_steering_rad = target
            return target
        max_delta = self.max_steering_rate_rad_s * max(float(dt), 1e-3)
        limited = clamp(
            target,
            self.last_steering_rad - max_delta,
            self.last_steering_rad + max_delta,
        )
        self.last_steering_rad = limited
        return limited

    def lane_centered_steering(self, nominal: float, speed_mps: float,
                              dt: float) -> Tuple[float, float, bool]:
        """Add bounded curve-compensated feedback from stable CAM1 frames."""
        ros_now, wall_now = rospy.Time.now().to_sec(), time.monotonic()
        with self.lane_lock:
            while self.enable_lane_centering and self.lane_observations:
                stamp, received, lateral, heading, confidence, quality = self.lane_observations[0]
                if (not -0.05 <= ros_now - stamp <= self.lane_centering_timeout_sec
                        or not 0 <= wall_now - received <= self.lane_centering_timeout_sec):
                    self.lane_observations.popleft()
                    self.lane_assist.invalidate("stale")
                    continue
                pose_sample = min(self.lane_pose_history,
                                  key=lambda item: abs(item[0] - stamp), default=None)
                if pose_sample is None or abs(pose_sample[0] - stamp) > 0.05:
                    # Defer a short callback-order delay, without counting an
                    # old image again. Prolonged missing pairs fade the assist.
                    if wall_now - received > 0.1:
                        self.lane_assist.invalidate("pose_unsynchronized")
                    break
                self.lane_observations.popleft()
                message = pose_sample[1]
                pose = message.pose.pose
                try:
                    q = pose.orientation
                    values = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w)
                    if (message.header.frame_id != self.map_frame
                            or not all(math.isfinite(v) for v in values)
                            or abs(math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) - 1) > 0.01):
                        raise ValueError("invalid lane pose")
                    # Image-time route progress stays on the current route
                    # branch; a nearby crossing road cannot become our lane.
                    radius = max(5.0, speed_mps * self.lane_centering_timeout_sec + 2.0)
                    center_s = self.last_progress_s if self.last_progress_s is not None else 0.0
                    start = max(0, bisect_left(self.s_values, center_s - radius) - 1)
                    end = min(len(self.points) - 2, bisect_right(self.s_values, center_s + radius))
                    projection = nearest_projection(self.points, self.s_values,
                                                    pose.position.x, pose.position.y, start, end)
                    before, _ = interpolate_by_s(self.points, self.s_values, projection.progress_s - 0.75)
                    after, _ = interpolate_by_s(self.points, self.s_values, projection.progress_s + 0.75)
                    route_yaw = math.atan2(after.y - before.y, after.x - before.x)
                    pose_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
                    yaw_error = math.atan2(math.sin(route_yaw - pose_yaw), math.cos(route_yaw - pose_yaw))
                    if projection.distance_m > 1.5 or abs(yaw_error) > 0.6:
                        raise ValueError("off-route lane pose")
                    reference = ideal_lane_reference(self.points, self.s_values, projection.progress_s)
                    if reference is None:
                        self.lane_assist.invalidate("route_reference_unavailable")
                        continue
                    self.lane_assist.observe(lateral, heading, reference, confidence, quality, stamp, received)
                except (AttributeError, TypeError, ValueError, OverflowError):
                    self.lane_assist.invalidate("route_reference_unavailable")
            status = self.lane_assist.update(speed_mps, ros_now, wall_now, dt,
                                             enabled=self.enable_lane_centering)
        correction = self.steering_sign * status["correction_rad"]
        centered = clamp(nominal + correction, -self.max_steering_rad, self.max_steering_rad)
        status["correction_rad"] = centered - nominal
        status["source_topic"] = self.lane_info_topic
        self.lane_centering_status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
        return centered, centered - nominal, status["active"]

    def stop_lane_assist(self, reason: str) -> None:
        """Do not retain steering evidence across a control/pose interruption."""
        with self.lane_lock:
            self.lane_observations.clear()
            self.lane_assist.invalidate(reason)
            status = self.lane_assist.update(0.0, rospy.Time.now().to_sec(), time.monotonic(),
                                             1.0 / self.rate_hz,
                                             enabled=self.enable_lane_centering, stopped=True)
        status.update(active=False, correction_rad=0.0, reason=reason,
                      source_topic=self.lane_info_topic)
        self.lane_centering_status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
        self.lane_correction_pub.publish(Float64(0.0))
        self.lane_correction_active_pub.publish(Bool(False))

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
            self.stop_lane_assist("pose_unsynchronized")
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
            self.stop_lane_assist("pose_unsynchronized")
            self.command_speed_mps = 0.0
            self.last_steering_rad = 0.0
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
        odom_speed_mps = math.hypot(
            self.latest_odom.twist.twist.linear.x,
            self.latest_odom.twist.twist.linear.y,
        )
        vehicle_speed = self.vehicle_speed_source.sample(now, time.monotonic())
        self.vehicle_speed_fresh_pub.publish(Bool(vehicle_speed is not None))
        self.odom_speed_pub.publish(Float64(odom_speed_mps * MPS_TO_KPH))
        if ((self.require_vehicle_speed and vehicle_speed is None)
                or (vehicle_speed is None and not math.isfinite(odom_speed_mps))):
            self.stop_lane_assist("stopped")
            self.command_speed_mps = 0.0
            self.speed_controller.reset()
            self.speed_command_pub.publish(Float64(0.0))
            self.accel_command_pub.publish(Float64(0.0))
            self.brake_command_pub.publish(Float64(1.0))
            if self.publish_command:
                self.command_pub.publish(self.make_command(
                    self.last_steering_rad, 0.0, 0.0, stop=True, dt=dt))
            rospy.logwarn_throttle(2.0, "Curvature PP stopped: fresh vehicle speed unavailable (%s)",
                                   self.vehicle_speed_topic)
            return
        measured_speed_mps = vehicle_speed if vehicle_speed is not None else odom_speed_mps
        measured_speed_kph = measured_speed_mps * MPS_TO_KPH
        self.measured_speed_pub.publish(Float64(measured_speed_kph))
        if (vehicle_speed is not None and math.isfinite(odom_speed_mps)
                and abs(vehicle_speed - odom_speed_mps) > 2.0):
            rospy.logwarn_throttle(2.0, "Curvature PP speed disagreement: vehicle=%.2f EKF=%.2f km/h; using vehicle speed",
                                   measured_speed_kph, odom_speed_mps * MPS_TO_KPH)

        projection = self.search_projection(x, y)
        path_start = self.points[projection.segment_index]
        path_end = self.points[projection.segment_index + 1]
        path_dx = path_end.x - path_start.x
        path_dy = path_end.y - path_start.y
        path_length = max(math.hypot(path_dx, path_dy), 1e-9)
        path_lateral_error = (
            path_dx * (y - path_start.y) - path_dy * (x - path_start.x)
        ) / path_length
        self.last_segment_index = projection.segment_index
        progress_s = projection.progress_s
        if self.last_progress_s is not None:
            progress_s = max(progress_s, self.last_progress_s)
        progress_s = min(progress_s, self.total_length_m)
        self.last_progress_s = progress_s

        remaining_m = max(0.0, self.total_length_m - progress_s)
        stop = remaining_m <= self.goal_tolerance_m
        speed_limit = minimum_speed_ahead(
            self.s_values, self.speed_profile, progress_s,
            measured_speed_mps * self.speed_preview_time_sec)
        stopline_cap_active = self.update_stopline_speed_cap(
            progress_s, self.latest_odom.header.stamp.to_sec(), now
        )
        if stopline_cap_active:
            speed_limit = min(speed_limit, self.stopline_approach_speed_kph / MPS_TO_KPH)
        command_speed = 0.0 if stop else self.apply_speed_rate_limit(speed_limit, dt)
        curvature = profile_value_at_s(self.s_values, self.curvatures, progress_s)

        if stop:
            self.stop_lane_assist("stopped")
            steering = 0.0
            self.last_steering_rad = 0.0
            target = self.points[-1]
            actual_lookahead = 0.0
            lane_correction = 0.0
            lane_correction_active = False
        else:
            steering, target, actual_lookahead = self.compute_steering(
                x, y, yaw, measured_speed_mps, progress_s, path_lateral_error
            )
            steering, lane_correction, lane_correction_active = self.lane_centered_steering(
                steering, measured_speed_mps, dt
            )
            steering = self.limit_steering_rate(steering, dt)

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
        self.path_lateral_error_pub.publish(Float64(path_lateral_error))
        self.lane_correction_pub.publish(Float64(lane_correction))
        self.lane_correction_active_pub.publish(Bool(lane_correction_active))
        self.progress_pub.publish(Float64(progress_s))
        self.goal_pub.publish(Bool(stop))
        self.stopline_cap_active_pub.publish(Bool(stopline_cap_active))
        self.stopline_cap_target_pub.publish(Float64(
            self.stopline_approach_speed_kph if stopline_cap_active else 0.0
        ))

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
            "lookahead=%.2f steering=%.4f path_lateral_error=%.2f "
            "lane_correction=%.4f lane_active=%s stop=%s stopline_cap=%s",
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
            path_lateral_error,
            lane_correction,
            lane_correction_active,
            stop,
            stopline_cap_active,
        )


if __name__ == "__main__":
    try:
        CurvatureSpeedPurePursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
