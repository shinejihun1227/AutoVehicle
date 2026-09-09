#!/usr/bin/env python3
"""GPS/IMU 품질에 따라 Pure Pursuit와 전방 차선 제어를 안전하게 전환한다.

정상 상태에서는 곡률 기반 Pure Pursuit 명령을 그대로 통과시킨다.
GPS blackout이면 검증된 차선 기반 조향을 사용한다. IMU까지 degraded이거나
센서 품질을 확인할 수 없으면 경로/차선 조향을 섞어 계속 달리지 않고 정지한다.

차선이 불량하거나 속도/IMU가 불확실하면 정지한다. 정상 위치에서 시작한
음영구간만 시간·거리 예산 내에서 허용하며, 복구 후 실제 안정화와 조향 전환을 한다.
레거시 stop_without_camera/fallback_nominal_weight 옵션은 안전 조건을 해제하지 않는다.

이 노드는 /Ego_topic에 의존하지 않는다. accel/brake(type 1) 경로에서는
nominal 명령을 속도 기준으로 유지하고, EKF odometry로 fallback 속도 상한을
감시한다. 카메라만으로 종방향 속도를 추정하지는 않는다.
"""

from __future__ import annotations

import copy
import json
import math
import time
import threading
from collections import deque
from typing import Deque, Optional, Tuple

import rospy
from morai_msgs.msg import CtrlCmd
from morai_perception_msgs.msg import LaneDetection, SensorQuality
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


NORMAL = "NORMAL"
GPS_BLACKOUT = "GPS_BLACKOUT"
SENSOR_DEGRADED = "SENSOR_DEGRADED"
MPS_TO_KPH = 3.6


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def finite(value: float) -> bool:
    return math.isfinite(float(value))


def median(values) -> float:
    samples = sorted(float(value) for value in values if finite(value))
    if not samples:
        return math.nan
    middle = len(samples) // 2
    if len(samples) % 2:
        return samples[middle]
    return 0.5 * (samples[middle - 1] + samples[middle])


class CameraLocalizationFallbackController:
    def __init__(self) -> None:
        rospy.init_node("camera_localization_fallback_controller", anonymous=False)
        self.lock = threading.RLock()

        self.nominal_topic = rospy.get_param(
            "~nominal_command_topic", "/control/curvature_ctrl_cmd"
        )
        self.output_topic = rospy.get_param(
            "~output_command_topic", "/control/camera_fallback_cmd"
        )
        self.lane_topic = rospy.get_param("~lane_topic", "/detection/lane")
        self.quality_topic = rospy.get_param(
            "~quality_topic", "/localization/sensor_quality"
        )
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/localization/odometry"
        )
        self.intersection_topic = rospy.get_param(
            "~intersection_topic", "/perception/intersection/detected"
        )

        self.rate_hz = max(1.0, float(rospy.get_param("~rate_hz", 20.0)))
        self.nominal_timeout = max(
            0.05, float(rospy.get_param("~nominal_timeout_sec", 0.5))
        )
        self.lane_timeout = max(
            0.05, float(rospy.get_param("~lane_timeout_sec", 0.3))
        )
        self.quality_timeout = max(
            0.05, float(rospy.get_param("~quality_timeout_sec", 0.5))
        )
        self.odom_timeout = max(
            0.05, float(rospy.get_param("~odom_timeout_sec", 0.5))
        )
        self.intersection_timeout = max(
            0.05, float(rospy.get_param("~intersection_timeout_sec", 0.5))
        )
        self.min_lane_confidence = clamp(
            float(rospy.get_param("~min_lane_confidence", 0.55)), 0.0, 1.0
        )
        self.lane_fallback_confidence = clamp(
            float(rospy.get_param("~lane_fallback_confidence", 0.80)),
            self.min_lane_confidence,
            1.0,
        )
        self.lane_stable_samples = max(
            1, int(rospy.get_param("~lane_stable_samples", 5))
        )
        self.lane_stable_sec = float(rospy.get_param("~lane_stable_sec", 0.20))
        self.lane_control_timeout = float(rospy.get_param("~lane_control_timeout_sec", 0.15))
        if (not finite(self.lane_stable_sec) or self.lane_stable_sec <= 0
                or not finite(self.lane_control_timeout)
                or not 0 < self.lane_control_timeout <= self.lane_timeout):
            raise ValueError("Lane confirmation must be positive; control age must be within lane timeout")
        self.lane_loss_grace_sec = max(
            0.0, float(rospy.get_param("~lane_loss_grace_sec", 0.25))
        )
        self.fallback_entry_delay_sec = max(
            0.0, float(rospy.get_param("~fallback_entry_delay_sec", 0.50))
        )
        self.recovery_stable_sec = max(
            0.0, float(rospy.get_param("~recovery_stable_sec", 1.0))
        )
        self.lane_filter_window = max(
            1, int(rospy.get_param("~lane_filter_window", 5))
        )
        # Distances must match the perception producer's error reference.
        self.preview_distance_m = float(rospy.get_param("~lane_preview_distance_m", 7.0))
        self.heading_preview_distance_m = float(rospy.get_param("~lane_heading_distance_m", 14.0))
        self.wheelbase_m = float(rospy.get_param("~wheelbase_m", 3.0))
        if (not all(finite(v) and v > 0 for v in (self.preview_distance_m,
                    self.heading_preview_distance_m, self.wheelbase_m))
                or self.heading_preview_distance_m <= self.preview_distance_m):
            raise ValueError("Lane reference distances and wheelbase must be finite, positive and ordered")
        self.lateral_gain = float(rospy.get_param("~lateral_gain", 0.8))
        self.heading_gain = float(rospy.get_param("~heading_gain", 0.7))
        if not all(finite(v) and v > 0 for v in (self.lateral_gain, self.heading_gain)):
            raise ValueError("Lane correction gains must be positive and finite")
        self.lane_sign = 1.0 if float(rospy.get_param("~lane_sign", 1.0)) >= 0.0 else -1.0
        self.max_steering_rad = max(
            0.05,
            float(rospy.get_param("~max_steering_rad", math.radians(40.0))),
        )
        self.max_steering_rate = max(
            0.0,
            float(rospy.get_param("~max_steering_rate_rad_s", 0.8)),
        )
        self.camera_assist_gain = max(
            0.0, float(rospy.get_param("~camera_assist_gain", 0.65))
        )
        self.fallback_nominal_weight = clamp(
            float(rospy.get_param("~fallback_nominal_weight", 0.0)), 0.0, 1.0
        )
        legacy_speed_cap_mps = max(
            0.0, float(rospy.get_param("~fallback_speed_cap_mps", 2.0))
        )
        speed_cap_kph = rospy.get_param("~fallback_speed_cap_kph", None)
        self.fallback_speed_cap_kph = max(
            0.0,
            float(speed_cap_kph)
            if speed_cap_kph is not None
            else legacy_speed_cap_mps * MPS_TO_KPH,
        )
        self.fallback_speed_cap_mps = self.fallback_speed_cap_kph / MPS_TO_KPH
        self.stop_without_camera = bool(
            rospy.get_param("~stop_without_camera", True)
        )
        if not self.stop_without_camera or self.fallback_nominal_weight:
            rospy.logwarn("Legacy nominal blending/no-camera driving options are ignored during GPS loss")
        self.longl_cmd_type = int(rospy.get_param("~longl_cmd_type", 1))
        if self.longl_cmd_type != 1 or not finite(self.max_steering_rate) or self.max_steering_rate <= 0:
            raise ValueError("Fallback requires type-1 pedals and a positive finite steering rate limit")
        self.fallback_speed_margin_mps = max(
            0.0, float(rospy.get_param("~fallback_speed_margin_mps", 0.20))
        )
        self.max_decel_mps2 = max(
            0.1, float(rospy.get_param("~max_decel_mps2", 1.5))
        )
        self.fallback_brake_gain = max(
            0.0, float(rospy.get_param("~fallback_brake_gain", 1.0))
        )

        self.last_nominal: Optional[CtrlCmd] = None
        self.last_quality: Optional[SensorQuality] = None
        self.last_lane: Optional[LaneDetection] = None
        self.last_odom: Optional[Odometry] = None
        self.intersection_detected = False
        self.last_intersection_time = 0.0
        self.last_nominal_time = 0.0
        self.last_quality_time = 0.0
        self.last_lane_time = 0.0
        self.last_odom_time = 0.0
        self.last_output_time = time.monotonic()
        self.last_output_steering = 0.0
        self.last_output_velocity = 0.0
        self.last_lane_good_time = 0.0
        self.lane_good_streak = 0
        self.quality_state_since = time.monotonic()
        self.last_quality_state = None
        self.last_mode = "initializing"
        self.recovery_since = None

        self.lateral_history: Deque[float] = deque(maxlen=self.lane_filter_window)
        self.heading_history: Deque[float] = deque(maxlen=self.lane_filter_window)
        self.lane_history_times = deque(maxlen=self.lane_filter_window)
        self.lane_good_since = None

        # Loss/recovery budgets are not reset by a single NORMAL frame.
        self.blackout_max_duration_sec = float(rospy.get_param("~blackout_max_duration_sec", 15.0))
        self.blackout_max_distance_m = float(rospy.get_param("~blackout_max_distance_m", 30.0))
        self.max_lateral_error_m = float(rospy.get_param("~max_lateral_error_m", 2.0))
        self.max_heading_error_rad = float(rospy.get_param("~max_heading_error_rad", 0.6))
        if not all(finite(v) and v > 0 for v in (self.blackout_max_duration_sec,
                   self.blackout_max_distance_m, self.max_lateral_error_m, self.max_heading_error_rad)):
            raise ValueError("Fallback budgets and geometry bounds must be positive and finite")
        self.require_fresh_intersection = bool(rospy.get_param("~require_fresh_intersection", True))
        self.source_stamps = {}
        self.epoch_ros = rospy.get_time()
        self.previous_tick = None
        self.control_dt = 0.0
        self.last_output_ros_time = self.epoch_ros
        self.last_accel = 0.0
        self.had_normal = False
        self.recovering = False
        self.untrusted_since = None
        self.untrusted_distance = 0.0
        self.previous_speed = None
        self.budget_exhausted = False
        self.quality_state_ros_since = self.epoch_ros
        self.last_primary_stamp = None
        self.primary_live = False

        self.output_pub = rospy.Publisher(self.output_topic, CtrlCmd, queue_size=1)
        self.status_pub = rospy.Publisher(
            "/stability/camera_fallback_status", String, queue_size=1, latch=True
        )
        rospy.Subscriber(
            self.nominal_topic, CtrlCmd, self.nominal_callback, queue_size=10
        )
        rospy.Subscriber(self.lane_topic, LaneDetection, self.lane_callback, queue_size=10)
        rospy.Subscriber(
            self.quality_topic, SensorQuality, self.quality_callback, queue_size=10
        )
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber(
            self.intersection_topic,
            Bool,
            self.intersection_callback,
            queue_size=1,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.rate_hz), self.publish_command
        )

        rospy.loginfo(
            "Camera fallback nominal=%s lane=%s quality=%s output=%s",
            self.nominal_topic,
            self.lane_topic,
            self.quality_topic,
            self.output_topic,
        )

    def reset_epoch(self):
        self.source_stamps.clear()
        self.last_lane = self.last_quality = self.last_odom = self.last_nominal = None
        self.lateral_history.clear()
        self.heading_history.clear()
        self.lane_history_times.clear()
        self.lane_good_since = None
        self.lane_good_streak = 0
        self.last_primary_stamp = None
        self.primary_live = self.had_normal = self.recovering = False
        self.untrusted_since = self.recovery_since = self.previous_tick = None
        self.untrusted_distance = 0.0
        self.previous_speed = None
        self.budget_exhausted = False
        self.last_quality_state = None
        self.last_intersection_time = 0.0
        self.last_accel = 0.0

    def check_epoch(self):
        ros_now = rospy.get_time()
        if ros_now < self.epoch_ros:
            self.reset_epoch()
        self.epoch_ros = ros_now
        return ros_now

    def accept_stamp(self, message, key, timeout):
        ros_now = self.check_epoch()
        try:
            stamp = float(message.header.stamp.to_sec())
        except (AttributeError, TypeError, ValueError):
            return False
        if (not finite(stamp) or stamp <= 0 or not -0.05 <= ros_now - stamp <= timeout
                or stamp <= self.source_stamps.get(key, 0.0)):
            return False
        self.source_stamps[key] = stamp
        return True

    def fresh(self, key, now, timeout):
        received = getattr(self, "last_" + key + "_time")
        stamp = self.source_stamps.get(key)
        return bool(stamp is not None and 0 <= now - received <= timeout
                    and -0.05 <= rospy.get_time() - stamp <= timeout)

    def nominal_callback(self, message):
        with self.lock:
            self.check_epoch()
            self.last_nominal = copy.deepcopy(message)
            self.last_nominal_time = time.monotonic()

    def quality_callback(self, message):
        with self.lock:
            self.check_epoch()
            interrupted = self.last_quality is not None and not self.fresh(
                "quality", time.monotonic(), self.quality_timeout)
            if self.accept_stamp(message, "quality", self.quality_timeout):
                self.last_quality = copy.deepcopy(message)
                self.last_quality_time = time.monotonic()
                if interrupted:
                    self.last_quality_state = None
                state, _ = self.quality_state(self.last_quality_time)
                # A bad frame between two control ticks must restart recovery,
                # even if a NORMAL frame overwrites it before the next tick.
                if interrupted or state != NORMAL:
                    self.mark_untrusted(self.last_quality_time, rospy.get_time())

    def mark_untrusted(self, now, ros_now):
        self.recovering = True
        self.recovery_since = None
        if self.untrusted_since is None:
            self.untrusted_since = (now, ros_now)

    def reset_lane_confirmation(self, discard_hold=False):
        self.lateral_history.clear()
        self.heading_history.clear()
        self.lane_history_times.clear()
        self.lane_good_streak = 0
        self.lane_good_since = None
        self.primary_live = False
        if discard_hold:
            self.last_primary_stamp = None

    def prune_lane_history(self, now):
        ros_now = rospy.get_time()
        while self.lane_history_times:
            received, stamp = self.lane_history_times[0]
            if 0 <= now - received <= self.lane_timeout and -0.05 <= ros_now - stamp <= self.lane_timeout:
                break
            self.lane_history_times.popleft()
            self.lateral_history.popleft()
            self.heading_history.popleft()

    def lane_callback(self, message):
        with self.lock:
            self.check_epoch()
            previous_stamp = self.source_stamps.get("lane")
            if not self.accept_stamp(message, "lane", self.lane_timeout):
                # An identical replay is ignored; conflicting payloads with the
                # same observation time revoke the cached lane immediately.
                try:
                    duplicate = message.header.stamp.to_sec() == previous_stamp
                except (AttributeError, TypeError, ValueError):
                    duplicate = False
                fields = ("valid", "confidence", "lateral_offset_m", "heading_error_rad")
                if duplicate and self.last_lane is not None and (
                        message.header.frame_id != self.last_lane.header.frame_id
                        or any(getattr(message, k) != getattr(self.last_lane, k) for k in fields)):
                    self.last_lane = None
                    self.reset_lane_confirmation(discard_hold=True)
                return
            now = time.monotonic()
            gap = (previous_stamp is None
                   or self.source_stamps["lane"] - previous_stamp > self.lane_control_timeout
                   or now - self.last_lane_time > self.lane_control_timeout)
            if gap:
                self.reset_lane_confirmation()
            self.prune_lane_history(now)
            self.last_lane, self.last_lane_time = copy.deepcopy(message), now
            self.primary_live = False
            lateral, heading, confidence = (float(message.lateral_offset_m),
                                           float(message.heading_error_rad), float(message.confidence))
            geometry_ok = (message.header.frame_id in ("front_camera", "base_link")
                           and all(finite(v) for v in (lateral, heading, confidence))
                           and 0 <= confidence <= 1
                           and abs(lateral) <= self.max_lateral_error_m
                           and abs(heading) <= self.max_heading_error_rad)
            jump = (self.lateral_history and
                    (abs(lateral - self.lateral_history[-1]) > .75
                     or abs(heading - self.heading_history[-1]) > .30))
            if not geometry_ok or jump:
                self.last_lane = None
                self.reset_lane_confirmation(discard_hold=True)
                return
            if message.valid and confidence >= self.min_lane_confidence:
                self.lateral_history.append(lateral)
                self.heading_history.append(heading)
                self.lane_history_times.append((now, self.source_stamps["lane"]))
            if message.valid and confidence >= self.lane_fallback_confidence:
                if self.lane_good_since is None:
                    self.lane_good_since = (now, self.source_stamps["lane"])
                self.lane_good_streak += 1
                self.primary_live = True
                if self.lane_is_primary_usable(now):
                    self.last_lane_good_time = now
                    self.last_primary_stamp = self.source_stamps["lane"]
            else:
                self.lane_good_streak = 0
                self.lane_good_since = None

    def odom_callback(self, message):
        with self.lock:
            if not self.accept_stamp(message, "odom", self.odom_timeout):
                return
            velocity = message.twist.twist.linear
            self.last_odom = (copy.deepcopy(message)
                              if message.header.frame_id == "map"
                              and all(finite(v) for v in (velocity.x, velocity.y)) else None)
            self.last_odom_time = time.monotonic()

    def intersection_callback(self, message):
        with self.lock:
            self.check_epoch()
            self.intersection_detected = bool(message.data)
            self.last_intersection_time = time.monotonic()

    def stop_command(self):
        command = CtrlCmd()
        command.longlCmdType = 1
        # Keep the last steering while braking; do not snap wheels to centre.
        command.steering = self.last_output_steering
        command.velocity = command.acceleration = command.accel = 0.0
        command.brake = 1.0
        return command

    def lane_is_usable(self, now):
        return bool(self.last_lane is not None and self.fresh("lane", now, self.lane_timeout)
                    and self.last_lane.valid and finite(self.last_lane.confidence)
                    and self.min_lane_confidence <= self.last_lane.confidence <= 1.
                    and self.lateral_history and self.heading_history)

    def lane_is_primary_usable(self, now):
        return bool(self.lane_is_usable(now) and self.primary_live
                    and self.fresh("lane", now, self.lane_control_timeout)
                    and self.lane_good_streak >= self.lane_stable_samples
                    and self.lane_good_since is not None
                    and min(now - self.lane_good_since[0],
                            self.source_stamps["lane"] - self.lane_good_since[1])
                        >= self.lane_stable_sec - 1e-9)

    def lane_is_hold_usable(self, now):
        return bool(self.last_primary_stamp is not None
                    and 0 <= now - self.last_lane_good_time <= self.lane_loss_grace_sec
                    and 0 <= rospy.get_time() - self.last_primary_stamp <= self.lane_loss_grace_sec)

    def intersection_is_active(self, now):
        # A previously detected intersection cannot vanish just by timing out.
        stale = not 0 <= now - self.last_intersection_time <= self.intersection_timeout
        return self.intersection_detected or (self.require_fresh_intersection and stale)

    def lane_steering(self):
        self.prune_lane_history(time.monotonic())
        lateral, heading = median(self.lateral_history), median(self.heading_history)
        if not all(finite(v) for v in (lateral, heading)):
            return 0.0
        # BEV x is forward, y is left. Recover both measured centre points
        # from -centre_y and the near-to-far chord heading. Directly adding
        # that heading to steering oversteers on curves: it contains preview
        # curvature as well as the vehicle's heading error.
        near_x, far_x = self.preview_distance_m, self.heading_preview_distance_m
        near_y = -lateral
        far_y = near_y + (far_x - near_x) * math.tan(heading)
        near_curvature = 2. * near_y / (near_x * near_x + near_y * near_y)
        far_curvature = 2. * far_y / (far_x * far_x + far_y * far_y)
        # Legacy gain parameters now weight the two geometric observations.
        curvature = (self.lateral_gain * near_curvature + self.heading_gain * far_curvature) / (
            self.lateral_gain + self.heading_gain)
        return self.lane_sign * math.atan(self.wheelbase_m * curvature)

    @staticmethod
    def command_velocity(command):
        if command is None:
            return None
        value = float(command.velocity)
        return max(0., value) if finite(value) else None

    def measured_speed_mps(self):
        if self.last_odom is None or not self.fresh("odom", time.monotonic(), self.odom_timeout):
            return None
        v = self.last_odom.twist.twist.linear
        if not all(finite(x) for x in (v.x, v.y)):
            return None
        speed = math.hypot(v.x, v.y)
        return speed if finite(speed) else None

    def fallback_velocity(self):
        measured = self.measured_speed_mps()
        return min(measured, self.fallback_speed_cap_mps) if measured is not None else 0.0

    def apply_fallback_speed_cap(self, command):
        measured = self.measured_speed_mps()
        values = (command.accel, command.brake, command.steering)
        if (measured is None or command.longlCmdType != 1 or not all(finite(v) for v in values)
                or self.fallback_speed_cap_mps <= 0):
            command.accel, command.brake = 0.0, 1.0
            return
        command.accel, command.brake = clamp(command.accel, 0., 1.), clamp(command.brake, 0., 1.)
        if measured >= self.fallback_speed_cap_mps:
            command.accel = 0.0
            excess = measured - self.fallback_speed_cap_mps
            if excess > self.fallback_speed_margin_mps:
                command.brake = max(command.brake, clamp(
                    self.fallback_brake_gain * excess / self.max_decel_mps2, 0., 1.))
        if command.brake > 0:
            command.accel = 0.
        command.velocity = command.acceleration = 0.0

    def apply_rate_limit(self, steering, now):
        steering = clamp(steering, -self.max_steering_rad, self.max_steering_rad)
        dt = max(0., min(.1, now - self.last_output_time,
                         rospy.get_time() - self.last_output_ros_time))
        max_delta = self.max_steering_rate * dt
        return clamp(steering, self.last_output_steering - max_delta,
                     self.last_output_steering + max_delta)

    def quality_state(self, now):
        if self.last_quality is None or not self.fresh("quality", now, self.quality_timeout):
            if self.last_quality_state != SENSOR_DEGRADED:
                self.last_quality_state = SENSOR_DEGRADED
                self.quality_state_since = now
                self.quality_state_ros_since = rospy.get_time()
            return SENSOR_DEGRADED, "sensor_quality_stale"
        q = self.last_quality
        state = str(q.state)
        reason = str(q.reason)
        if (state not in (NORMAL, GPS_BLACKOUT, SENSOR_DEGRADED)
                or not finite(q.confidence) or not 0 <= q.confidence <= 1.):
            state, reason = SENSOR_DEGRADED, "invalid_quality_state"
        elif q.imu_stale:
            state, reason = SENSOR_DEGRADED, "imu_stale"
        elif state == NORMAL and (not q.gps_valid or q.gps_blackout or q.gps_recovering):
            state, reason = SENSOR_DEGRADED, "inconsistent_quality"
        elif state == GPS_BLACKOUT and not q.gps_blackout:
            state, reason = SENSOR_DEGRADED, "inconsistent_quality"
        if state != self.last_quality_state:
            self.last_quality_state = state
            self.quality_state_since = now
            self.quality_state_ros_since = rospy.get_time()
        return state, reason

    def emit(self, output, mode, state, reason, camera_used=False, nominal_fresh=False):
        output.longlCmdType = 1
        output.velocity = output.acceleration = 0.
        output.brake = clamp(float(output.brake), 0., 1.)
        output.accel = (0. if output.brake > 0 else
                        min(clamp(float(output.accel), 0., 1.), self.last_accel + .5 * self.control_dt))
        self.last_accel = output.accel
        self.last_output_steering = float(output.steering)
        self.last_output_time, self.last_output_ros_time = time.monotonic(), rospy.get_time()
        self.last_output_velocity = self.fallback_velocity()
        self.last_mode = mode
        self.output_pub.publish(output)
        confidence = float(self.last_lane.confidence) if self.last_lane else 0.
        self.status_pub.publish(String(data=json.dumps({
            "stamp": rospy.get_time(), "mode": mode, "active_mode": mode,
            "quality_state": state, "quality_reason": reason, "camera_used": camera_used,
            "lane_usable": self.lane_is_usable(time.monotonic()),
            "lane_primary_usable": self.lane_is_primary_usable(time.monotonic()),
            "lane_confidence": confidence if finite(confidence) else 0.,
            "lane_good_streak": self.lane_good_streak, "nominal_fresh": nominal_fresh,
            "lane_source_stamp": self.source_stamps.get("lane"),
            "lane_source_age_sec": (max(0., rospy.get_time() - self.source_stamps["lane"])
                                    if "lane" in self.source_stamps else None),
            "lane_control_timeout_sec": self.lane_control_timeout,
            "output_steering_rad": self.last_output_steering,
            "accel": output.accel, "brake": output.brake,
            "output_velocity_mps": self.last_output_velocity,
            "fallback_speed_cap_kph": self.fallback_speed_cap_kph,
            "blackout_distance_m": self.untrusted_distance,
            "blackout_budget_exhausted": self.budget_exhausted,
        }, ensure_ascii=False, allow_nan=False)))

    def publish_command(self, _event):
        with self.lock:
            now, ros_now = time.monotonic(), self.check_epoch()
            previous = self.previous_tick
            ros_dt = 0. if previous is None else ros_now - previous[1]
            wall_dt = 0. if previous is None else now - previous[0]
            gap = previous is not None and (not 0 <= ros_dt <= .5 or not 0 <= wall_dt <= .5)
            self.previous_tick = (now, ros_now)
            self.control_dt = max(0., min(.1, ros_dt, wall_dt))
            state, reason = self.quality_state(now)
            nominal = self.last_nominal
            nominal_fresh = bool(nominal is not None and 0 <= now - self.last_nominal_time <= self.nominal_timeout
                                 and nominal.longlCmdType == 1
                                 and all(finite(getattr(nominal, k)) for k in ("accel", "brake", "steering")))
            speed = self.measured_speed_mps()
            if self.primary_live and not self.fresh("lane", now, self.lane_control_timeout):
                self.reset_lane_confirmation()
            primary = self.lane_is_primary_usable(now)

            if state != NORMAL:
                self.mark_untrusted(now, ros_now)
            if self.untrusted_since is not None:
                if speed is not None:
                    # Conservative endpoint integration includes braking travel.
                    self.untrusted_distance += max(speed, self.previous_speed or 0.) * max(0., ros_dt)
                elapsed = max(now - self.untrusted_since[0], ros_now - self.untrusted_since[1])
                self.budget_exhausted |= (gap or speed is None or elapsed > self.blackout_max_duration_sec
                                         or self.untrusted_distance > self.blackout_max_distance_m)
            self.previous_speed = speed

            def stop(why, mode="fallback_safe_stop"):
                self.emit(self.stop_command(), mode, state, why, nominal_fresh=nominal_fresh)

            if not nominal_fresh or speed is None or gap:
                self.recovery_since = None
                stop("nominal_or_speed_unavailable" if not gap else "control_clock_gap")
                return
            if state == SENSOR_DEGRADED:
                # IMU loss/unknown health is not a licence to blend an unsafe PP.
                stop(reason)
                return
            if state == NORMAL and not self.recovering:
                self.had_normal = True
                self.emit(copy.deepcopy(nominal), "normal_nominal", state, reason, nominal_fresh=True)
                return

            if state == NORMAL:
                if self.recovery_since is None:
                    self.recovery_since = (now, ros_now)
                stable = min(now - self.recovery_since[0], ros_now - self.recovery_since[1]) >= self.recovery_stable_sec
                if stable:
                    output = copy.deepcopy(nominal)
                    output.steering = self.apply_rate_limit(output.steering, now)
                    self.apply_fallback_speed_cap(output)
                    aligned = abs(output.steering - clamp(nominal.steering, -self.max_steering_rad,
                                                         self.max_steering_rad)) < 1e-6
                    if aligned:
                        self.recovering = False
                        self.had_normal = True
                        self.untrusted_since = None
                        self.untrusted_distance = 0.
                        self.budget_exhausted = False
                    self.emit(output, "normal_nominal" if aligned else "recovery_blend",
                              state, reason, nominal_fresh=True)
                    return
            elif min(now - self.quality_state_since, ros_now - self.quality_state_ros_since) < self.fallback_entry_delay_sec:
                stop("blackout_entry_confirmation")
                return

            if not self.had_normal or self.budget_exhausted:
                stop("blackout_budget_exhausted" if self.budget_exhausted else "no_normal_reference")
                return
            if self.intersection_is_active(now):
                stop("intersection_detected_or_stale")
                return
            hold = self.last_mode in ("gps_blackout_camera_fallback", "lane_loss_braking", "recovery_camera") and self.lane_is_hold_usable(now)
            if not primary and not hold:
                stop("camera_lane_not_primary_usable")
                return
            output = copy.deepcopy(nominal)
            # Blackout steering has no contribution from the drifting map pose.
            output.steering = self.apply_rate_limit(self.lane_steering(), now) if primary else self.last_output_steering
            self.apply_fallback_speed_cap(output)
            if not primary:
                output.accel = 0.
                output.brake = max(output.brake, .2)
            mode = ("lane_loss_braking" if not primary else
                    "recovery_camera" if state == NORMAL else "gps_blackout_camera_fallback")
            self.emit(output, mode, state, reason, camera_used=True, nominal_fresh=True)


if __name__ == "__main__":
    try:
        CameraLocalizationFallbackController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
