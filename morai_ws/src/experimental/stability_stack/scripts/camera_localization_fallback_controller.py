#!/usr/bin/env python3
"""GPS/IMU 품질에 따라 Pure Pursuit와 전방 차선 제어를 안전하게 전환한다.

정상 상태에서는 곡률 기반 Pure Pursuit 명령을 그대로 통과시킨다.
GPS jump/noise 또는 IMU 급변이 감지되면 차선 결과를 이용해 Pure Pursuit를
보조하고, GPS blackout이면 차선 기반 조향을 주 제어로 사용한다.

카메라가 stale이거나 confidence가 부족한 경우에는 카메라가 없는 상태에서
무리하게 주행하지 않고 기본적으로 정지 명령을 만든다. 이 정책은
stop_without_camera=false로 바꿀 수 있지만 실제 차량 제어 전에는 권장하지 않는다.

이 노드는 /Ego_topic에 의존하지 않는다. blackout 시에도 속도는 nominal CtrlCmd가
유효하면 그 값을 유지하고, nominal이 끊긴 경우에는 EKF odometry의 속도 또는
마지막 명령을 사용한다. 카메라만으로 정밀한 종방향 속도를 추정하지는 않는다.
"""

from __future__ import annotations

import copy
import json
import math
import time
from collections import deque
from typing import Deque, Optional, Tuple

import rospy
from morai_msgs.msg import CtrlCmd
from morai_perception_msgs.msg import LaneDetection, SensorQuality
from nav_msgs.msg import Odometry
from std_msgs.msg import String


NORMAL = "NORMAL"
GPS_BLACKOUT = "GPS_BLACKOUT"
GPS_NOISE = "GPS_NOISE"
IMU_NOISE = "IMU_NOISE"
SENSOR_DEGRADED = "SENSOR_DEGRADED"


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
        self.min_lane_confidence = clamp(
            float(rospy.get_param("~min_lane_confidence", 0.65)), 0.0, 1.0
        )
        self.lane_filter_window = max(
            1, int(rospy.get_param("~lane_filter_window", 5))
        )
        self.preview_distance_m = max(
            0.1, float(rospy.get_param("~lane_preview_distance_m", 4.0))
        )
        self.lateral_gain = float(rospy.get_param("~lateral_gain", 0.8))
        self.heading_gain = float(rospy.get_param("~heading_gain", 0.7))
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
            float(rospy.get_param("~fallback_nominal_weight", 0.25)), 0.0, 1.0
        )
        self.fallback_speed_cap_mps = max(
            0.0, float(rospy.get_param("~fallback_speed_cap_mps", 2.0))
        )
        self.stop_without_camera = bool(
            rospy.get_param("~stop_without_camera", True)
        )

        self.last_nominal: Optional[CtrlCmd] = None
        self.last_quality: Optional[SensorQuality] = None
        self.last_lane: Optional[LaneDetection] = None
        self.last_odom: Optional[Odometry] = None
        self.last_nominal_time = 0.0
        self.last_quality_time = 0.0
        self.last_lane_time = 0.0
        self.last_odom_time = 0.0
        self.last_output_time = time.monotonic()
        self.last_output_steering = 0.0
        self.last_output_velocity = 0.0

        self.lateral_history: Deque[float] = deque(maxlen=self.lane_filter_window)
        self.heading_history: Deque[float] = deque(maxlen=self.lane_filter_window)

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

    def nominal_callback(self, message: CtrlCmd) -> None:
        self.last_nominal = copy.deepcopy(message)
        self.last_nominal_time = time.monotonic()

    def quality_callback(self, message: SensorQuality) -> None:
        self.last_quality = message
        self.last_quality_time = time.monotonic()

    def lane_callback(self, message: LaneDetection) -> None:
        self.last_lane = message
        self.last_lane_time = time.monotonic()
        if message.valid and message.confidence >= self.min_lane_confidence:
            lateral = float(message.lateral_offset_m)
            heading = float(message.heading_error_rad)
            if finite(lateral) and finite(heading):
                self.lateral_history.append(lateral)
                self.heading_history.append(heading)

    def odom_callback(self, message: Odometry) -> None:
        self.last_odom = message
        self.last_odom_time = time.monotonic()

    def stop_command(self) -> CtrlCmd:
        command = CtrlCmd()
        if hasattr(command, "longlCmdType"):
            command.longlCmdType = 2
        if hasattr(command, "steering"):
            command.steering = 0.0
        if hasattr(command, "velocity"):
            command.velocity = 0.0
        if hasattr(command, "accel"):
            command.accel = 0.0
        if hasattr(command, "acceleration"):
            command.acceleration = 0.0
        if hasattr(command, "brake"):
            command.brake = 1.0
        return command

    def lane_is_usable(self, now: float) -> bool:
        return bool(
            self.last_lane is not None
            and now - self.last_lane_time <= self.lane_timeout
            and self.last_lane.valid
            and self.last_lane.confidence >= self.min_lane_confidence
            and self.lateral_history
            and self.heading_history
        )

    def lane_steering(self) -> float:
        lateral = median(self.lateral_history)
        heading = median(self.heading_history)
        if not finite(lateral) or not finite(heading):
            return 0.0
        lateral_term = math.atan2(lateral, self.preview_distance_m)
        return self.lane_sign * (
            self.lateral_gain * lateral_term + self.heading_gain * heading
        )

    @staticmethod
    def command_velocity(command: Optional[CtrlCmd]) -> Optional[float]:
        if command is None or not hasattr(command, "velocity"):
            return None
        value = float(command.velocity)
        return max(0.0, value) if finite(value) else None

    def fallback_velocity(self) -> float:
        nominal_velocity = self.command_velocity(self.last_nominal)
        if nominal_velocity is not None:
            return min(nominal_velocity, self.fallback_speed_cap_mps)
        if (
            self.last_odom is not None
            and time.monotonic() - self.last_odom_time <= self.odom_timeout
        ):
            vx = float(self.last_odom.twist.twist.linear.x)
            vy = float(self.last_odom.twist.twist.linear.y)
            if finite(vx) and finite(vy):
                return min(math.hypot(vx, vy), self.fallback_speed_cap_mps)
        return min(self.last_output_velocity, self.fallback_speed_cap_mps)

    def apply_rate_limit(self, steering: float, now: float) -> float:
        steering = clamp(steering, -self.max_steering_rad, self.max_steering_rad)
        dt = max(1e-3, now - self.last_output_time)
        if self.max_steering_rate > 0.0:
            max_delta = self.max_steering_rate * dt
            steering = clamp(
                steering,
                self.last_output_steering - max_delta,
                self.last_output_steering + max_delta,
            )
        self.last_output_steering = steering
        return steering

    def quality_state(self, now: float) -> Tuple[str, str]:
        if self.last_quality is None or now - self.last_quality_time > self.quality_timeout:
            return SENSOR_DEGRADED, "sensor_quality_stale"
        return str(self.last_quality.state), str(self.last_quality.reason)

    def publish_status(
        self,
        mode: str,
        quality_state: str,
        reason: str,
        camera_used: bool,
        lane_usable: bool,
        nominal_fresh: bool,
    ) -> None:
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "mode": mode,
                        "quality_state": quality_state,
                        "quality_reason": reason,
                        "camera_used": camera_used,
                        "lane_usable": lane_usable,
                        "nominal_fresh": nominal_fresh,
                        "output_steering_rad": self.last_output_steering,
                        "output_velocity_mps": self.last_output_velocity,
                    },
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
        )

    def publish_command(self, _event) -> None:
        now = time.monotonic()
        quality_state, quality_reason = self.quality_state(now)
        nominal_fresh = (
            self.last_nominal is not None
            and now - self.last_nominal_time <= self.nominal_timeout
        )
        lane_usable = self.lane_is_usable(now)

        # Quality monitor가 정상이라고 판단하면 기존 경로 제어를 보존한다.
        if quality_state == NORMAL:
            if not nominal_fresh:
                self.output_pub.publish(self.stop_command())
                self.last_output_velocity = 0.0
                self.publish_status(
                    "normal_nominal_stale",
                    quality_state,
                    "nominal_command_stale",
                    False,
                    lane_usable,
                    False,
                )
                return
            output = copy.deepcopy(self.last_nominal)
            if hasattr(output, "steering"):
                self.last_output_steering = float(output.steering)
            velocity = self.command_velocity(output)
            if velocity is not None:
                self.last_output_velocity = velocity
            self.output_pub.publish(output)
            self.publish_status(
                "normal_nominal",
                quality_state,
                quality_reason,
                False,
                lane_usable,
                True,
            )
            return

        # 이상 상태에서는 차선이 유효할 때만 camera assist/fallback을 허용한다.
        if not lane_usable:
            if self.stop_without_camera:
                self.output_pub.publish(self.stop_command())
                self.last_output_steering = 0.0
                self.last_output_velocity = 0.0
                self.publish_status(
                    "degraded_no_camera_stop",
                    quality_state,
                    "camera_lane_unusable",
                    False,
                    False,
                    nominal_fresh,
                )
                return

            if not nominal_fresh:
                self.output_pub.publish(self.stop_command())
                self.last_output_velocity = 0.0
                self.publish_status(
                    "degraded_nominal_stale_stop",
                    quality_state,
                    "camera_only_without_speed_reference",
                    False,
                    False,
                    False,
                )
                return

            output = copy.deepcopy(self.last_nominal)
            if hasattr(output, "velocity"):
                output.velocity = min(
                    max(0.0, float(output.velocity)), self.fallback_speed_cap_mps
                )
            if hasattr(output, "steering"):
                output.steering = self.apply_rate_limit(float(output.steering), now)
            self.last_output_velocity = self.command_velocity(output) or 0.0
            self.output_pub.publish(output)
            self.publish_status(
                "degraded_nominal_limited",
                quality_state,
                quality_reason,
                False,
                False,
                True,
            )
            return

        lane_steering = clamp(
            self.lane_steering(), -self.max_steering_rad, self.max_steering_rad
        )
        if quality_state == GPS_BLACKOUT:
            # 위치가 사라진 경우에는 카메라를 주 제어로 사용한다. nominal은
            # 조향의 급격한 변화를 줄이는 보조항으로만 남긴다.
            if nominal_fresh and hasattr(self.last_nominal, "steering"):
                nominal_steering = float(self.last_nominal.steering)
                target_steering = (
                    self.fallback_nominal_weight * nominal_steering
                    + (1.0 - self.fallback_nominal_weight) * lane_steering
                )
            else:
                target_steering = lane_steering
            mode = "gps_blackout_camera_fallback"
        else:
            # GPS/IMU noise 또는 recovery는 원래 PP를 유지하면서 camera 보정을
            # 작게 더한다. 이상이 해제되면 NORMAL 경로로 자동 복귀한다.
            nominal_steering = (
                float(self.last_nominal.steering)
                if nominal_fresh and hasattr(self.last_nominal, "steering")
                else self.last_output_steering
            )
            target_steering = nominal_steering + self.camera_assist_gain * lane_steering
            mode = "sensor_anomaly_camera_assist"

        output = copy.deepcopy(self.last_nominal) if nominal_fresh else CtrlCmd()
        if hasattr(output, "longlCmdType"):
            output.longlCmdType = 2
        if hasattr(output, "steering"):
            output.steering = self.apply_rate_limit(target_steering, now)
        if not nominal_fresh and hasattr(output, "brake"):
            output.brake = 0.0
        if hasattr(output, "velocity"):
            output.velocity = self.fallback_velocity()
            self.last_output_velocity = max(0.0, float(output.velocity))
        else:
            self.last_output_velocity = self.fallback_velocity()
        self.output_pub.publish(output)
        self.publish_status(
            mode,
            quality_state,
            quality_reason,
            True,
            True,
            nominal_fresh,
        )


if __name__ == "__main__":
    try:
        CameraLocalizationFallbackController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
