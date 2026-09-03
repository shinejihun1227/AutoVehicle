#!/usr/bin/env python3
"""주행 중 GPS·IMU 품질을 감시하는 온라인 이상 판정 노드.

이 노드는 노이즈를 만들거나 센서 데이터를 필터링하지 않는다. 원본에 가까운
GPS 변환 결과와 원본 IMU를 관찰하면서 blackout, 급격한 GPS jump, GPS-odometry
잔차, IMU 샘플 급변을 판정한다. 차선 fallback 제어기는 이 상태만 사용한다.

정지 구간의 정확한 noise 크기 측정은 sensor_noise_estimator 패키지가 담당하고,
이 노드는 주행 중 제어 전환을 위한 보수적인 품질 감시를 담당한다.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from typing import Deque, Dict, Iterable, Optional, Tuple

import rospy
from morai_perception_msgs.msg import GpsHealth, SensorQuality
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String


NORMAL = "NORMAL"
GPS_BLACKOUT = "GPS_BLACKOUT"
GPS_NOISE = "GPS_NOISE"
IMU_NOISE = "IMU_NOISE"
SENSOR_DEGRADED = "SENSOR_DEGRADED"


def finite(value: float) -> bool:
    return math.isfinite(float(value))


def norm3(values: Iterable[float]) -> float:
    x, y, z = [float(value) for value in values]
    return math.sqrt(x * x + y * y + z * z)


def robust_center_sigma(values: Iterable[float]) -> Tuple[float, float]:
    samples = sorted(float(value) for value in values if finite(value))
    if not samples:
        return math.nan, math.nan
    middle = len(samples) // 2
    if len(samples) % 2:
        center = samples[middle]
    else:
        center = 0.5 * (samples[middle - 1] + samples[middle])
    deviations = sorted(abs(value - center) for value in samples)
    middle = len(deviations) // 2
    if len(deviations) % 2:
        mad = deviations[middle]
    else:
        mad = 0.5 * (deviations[middle - 1] + deviations[middle])
    return center, 1.4826 * mad


class SensorQualityMonitor:
    def __init__(self) -> None:
        rospy.init_node("sensor_quality_monitor", anonymous=False)

        self.gps_topic = rospy.get_param("~gps_topic", "/localization/gps")
        self.imu_topic = rospy.get_param("~imu_topic", "/Imu")
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/odometry")
        self.health_topic = rospy.get_param(
            "~health_topic", "/localization/gps_health"
        )
        self.quality_topic = rospy.get_param(
            "~quality_topic", "/localization/sensor_quality"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/stability/sensor_quality_status"
        )

        self.publish_rate_hz = max(
            1.0, float(rospy.get_param("~publish_rate_hz", 10.0))
        )
        self.sensor_timeout_sec = max(
            0.1, float(rospy.get_param("~sensor_timeout_sec", 0.5))
        )

        # These are anomaly gates, not assumed MORAI noise ranges. They should be
        # tuned after collecting the official sensor range and real logs.
        self.gps_jump_threshold_m = max(
            0.1, float(rospy.get_param("~gps_jump_threshold_m", 3.0))
        )
        self.gps_step_speed_threshold_mps = max(
            0.5, float(rospy.get_param("~gps_step_speed_threshold_mps", 10.0))
        )
        self.gps_residual_threshold_m = max(
            0.1, float(rospy.get_param("~gps_residual_threshold_m", 2.0))
        )
        self.residual_sigma_multiplier = max(
            1.0, float(rospy.get_param("~residual_sigma_multiplier", 6.0))
        )
        self.min_residual_samples = max(
            5, int(rospy.get_param("~min_residual_samples", 20))
        )

        self.imu_gyro_diff_threshold = max(
            0.01, float(rospy.get_param("~imu_gyro_diff_threshold_rad_s", 0.8))
        )
        self.imu_accel_diff_threshold = max(
            0.1, float(rospy.get_param("~imu_accel_diff_threshold_m_s2", 6.0))
        )
        self.imu_sigma_multiplier = max(
            1.0, float(rospy.get_param("~imu_sigma_multiplier", 8.0))
        )

        self.bad_confirm_samples = max(
            1, int(rospy.get_param("~bad_confirm_samples", 2))
        )
        self.good_clear_samples = max(
            self.bad_confirm_samples,
            int(rospy.get_param("~good_clear_samples", 10)),
        )

        self.lock = threading.RLock()
        self.last_gps_time: Optional[float] = None
        self.last_gps_position: Optional[Tuple[float, float]] = None
        self.last_gps_step_speed = math.nan
        self.last_gps_residual = math.nan
        self.gps_residuals: Deque[float] = deque(maxlen=200)
        self.gps_bad_streak = 0
        self.gps_good_streak = 0

        self.last_imu_time: Optional[float] = None
        self.last_gyro: Optional[Tuple[float, float, float]] = None
        self.last_accel: Optional[Tuple[float, float, float]] = None
        self.gyro_diffs: Deque[float] = deque(maxlen=200)
        self.accel_diffs: Deque[float] = deque(maxlen=200)
        self.last_gyro_spike_score = 0.0
        self.last_accel_spike_score = 0.0
        self.imu_bad_streak = 0
        self.imu_good_streak = 0

        self.last_odom: Optional[Odometry] = None
        self.last_odom_time = 0.0
        self.last_health: Optional[GpsHealth] = None
        self.last_health_time = 0.0

        self.quality_pub = rospy.Publisher(
            self.quality_topic, SensorQuality, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=1, latch=True
        )
        rospy.Subscriber(self.gps_topic, Odometry, self.gps_callback, queue_size=50)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=100)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber(
            self.health_topic, GpsHealth, self.health_callback, queue_size=20
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz), self.publish_quality
        )

        rospy.loginfo(
            "Sensor quality monitor gps=%s imu=%s odom=%s health=%s output=%s",
            self.gps_topic,
            self.imu_topic,
            self.odom_topic,
            self.health_topic,
            self.quality_topic,
        )

    def odom_callback(self, message: Odometry) -> None:
        with self.lock:
            self.last_odom = message
            self.last_odom_time = time.monotonic()

    def health_callback(self, message: GpsHealth) -> None:
        with self.lock:
            self.last_health = message
            self.last_health_time = time.monotonic()

    def gps_callback(self, message: Odometry) -> None:
        now = time.monotonic()
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        if not finite(x) or not finite(y):
            return

        with self.lock:
            step_speed = math.nan
            jump = False
            if self.last_gps_position is not None and self.last_gps_time is not None:
                dt = now - self.last_gps_time
                distance = math.hypot(
                    x - self.last_gps_position[0], y - self.last_gps_position[1]
                )
                step_speed = distance / dt if dt > 1e-3 else math.inf
                jump = distance > self.gps_jump_threshold_m or (
                    step_speed > self.gps_step_speed_threshold_mps
                )

            residual = math.nan
            if (
                self.last_odom is not None
                and now - self.last_odom_time <= self.sensor_timeout_sec
            ):
                ox = float(self.last_odom.pose.pose.position.x)
                oy = float(self.last_odom.pose.pose.position.y)
                if finite(ox) and finite(oy):
                    residual = math.hypot(x - ox, y - oy)

            residual_bad = False
            if finite(residual) and len(self.gps_residuals) >= self.min_residual_samples:
                center, sigma = robust_center_sigma(self.gps_residuals)
                adaptive_limit = center + self.residual_sigma_multiplier * max(
                    sigma, 1e-3
                )
                residual_bad = (
                    residual > self.gps_residual_threshold_m
                    and residual > adaptive_limit
                )

            if finite(residual) and not jump:
                self.gps_residuals.append(residual)

            anomaly = jump or residual_bad
            if anomaly:
                self.gps_bad_streak += 1
                self.gps_good_streak = 0
            else:
                self.gps_good_streak += 1
                if self.gps_good_streak >= self.good_clear_samples:
                    self.gps_bad_streak = 0

            self.last_gps_time = now
            self.last_gps_position = (x, y)
            self.last_gps_step_speed = step_speed
            self.last_gps_residual = residual

    def imu_callback(self, message: Imu) -> None:
        now = time.monotonic()
        gyro = (
            float(message.angular_velocity.x),
            float(message.angular_velocity.y),
            float(message.angular_velocity.z),
        )
        accel = (
            float(message.linear_acceleration.x),
            float(message.linear_acceleration.y),
            float(message.linear_acceleration.z),
        )
        if not all(finite(value) for value in gyro + accel):
            return

        with self.lock:
            gyro_score = 0.0
            accel_score = 0.0
            gyro_bad = False
            accel_bad = False
            if self.last_gyro is not None:
                gyro_diff = norm3(
                    [gyro[index] - self.last_gyro[index] for index in range(3)]
                )
                center, sigma = robust_center_sigma(self.gyro_diffs)
                if finite(center):
                    limit = max(
                        self.imu_gyro_diff_threshold,
                        center + self.imu_sigma_multiplier * max(sigma, 1e-3),
                    )
                    gyro_score = gyro_diff / max(limit, 1e-6)
                    gyro_bad = gyro_diff > limit
                self.gyro_diffs.append(gyro_diff)

            if self.last_accel is not None:
                accel_diff = norm3(
                    [accel[index] - self.last_accel[index] for index in range(3)]
                )
                center, sigma = robust_center_sigma(self.accel_diffs)
                if finite(center):
                    limit = max(
                        self.imu_accel_diff_threshold,
                        center + self.imu_sigma_multiplier * max(sigma, 1e-3),
                    )
                    accel_score = accel_diff / max(limit, 1e-6)
                    accel_bad = accel_diff > limit
                self.accel_diffs.append(accel_diff)

            anomaly = gyro_bad or accel_bad
            if anomaly:
                self.imu_bad_streak += 1
                self.imu_good_streak = 0
            else:
                self.imu_good_streak += 1
                if self.imu_good_streak >= self.good_clear_samples:
                    self.imu_bad_streak = 0

            self.last_imu_time = now
            self.last_gyro = gyro
            self.last_accel = accel
            self.last_gyro_spike_score = gyro_score
            self.last_accel_spike_score = accel_score

    def snapshot(self) -> Dict[str, object]:
        now = time.monotonic()
        with self.lock:
            gps_age = (
                math.inf if self.last_gps_time is None else now - self.last_gps_time
            )
            imu_age = (
                math.inf if self.last_imu_time is None else now - self.last_imu_time
            )
            health_age = (
                math.inf if self.last_health_time <= 0.0 else now - self.last_health_time
            )

            health_fresh = self.last_health is not None and health_age <= self.sensor_timeout_sec
            if health_fresh:
                gps_valid = bool(self.last_health.valid)
                gps_blackout = bool(self.last_health.blackout)
                gps_recovering = self.last_health.state == "GPS_RECOVERING"
                health_reason = str(self.last_health.reason)
            else:
                gps_valid = gps_age <= self.sensor_timeout_sec
                gps_blackout = not gps_valid
                gps_recovering = False
                health_reason = "gps_health_stale" if self.last_health is not None else "no_gps_health"

            gps_noisy = self.gps_bad_streak >= self.bad_confirm_samples
            imu_noisy = self.imu_bad_streak >= self.bad_confirm_samples
            imu_stale = imu_age > self.sensor_timeout_sec
            gps_stale = gps_age > self.sensor_timeout_sec

            if gps_blackout:
                state = GPS_BLACKOUT
                reason = health_reason or "gps_blackout"
            elif gps_noisy:
                state = GPS_NOISE
                reason = "gps_jump_or_residual"
            elif imu_noisy:
                state = IMU_NOISE
                reason = "imu_sample_spike"
            elif gps_stale or imu_stale or gps_recovering:
                state = SENSOR_DEGRADED
                reason = "sensor_stale_or_gps_recovering"
            else:
                state = NORMAL
                reason = "sensors_nominal"

            if gps_recovering and state == NORMAL:
                state = SENSOR_DEGRADED
            if state == GPS_BLACKOUT and imu_stale:
                reason = "gps_blackout_and_imu_stale"
            elif state == GPS_NOISE and imu_noisy:
                reason = "gps_and_imu_anomaly"

            confidence = 1.0
            if gps_blackout:
                confidence = 0.0
            elif gps_noisy or imu_noisy or gps_recovering:
                confidence = 0.35
            elif gps_stale or imu_stale:
                confidence = 0.2

            return {
                "state": state,
                "gps_valid": gps_valid,
                "gps_blackout": gps_blackout,
                "gps_noisy": gps_noisy,
                "imu_noisy": imu_noisy,
                "gps_recovering": gps_recovering,
                "imu_stale": imu_stale,
                "gps_residual_m": self.last_gps_residual,
                "gps_step_speed_mps": self.last_gps_step_speed,
                "imu_gyro_spike_score": self.last_gyro_spike_score,
                "imu_accel_spike_score": self.last_accel_spike_score,
                "confidence": confidence,
                "reason": reason,
                "gps_age_sec": gps_age,
                "imu_age_sec": imu_age,
                "gps_bad_streak": self.gps_bad_streak,
                "imu_bad_streak": self.imu_bad_streak,
            }

    def publish_quality(self, _event) -> None:
        data = self.snapshot()
        message = SensorQuality()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "base_link"
        message.state = str(data["state"])
        message.gps_valid = bool(data["gps_valid"])
        message.gps_blackout = bool(data["gps_blackout"])
        message.gps_noisy = bool(data["gps_noisy"])
        message.imu_noisy = bool(data["imu_noisy"])
        message.gps_recovering = bool(data["gps_recovering"])
        message.imu_stale = bool(data["imu_stale"])
        message.gps_residual_m = float(data["gps_residual_m"] if finite(float(data["gps_residual_m"])) else -1.0)
        message.gps_step_speed_mps = float(data["gps_step_speed_mps"] if finite(float(data["gps_step_speed_mps"])) else -1.0)
        message.imu_gyro_spike_score = float(data["imu_gyro_spike_score"])
        message.imu_accel_spike_score = float(data["imu_accel_spike_score"])
        message.confidence = float(data["confidence"])
        message.reason = str(data["reason"])
        self.quality_pub.publish(message)

        json_data = dict(data)
        for key in ("gps_age_sec", "imu_age_sec"):
            if not finite(float(json_data[key])):
                json_data[key] = None
        self.status_pub.publish(
            String(data=json.dumps(json_data, ensure_ascii=False, sort_keys=True))
        )
        rospy.loginfo_throttle(
            2.0,
            "sensor quality state=%s reason=%s gps_bad=%d imu_bad=%d",
            data["state"],
            data["reason"],
            data["gps_bad_streak"],
            data["imu_bad_streak"],
        )


if __name__ == "__main__":
    try:
        SensorQualityMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
