#!/usr/bin/env python3
"""원본 MORAI GPS·IMU의 정지 구간 노이즈를 관찰하는 독립 ROS 노드.

이 노드는 어떤 센서도 보정하거나 주행 명령에 연결하지 않는다.
`/localization/gps`와 `/Imu`를 읽어 차량이 정지한 구간에서만 robust 통계를
계산하고, 결과를 진단용 토픽과 선택적 CSV로 남긴다.

GPS의 중심값은 절대 bias라고 부르지 않는다. 정확한 기준 위치가 없으면 GPS
관측의 중심과 반복성만 알 수 있고, 절대 위치 bias는 알 수 없기 때문이다.
Gyro의 정지 평균은 실제 회전이 0이라는 가정에서 gyro bias 후보로 해석한다.
가속도 평균은 차체 자세와 중력 투영을 모르면 bias가 아니라 정지 기준값으로
기록한다.
"""

from __future__ import annotations

import csv
import json
import math
import os
import threading
import time
from collections import deque
from typing import Deque, Dict, Iterable, Optional, Tuple

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from sensor_noise_estimator.msg import NoiseStatistics
from sensor_noise_estimator.statistics import (
    robust_center_sigma,
    robust_speed_from_gps,
    sample_rate_and_max_gap,
)


GpsSample = Tuple[float, float, float]
ImuSample = Tuple[float, Tuple[float, float, float], Tuple[float, float, float]]


def finite(value: float) -> bool:
    return math.isfinite(float(value))


def message_time(stamp, fallback: Optional[float] = None) -> float:
    value = stamp.to_sec() if stamp is not None else 0.0
    if value > 0.0 and finite(value):
        return float(value)
    return float(time.monotonic() if fallback is None else fallback)


def vector_norm(values: Iterable[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in values))


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    return value


class SensorNoiseEstimator:
    def __init__(self) -> None:
        rospy.init_node("sensor_noise_estimator", anonymous=False)

        self.gps_topic = rospy.get_param("~gps_topic", "/localization/gps")
        self.imu_topic = rospy.get_param("~imu_topic", "/Imu")
        self.speed_topic = str(rospy.get_param("~speed_topic", "")).strip()
        self.stats_topic = rospy.get_param(
            "~stats_topic", "/localization/noise_statistics"
        )
        self.json_topic = rospy.get_param(
            "~json_topic", "/localization/noise_statistics_json"
        )
        self.csv_path = str(rospy.get_param("~csv_path", "")).strip()

        self.sensor_timeout_sec = max(
            0.05, float(rospy.get_param("~sensor_timeout_sec", 0.5))
        )
        self.stationary_speed_mps = max(
            0.0, float(rospy.get_param("~stationary_speed_mps", 0.15))
        )
        self.gyro_stationary_threshold = max(
            0.0,
            float(rospy.get_param("~gyro_stationary_threshold_rad_s", 0.04)),
        )
        self.accel_xy_stationary_threshold = max(
            0.0,
            float(rospy.get_param("~accel_xy_stationary_threshold_m_s2", 0.6)),
        )
        self.gps_stationary_speed_threshold = max(
            0.0,
            float(rospy.get_param("~gps_stationary_speed_threshold_mps", 1.0)),
        )
        self.use_gps_motion_without_speed = bool(
            rospy.get_param("~use_gps_motion_without_speed", True)
        )
        self.settle_time_sec = max(
            0.0, float(rospy.get_param("~settle_time_sec", 1.5))
        )
        self.window_duration_sec = max(
            0.5, float(rospy.get_param("~window_duration_sec", 5.0))
        )
        self.update_period_sec = max(
            0.1, float(rospy.get_param("~update_period_sec", 1.0))
        )
        self.min_gps_samples = max(
            2, int(rospy.get_param("~min_gps_samples", 10))
        )
        self.min_imu_samples = max(
            2, int(rospy.get_param("~min_imu_samples", 30))
        )
        self.publish_rate_hz = max(
            1.0, float(rospy.get_param("~publish_rate_hz", 5.0))
        )

        self.lock = threading.RLock()
        self.recent_gps: Deque[GpsSample] = deque(maxlen=2000)
        self.recent_imu: Deque[ImuSample] = deque(maxlen=5000)
        self.segment_gps: Deque[GpsSample] = deque(maxlen=10000)
        self.segment_imu: Deque[ImuSample] = deque(maxlen=20000)
        self.last_gps: Optional[GpsSample] = None
        self.last_imu: Optional[ImuSample] = None
        # 메시지 header stamp는 ROS/시뮬레이터 시계일 수 있고, 정지 판정의
        # freshness는 wall monotonic 시계여야 한다. 두 시계를 섞지 않는다.
        self.last_gps_arrival = 0.0
        self.last_imu_arrival = 0.0
        self.last_speed: Optional[float] = None
        self.last_speed_time = 0.0
        self.stationary = False
        self.stationary_since: Optional[float] = None
        self.last_update_time = 0.0
        self.total_stationary_updates = 0
        self.latest_stats: Dict[str, float] = {}

        self.stats_pub = rospy.Publisher(
            self.stats_topic, NoiseStatistics, queue_size=10
        )
        self.json_pub = rospy.Publisher(self.json_topic, String, queue_size=10)
        rospy.Subscriber(self.gps_topic, Odometry, self.gps_callback, queue_size=100)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=200)
        if self.speed_topic:
            rospy.Subscriber(
                self.speed_topic, Odometry, self.speed_callback, queue_size=20
            )

        self.csv_fields = [
            "wall_time",
            "mode",
            "stationary",
            "calibration_valid",
            "stationary_duration_sec",
            "gps_samples",
            "imu_samples",
            "gps_center_x_m",
            "gps_center_y_m",
            "gps_std_x_m",
            "gps_std_y_m",
            "gps_3sigma_x_m",
            "gps_3sigma_y_m",
            "gyro_mean_x_rad_s",
            "gyro_mean_y_rad_s",
            "gyro_mean_z_rad_s",
            "gyro_std_x_rad_s",
            "gyro_std_y_rad_s",
            "gyro_std_z_rad_s",
            "accel_mean_x_m_s2",
            "accel_mean_y_m_s2",
            "accel_mean_z_m_s2",
            "accel_std_x_m_s2",
            "accel_std_y_m_s2",
            "accel_std_z_m_s2",
            "gps_rate_hz",
            "imu_rate_hz",
            "gps_max_gap_sec",
            "imu_max_gap_sec",
        ]
        self._prepare_csv()

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz), self.timer_callback
        )
        rospy.loginfo(
            "sensor noise estimator: gps=%s imu=%s speed=%s stats=%s csv=%s",
            self.gps_topic,
            self.imu_topic,
            self.speed_topic or "none(gps motion fallback)",
            self.stats_topic,
            self.csv_path or "disabled",
        )

    def _prepare_csv(self) -> None:
        if not self.csv_path:
            return
        parent = os.path.dirname(os.path.abspath(self.csv_path))
        if parent:
            os.makedirs(parent, exist_ok=True)
        needs_header = not os.path.isfile(self.csv_path) or os.path.getsize(self.csv_path) == 0
        if needs_header:
            with open(self.csv_path, "a", newline="", encoding="utf-8") as fp:
                csv.writer(fp).writerow(self.csv_fields)

    def gps_callback(self, message: Odometry) -> None:
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        if not finite(x) or not finite(y):
            return
        sample = (message_time(message.header.stamp), x, y)
        with self.lock:
            self.last_gps = sample
            self.last_gps_arrival = time.monotonic()
            self.recent_gps.append(sample)
            if self.stationary:
                self.segment_gps.append(sample)
                self._trim_segment_locked(sample[0])

    def imu_callback(self, message: Imu) -> None:
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
        sample = (message_time(message.header.stamp), gyro, accel)
        with self.lock:
            self.last_imu = sample
            self.last_imu_arrival = time.monotonic()
            self.recent_imu.append(sample)
            if self.stationary:
                self.segment_imu.append(sample)
                self._trim_segment_locked(sample[0])

    def speed_callback(self, message: Odometry) -> None:
        vx = float(message.twist.twist.linear.x)
        vy = float(message.twist.twist.linear.y)
        if not finite(vx) or not finite(vy):
            return
        stamp = message_time(message.header.stamp)
        with self.lock:
            self.last_speed = math.hypot(vx, vy)
            self.last_speed_time = time.monotonic()

    def _trim_segment_locked(self, newest_stamp: float) -> None:
        cutoff = newest_stamp - self.window_duration_sec
        while self.segment_gps and self.segment_gps[0][0] < cutoff:
            self.segment_gps.popleft()
        while self.segment_imu and self.segment_imu[0][0] < cutoff:
            self.segment_imu.popleft()

    def _fresh(self, stamp: Optional[float], now: float) -> bool:
        return stamp is not None and abs(now - stamp) <= self.sensor_timeout_sec

    def _gps_motion_speed_locked(self) -> float:
        if len(self.recent_gps) < 3:
            return math.nan
        samples = list(self.recent_gps)
        newest = samples[-1][0]
        cutoff = newest - max(1.0, self.window_duration_sec / 2.0)
        recent = [sample for sample in samples if sample[0] >= cutoff]
        return robust_speed_from_gps(recent)

    def _stationary_score_locked(self, now: float) -> Tuple[bool, float, str]:
        if self.last_imu is None or not self._fresh(self.last_imu_arrival, now):
            return False, 0.0, "imu_stale"

        _, gyro, accel = self.last_imu
        gyro_norm = vector_norm(gyro)
        accel_xy_norm = vector_norm(accel[:2])
        gyro_score = max(
            0.0,
            1.0 - gyro_norm / max(self.gyro_stationary_threshold, 1e-6),
        )
        accel_score = max(
            0.0,
            1.0
            - accel_xy_norm / max(self.accel_xy_stationary_threshold, 1e-6),
        )
        imu_ok = (
            gyro_norm <= self.gyro_stationary_threshold
            and accel_xy_norm <= self.accel_xy_stationary_threshold
        )

        if self.speed_topic:
            speed_fresh = now - self.last_speed_time <= self.sensor_timeout_sec
            speed_ok = speed_fresh and self.last_speed is not None and self.last_speed <= self.stationary_speed_mps
            if not speed_fresh:
                return False, 0.0, "speed_stale"
            score = min(gyro_score, accel_score, max(0.0, 1.0 - self.last_speed / max(self.stationary_speed_mps, 1e-6)))
            return bool(imu_ok and speed_ok), float(score), "speed_imu"

        gps_speed = self._gps_motion_speed_locked()
        if self.use_gps_motion_without_speed:
            if not finite(gps_speed):
                return False, 0.0, "waiting_gps_motion"
            gps_ok = gps_speed <= self.gps_stationary_speed_threshold
            gps_score = max(
                0.0,
                1.0 - gps_speed / max(self.gps_stationary_speed_threshold, 1e-6),
            )
            return bool(imu_ok and gps_ok), float(min(gyro_score, accel_score, gps_score)), "gps_imu"
        return bool(imu_ok), float(min(gyro_score, accel_score)), "imu_only"

    def _update_stationary_state_locked(self, now: float) -> Tuple[bool, float, str]:
        stationary, score, mode = self._stationary_score_locked(now)
        if stationary and not self.stationary:
            self.segment_gps.clear()
            self.segment_imu.clear()
            self.stationary_since = now
        elif not stationary and self.stationary:
            self.segment_gps.clear()
            self.segment_imu.clear()
            self.stationary_since = None
        self.stationary = stationary
        return stationary, score, mode

    @staticmethod
    def _axis_stats(values: Iterable[float]) -> Tuple[float, float]:
        return robust_center_sigma(values)

    def _estimate_locked(self, now: float, score: float, mode: str) -> Dict[str, float]:
        gps = list(self.segment_gps)
        imu = list(self.segment_imu)
        gps_x, gps_sigma_x = self._axis_stats(sample[1] for sample in gps)
        gps_y, gps_sigma_y = self._axis_stats(sample[2] for sample in gps)

        gyro_axes = [[sample[1][axis] for sample in imu] for axis in range(3)]
        accel_axes = [[sample[2][axis] for sample in imu] for axis in range(3)]
        gyro_stats = [self._axis_stats(axis) for axis in gyro_axes]
        accel_stats = [self._axis_stats(axis) for axis in accel_axes]
        gps_rate, gps_gap = sample_rate_and_max_gap(gps)
        imu_rate, imu_gap = sample_rate_and_max_gap(imu)
        gps_speed = robust_speed_from_gps(gps)

        result: Dict[str, float] = {
            "stationary": True,
            "calibration_valid": len(gps) >= self.min_gps_samples and len(imu) >= self.min_imu_samples,
            "stationary_duration_sec": max(0.0, now - (self.stationary_since or now)),
            "stationary_score": score,
            "current_window_samples_gps": len(gps),
            "current_window_samples_imu": len(imu),
            "gps_center_x_m": gps_x,
            "gps_center_y_m": gps_y,
            "gps_std_x_m": gps_sigma_x,
            "gps_std_y_m": gps_sigma_y,
            "gps_3sigma_x_m": 3.0 * gps_sigma_x if finite(gps_sigma_x) else math.nan,
            "gps_3sigma_y_m": 3.0 * gps_sigma_y if finite(gps_sigma_y) else math.nan,
            "gps_step_speed_mps": gps_speed,
            "gps_rate_hz": gps_rate,
            "gps_max_gap_sec": gps_gap,
            "imu_rate_hz": imu_rate,
            "imu_max_gap_sec": imu_gap,
            "total_stationary_updates": self.total_stationary_updates + 1,
        }
        for name, stats, scale in (("gyro", gyro_stats, 3.0), ("accel", accel_stats, 3.0)):
            for axis, (center, sigma) in zip(("x", "y", "z"), stats):
                result[f"{name}_mean_{axis}"] = center
                result[f"{name}_std_{axis}"] = sigma
                result[f"{name}_3sigma_{axis}"] = scale * sigma if finite(sigma) else math.nan
        return result

    def _empty_result(self, stationary: bool, score: float, mode: str, now: float) -> Dict[str, float]:
        result: Dict[str, float] = {
            "stationary": stationary,
            "calibration_valid": False,
            "stationary_duration_sec": max(0.0, now - (self.stationary_since or now)) if stationary else 0.0,
            "stationary_score": score,
            "current_window_samples_gps": len(self.segment_gps),
            "current_window_samples_imu": len(self.segment_imu),
            "total_stationary_updates": self.total_stationary_updates,
        }
        for key in (
            "gps_center_x_m", "gps_center_y_m", "gps_std_x_m", "gps_std_y_m",
            "gps_3sigma_x_m", "gps_3sigma_y_m", "gps_step_speed_mps",
            "gps_rate_hz", "gps_max_gap_sec", "imu_rate_hz", "imu_max_gap_sec",
        ):
            result[key] = math.nan
        for name in ("gyro", "accel"):
            for axis in ("x", "y", "z"):
                result[f"{name}_mean_{axis}"] = math.nan
                result[f"{name}_std_{axis}"] = math.nan
                result[f"{name}_3sigma_{axis}"] = math.nan
        return result

    def _make_message(self, result: Dict[str, float], mode: str) -> NoiseStatistics:
        message = NoiseStatistics()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "base_link"
        message.stationary = bool(result["stationary"])
        message.calibration_valid = bool(result["calibration_valid"])
        message.mode = mode
        message.current_window_samples_gps = int(result["current_window_samples_gps"])
        message.current_window_samples_imu = int(result["current_window_samples_imu"])
        message.total_stationary_updates = int(result["total_stationary_updates"])

        float_fields = (
            "stationary_duration_sec", "stationary_score", "gps_center_x_m",
            "gps_center_y_m", "gps_std_x_m", "gps_std_y_m", "gps_3sigma_x_m",
            "gps_3sigma_y_m", "gps_step_speed_mps", "gps_rate_hz", "imu_rate_hz",
            "gps_max_gap_sec", "imu_max_gap_sec",
        )
        for field in float_fields:
            setattr(message, field, float(result[field]))
        for name, prefix in (("gyro", "gyro"), ("accel", "accel")):
            for axis in ("x", "y", "z"):
                setattr(message, f"{prefix}_mean_{axis}_rad_s" if name == "gyro" else f"{prefix}_mean_{axis}_m_s2", float(result[f"{name}_mean_{axis}"]))
                setattr(message, f"{prefix}_std_{axis}_rad_s" if name == "gyro" else f"{prefix}_std_{axis}_m_s2", float(result[f"{name}_std_{axis}"]))
                setattr(message, f"{prefix}_3sigma_{axis}_rad_s" if name == "gyro" else f"{prefix}_3sigma_{axis}_m_s2", float(result[f"{name}_3sigma_{axis}"]))
        return message

    def _write_csv(self, result: Dict[str, float], mode: str) -> None:
        if not self.csv_path:
            return
        row = {
            "wall_time": time.time(),
            "mode": mode,
            "stationary": result["stationary"],
            "calibration_valid": result["calibration_valid"],
            "stationary_duration_sec": result["stationary_duration_sec"],
            "gps_samples": result["current_window_samples_gps"],
            "imu_samples": result["current_window_samples_imu"],
            "gps_center_x_m": result["gps_center_x_m"],
            "gps_center_y_m": result["gps_center_y_m"],
            "gps_std_x_m": result["gps_std_x_m"],
            "gps_std_y_m": result["gps_std_y_m"],
            "gps_3sigma_x_m": result["gps_3sigma_x_m"],
            "gps_3sigma_y_m": result["gps_3sigma_y_m"],
            "gyro_mean_x_rad_s": result["gyro_mean_x"],
            "gyro_mean_y_rad_s": result["gyro_mean_y"],
            "gyro_mean_z_rad_s": result["gyro_mean_z"],
            "gyro_std_x_rad_s": result["gyro_std_x"],
            "gyro_std_y_rad_s": result["gyro_std_y"],
            "gyro_std_z_rad_s": result["gyro_std_z"],
            "accel_mean_x_m_s2": result["accel_mean_x"],
            "accel_mean_y_m_s2": result["accel_mean_y"],
            "accel_mean_z_m_s2": result["accel_mean_z"],
            "accel_std_x_m_s2": result["accel_std_x"],
            "accel_std_y_m_s2": result["accel_std_y"],
            "accel_std_z_m_s2": result["accel_std_z"],
            "gps_rate_hz": result["gps_rate_hz"],
            "imu_rate_hz": result["imu_rate_hz"],
            "gps_max_gap_sec": result["gps_max_gap_sec"],
            "imu_max_gap_sec": result["imu_max_gap_sec"],
        }
        with open(self.csv_path, "a", newline="", encoding="utf-8") as fp:
            csv.DictWriter(fp, fieldnames=self.csv_fields).writerow(row)

    def timer_callback(self, _event) -> None:
        now = time.monotonic()
        with self.lock:
            stationary, score, mode = self._update_stationary_state_locked(now)
            if stationary and self.stationary_since is not None:
                duration = now - self.stationary_since
            else:
                duration = 0.0

            enough_time = duration >= self.settle_time_sec
            enough_samples = (
                len(self.segment_gps) >= self.min_gps_samples
                and len(self.segment_imu) >= self.min_imu_samples
            )
            should_update = (
                stationary
                and enough_time
                and enough_samples
                and now - self.last_update_time >= self.update_period_sec
            )
            if should_update:
                self.total_stationary_updates += 1
                result = self._estimate_locked(now, score, mode)
                result["total_stationary_updates"] = self.total_stationary_updates
                self.last_update_time = now
            elif stationary and self.latest_stats:
                result = dict(self.latest_stats)
                result["stationary"] = True
                result["stationary_duration_sec"] = duration
                result["stationary_score"] = score
                result["current_window_samples_gps"] = len(self.segment_gps)
                result["current_window_samples_imu"] = len(self.segment_imu)
                result["total_stationary_updates"] = self.total_stationary_updates
            else:
                result = self._empty_result(stationary, score, mode, now)

            if should_update:
                self.latest_stats = dict(result)
            message = self._make_message(result, mode)
            payload = dict(result)
            payload["mode"] = mode
            payload["stationary"] = bool(result["stationary"])
            payload["calibration_valid"] = bool(result["calibration_valid"])
            self._write_csv(result, mode)

        self.stats_pub.publish(message)
        self.json_pub.publish(String(data=json.dumps(json_safe(payload), ensure_ascii=False)))
        if result["calibration_valid"]:
            rospy.loginfo_throttle(
                10.0,
                "sensor noise measured: gps_sigma=(%.3f, %.3f)m gyro_bias_z=%.5f rad/s accel_mean_x=%.4f m/s2",
                result["gps_std_x_m"],
                result["gps_std_y_m"],
                result["gyro_mean_z"],
                result["accel_mean_x"],
            )


if __name__ == "__main__":
    try:
        SensorNoiseEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
