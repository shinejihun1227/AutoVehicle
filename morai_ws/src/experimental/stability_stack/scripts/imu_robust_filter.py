#!/usr/bin/env python3
"""IMU의 순간 spike를 제한하고 EMA로 완화하는 2D 주행용 robust filter.

EKF가 주로 사용하는 gyro z축, 가속도 x축을 포함해 세 축 전체를 필터링한다.
orientation은 평면 주행을 가정해 yaw만 wrap-aware EMA로 필터링한다.
이 노드는 기존 주행 launch에는 연결하지 않고, 통합 곡률 주행 launch에서만
사용하도록 추가되었다.
"""

from __future__ import annotations

import copy
import math
from typing import List, Optional, Tuple

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> Optional[float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def yaw_to_quaternion(yaw_rad: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw_rad
    return 0.0, 0.0, math.sin(half), math.cos(half)


class ImuRobustFilter:
    def __init__(self) -> None:
        rospy.init_node("imu_robust_filter", anonymous=False)

        self.input_topic = rospy.get_param("~input_topic", "/Imu_noisy")
        self.output_topic = rospy.get_param("~output_topic", "/Imu_filtered")
        self.status_topic = rospy.get_param(
            "~status_topic", "/stability/imu_filter_status"
        )
        self.alpha = clamp(float(rospy.get_param("~ema_alpha", 0.35)), 0.0, 1.0)
        self.max_gyro_jump = max(
            0.0, float(rospy.get_param("~max_gyro_jump_rad_s", 2.0))
        )
        self.max_accel_jump = max(
            0.0, float(rospy.get_param("~max_accel_jump_m_s2", 10.0))
        )
        self.filter_orientation_yaw = bool(
            rospy.get_param("~filter_orientation_yaw", True)
        )
        self.max_yaw_jump = max(
            0.0, float(rospy.get_param("~max_yaw_jump_rad", math.radians(45.0)))
        )

        self.previous_gyro: Optional[List[float]] = None
        self.previous_accel: Optional[List[float]] = None
        self.previous_yaw: Optional[float] = None
        self.total = 0
        self.published = 0
        self.rejected = 0
        self.clamped = 0

        self.publisher = rospy.Publisher(self.output_topic, Imu, queue_size=50)
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=1, latch=True
        )
        rospy.Subscriber(self.input_topic, Imu, self.callback, queue_size=50)
        rospy.loginfo(
            "IMU robust filter: %s -> %s alpha=%.2f gyro_jump=%.2f accel_jump=%.2f "
            "yaw_filter=%s",
            self.input_topic,
            self.output_topic,
            self.alpha,
            self.max_gyro_jump,
            self.max_accel_jump,
            self.filter_orientation_yaw,
        )

    def filter_vector(
        self,
        values: List[float],
        previous: Optional[List[float]],
        max_jump: float,
    ) -> Tuple[List[float], bool]:
        if any(not math.isfinite(value) for value in values):
            raise ValueError("non-finite IMU value")
        if previous is None:
            return list(values), False

        bounded = list(values)
        was_clamped = False
        for index, value in enumerate(values):
            if max_jump > 0.0:
                delta = value - previous[index]
                limited_delta = clamp(delta, -max_jump, max_jump)
                if limited_delta != delta:
                    was_clamped = True
                bounded[index] = previous[index] + limited_delta
        filtered = [
            (1.0 - self.alpha) * previous[index] + self.alpha * bounded[index]
            for index in range(3)
        ]
        return filtered, was_clamped

    def filter_yaw(self, message: Imu) -> Tuple[Optional[float], bool]:
        yaw = quaternion_to_yaw(
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        )
        if yaw is None or not self.filter_orientation_yaw:
            return yaw, False
        if self.previous_yaw is None:
            return yaw, False

        delta = wrap_angle(yaw - self.previous_yaw)
        was_clamped = False
        if self.max_yaw_jump > 0.0:
            limited_delta = clamp(delta, -self.max_yaw_jump, self.max_yaw_jump)
            was_clamped = limited_delta != delta
            delta = limited_delta
        return wrap_angle(self.previous_yaw + self.alpha * delta), was_clamped

    def callback(self, message: Imu) -> None:
        self.total += 1
        gyro = [
            float(message.angular_velocity.x),
            float(message.angular_velocity.y),
            float(message.angular_velocity.z),
        ]
        accel = [
            float(message.linear_acceleration.x),
            float(message.linear_acceleration.y),
            float(message.linear_acceleration.z),
        ]

        try:
            filtered_gyro, gyro_clamped = self.filter_vector(
                gyro, self.previous_gyro, self.max_gyro_jump
            )
            filtered_accel, accel_clamped = self.filter_vector(
                accel, self.previous_accel, self.max_accel_jump
            )
            filtered_yaw, yaw_clamped = self.filter_yaw(message)
        except ValueError:
            self.rejected += 1
            self.publish_status("reject_nonfinite")
            return

        was_clamped = gyro_clamped or accel_clamped or yaw_clamped
        if was_clamped:
            self.clamped += 1

        filtered = copy.deepcopy(message)
        filtered.angular_velocity.x = filtered_gyro[0]
        filtered.angular_velocity.y = filtered_gyro[1]
        filtered.angular_velocity.z = filtered_gyro[2]
        filtered.linear_acceleration.x = filtered_accel[0]
        filtered.linear_acceleration.y = filtered_accel[1]
        filtered.linear_acceleration.z = filtered_accel[2]

        if filtered_yaw is not None and self.filter_orientation_yaw:
            qx, qy, qz, qw = yaw_to_quaternion(filtered_yaw)
            filtered.orientation.x = qx
            filtered.orientation.y = qy
            filtered.orientation.z = qz
            filtered.orientation.w = qw

        self.previous_gyro = filtered_gyro
        self.previous_accel = filtered_accel
        if filtered_yaw is not None:
            self.previous_yaw = filtered_yaw

        self.publisher.publish(filtered)
        self.published += 1
        self.publish_status("clamped" if was_clamped else "ok")

    def publish_status(self, state: str) -> None:
        self.status_pub.publish(
            String(
                data=(
                    "state=%s total=%d published=%d rejected=%d clamped=%d"
                    % (state, self.total, self.published, self.rejected, self.clamped)
                )
            )
        )


if __name__ == "__main__":
    try:
        ImuRobustFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
