#!/usr/bin/env python3
"""IMU에 gyro·accel white noise와 bias random walk를 추가한다.

기본값은 quaternion을 보존한다. 현재 EKF가 gyro 예측과 quaternion yaw 보정을
동시에 사용하므로, orientation noise는 별도 파라미터로 명시적으로 켠다.
"""

from __future__ import annotations

import copy
import math
import random
import time

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def quaternion_to_yaw(x: float, y: float, z: float, w: float):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class ImuNoiseInjector:
    def __init__(self) -> None:
        rospy.init_node("imu_noise_injector", anonymous=False)
        self.input_topic = rospy.get_param("~input_topic", "/Imu")
        self.output_topic = rospy.get_param("~output_topic", "/Imu_noisy")
        self.gyro_std = float(rospy.get_param("~gyro_noise_std_rad_s", 0.01))
        self.accel_std = float(rospy.get_param("~accel_noise_std_m_s2", 0.1))
        self.gyro_bias_rw = float(rospy.get_param("~gyro_bias_random_walk_rad_s_sqrt_s", 0.002))
        self.accel_bias_rw = float(rospy.get_param("~accel_bias_random_walk_m_s2_sqrt_s", 0.02))
        self.orientation_yaw_std = float(rospy.get_param("~orientation_yaw_noise_std_rad", 0.0))
        self.dropout_probability = float(rospy.get_param("~dropout_probability", 0.0))
        self.seed = int(rospy.get_param("~random_seed", 20260823))
        self.rng = random.Random(self.seed)
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.accel_bias = [0.0, 0.0, 0.0]
        self.last_time = time.monotonic()
        self.total = 0
        self.dropped = 0

        self.publisher = rospy.Publisher(self.output_topic, Imu, queue_size=50)
        self.status_pub = rospy.Publisher("/stability/imu_noise_status", String, queue_size=1, latch=True)
        rospy.Subscriber(self.input_topic, Imu, self.callback, queue_size=50)
        rospy.logwarn(
            "IMU noise injector active: %s -> %s gyro=%.4f accel=%.3f seed=%d orientation_yaw_std=%.4f",
            self.input_topic, self.output_topic, self.gyro_std, self.accel_std,
            self.seed, self.orientation_yaw_std,
        )

    def callback(self, message: Imu) -> None:
        self.total += 1
        if self.rng.random() < self.dropout_probability:
            self.dropped += 1
            self.publish_status("dropout")
            return

        now = time.monotonic()
        dt = max(1e-3, min(now - self.last_time, 1.0))
        self.last_time = now
        for index in range(3):
            self.gyro_bias[index] += self.rng.gauss(0.0, self.gyro_bias_rw * math.sqrt(dt))
            self.accel_bias[index] += self.rng.gauss(0.0, self.accel_bias_rw * math.sqrt(dt))

        noisy = copy.deepcopy(message)
        gyro_values = [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z]
        accel_values = [message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z]
        for index in range(3):
            gyro_values[index] += self.gyro_bias[index] + self.rng.gauss(0.0, self.gyro_std)
            accel_values[index] += self.accel_bias[index] + self.rng.gauss(0.0, self.accel_std)
        noisy.angular_velocity.x, noisy.angular_velocity.y, noisy.angular_velocity.z = gyro_values
        noisy.linear_acceleration.x, noisy.linear_acceleration.y, noisy.linear_acceleration.z = accel_values
        noisy.angular_velocity_covariance[8] = self.gyro_std**2
        noisy.linear_acceleration_covariance[0] = self.accel_std**2

        if self.orientation_yaw_std > 0.0:
            yaw = quaternion_to_yaw(
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            )
            if yaw is not None:
                qx, qy, qz, qw = yaw_to_quaternion(yaw + self.rng.gauss(0.0, self.orientation_yaw_std))
                noisy.orientation.x, noisy.orientation.y = qx, qy
                noisy.orientation.z, noisy.orientation.w = qz, qw
                noisy.orientation_covariance[8] = self.orientation_yaw_std**2

        self.publisher.publish(noisy)
        self.publish_status("ok")

    def publish_status(self, state: str) -> None:
        self.status_pub.publish(
            String(data="state=%s total=%d dropped=%d bias_gyro_z=%.5f bias_accel_x=%.5f" % (
                state, self.total, self.dropped, self.gyro_bias[2], self.accel_bias[0]
            ))
        )


if __name__ == "__main__":
    try:
        ImuNoiseInjector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
