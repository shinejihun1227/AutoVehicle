#!/usr/bin/env python3
"""GPS Odometry에 재현 가능한 noise·bias·outlier·dropout을 추가한다.

기본 입력은 /localization/gps이며, 출력은 /localization/gps_noisy이다.
원본 토픽은 변경하지 않는다.
"""

from __future__ import annotations

import copy
import math
import random
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class GpsNoiseInjector:
    def __init__(self) -> None:
        rospy.init_node("gps_noise_injector", anonymous=False)
        self.input_topic = rospy.get_param("~input_topic", "/localization/gps")
        self.output_topic = rospy.get_param("~output_topic", "/localization/gps_noisy")
        self.std_x = float(rospy.get_param("~position_noise_std_x_m", 0.5))
        self.std_y = float(rospy.get_param("~position_noise_std_y_m", 0.5))
        self.bias_rw_x = float(rospy.get_param("~bias_random_walk_x_m_sqrt_s", 0.02))
        self.bias_rw_y = float(rospy.get_param("~bias_random_walk_y_m_sqrt_s", 0.02))
        self.outlier_probability = float(rospy.get_param("~outlier_probability", 0.0))
        self.outlier_std = float(rospy.get_param("~outlier_std_m", 3.0))
        self.dropout_probability = float(rospy.get_param("~dropout_probability", 0.0))
        self.seed = int(rospy.get_param("~random_seed", 20260822))
        self.rng = random.Random(self.seed)
        self.bias_x = 0.0
        self.bias_y = 0.0
        self.last_time = time.monotonic()
        self.total = 0
        self.dropped = 0
        self.outliers = 0

        self.publisher = rospy.Publisher(self.output_topic, Odometry, queue_size=20)
        self.status_pub = rospy.Publisher("/stability/gps_noise_status", String, queue_size=1, latch=True)
        rospy.Subscriber(self.input_topic, Odometry, self.callback, queue_size=20)
        rospy.logwarn(
            "GPS noise injector active: %s -> %s std=(%.2f, %.2f)m seed=%d",
            self.input_topic, self.output_topic, self.std_x, self.std_y, self.seed,
        )

    def callback(self, message: Odometry) -> None:
        self.total += 1
        if self.rng.random() < self.dropout_probability:
            self.dropped += 1
            self.publish_status("dropout")
            return

        now = time.monotonic()
        dt = max(1e-3, min(now - self.last_time, 1.0))
        self.last_time = now
        self.bias_x += self.rng.gauss(0.0, self.bias_rw_x * math.sqrt(dt))
        self.bias_y += self.rng.gauss(0.0, self.bias_rw_y * math.sqrt(dt))

        outlier_x = 0.0
        outlier_y = 0.0
        if self.rng.random() < self.outlier_probability:
            outlier_x = self.rng.gauss(0.0, self.outlier_std)
            outlier_y = self.rng.gauss(0.0, self.outlier_std)
            self.outliers += 1

        noisy = copy.deepcopy(message)
        noisy.pose.pose.position.x += self.bias_x + self.rng.gauss(0.0, self.std_x) + outlier_x
        noisy.pose.pose.position.y += self.bias_y + self.rng.gauss(0.0, self.std_y) + outlier_y
        noisy.pose.covariance[0] = max(float(noisy.pose.covariance[0]), self.std_x**2)
        noisy.pose.covariance[7] = max(float(noisy.pose.covariance[7]), self.std_y**2)
        self.publisher.publish(noisy)
        self.publish_status("outlier" if outlier_x or outlier_y else "ok")

    def publish_status(self, state: str) -> None:
        self.status_pub.publish(
            String(
                data=(
                    "state=%s total=%d dropped=%d outliers=%d bias=(%.3f,%.3f)"
                    % (state, self.total, self.dropped, self.outliers, self.bias_x, self.bias_y)
                )
            )
        )


if __name__ == "__main__":
    try:
        GpsNoiseInjector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
