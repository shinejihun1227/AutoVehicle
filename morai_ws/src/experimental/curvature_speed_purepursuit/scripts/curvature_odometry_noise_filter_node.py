#!/usr/bin/env python3
"""Odometry noise injection and robust filtering for the isolated curvature test.

This node intentionally operates on Odometry rather than the existing GPS/IMU
topics. It is therefore a controller-input robustness test and does not modify
the current localization or control pipeline.
"""

from __future__ import annotations

import copy
import math
import time
from typing import Optional, Tuple

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from curvature_speed_purepursuit.noise_filter import (
    MotionState,
    OdometryNoiseModel,
    RobustOdometryFilter,
    wrap_angle,
)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def yaw_to_quaternion(yaw_rad: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def state_from_odometry(message: Odometry) -> MotionState:
    pose = message.pose.pose
    twist = message.twist.twist
    return MotionState(
        x=float(pose.position.x),
        y=float(pose.position.y),
        yaw=quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
        vx=float(twist.linear.x),
        vy=float(twist.linear.y),
    )


def odometry_with_state(source: Odometry, state: MotionState) -> Odometry:
    message = copy.deepcopy(source)
    message.pose.pose.position.x = state.x
    message.pose.pose.position.y = state.y
    qx, qy, qz, qw = yaw_to_quaternion(wrap_angle(state.yaw))
    message.pose.pose.orientation.x = qx
    message.pose.pose.orientation.y = qy
    message.pose.pose.orientation.z = qz
    message.pose.pose.orientation.w = qw
    message.twist.twist.linear.x = state.vx
    message.twist.twist.linear.y = state.vy
    return message


class CurvatureOdometryNoiseFilterNode:
    def __init__(self) -> None:
        rospy.init_node("curvature_odometry_noise_filter", anonymous=False)

        self.input_topic = rospy.get_param(
            "~input_topic", "/localization/odometry"
        )
        self.noisy_topic = rospy.get_param(
            "~noisy_topic", "/experimental/curvature_noisy_odometry"
        )
        self.filtered_topic = rospy.get_param(
            "~filtered_topic", "/experimental/curvature_filtered_odometry"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/experimental/curvature_noise_filter_status"
        )

        self.noise_model = OdometryNoiseModel(
            position_std_m=float(rospy.get_param("~position_noise_std_m", 0.25)),
            yaw_std_rad=float(rospy.get_param("~yaw_noise_std_rad", 0.03)),
            velocity_std_mps=float(
                rospy.get_param("~velocity_noise_std_mps", 0.05)
            ),
            position_bias_rw_m_sqrt_s=float(
                rospy.get_param("~position_bias_random_walk_m_sqrt_s", 0.02)
            ),
            yaw_bias_rw_rad_sqrt_s=float(
                rospy.get_param("~yaw_bias_random_walk_rad_sqrt_s", 0.002)
            ),
            velocity_bias_rw_mps_sqrt_s=float(
                rospy.get_param("~velocity_bias_random_walk_mps_sqrt_s", 0.01)
            ),
            seed=int(rospy.get_param("~random_seed", 20260901)),
        )
        self.dropout_probability = max(
            0.0, min(1.0, float(rospy.get_param("~dropout_probability", 0.02)))
        )
        self.filter = RobustOdometryFilter(
            median_window_size=int(rospy.get_param("~median_window_size", 3)),
            ema_alpha=float(rospy.get_param("~ema_alpha", 0.35)),
            max_position_jump_m=float(
                rospy.get_param("~max_position_jump_m", 5.0)
            ),
            max_measurement_speed_mps=float(
                rospy.get_param("~max_measurement_speed_mps", 50.0)
            ),
            max_yaw_jump_rad=float(
                rospy.get_param("~max_yaw_jump_rad", math.radians(45.0))
            ),
        )

        self.noisy_pub = rospy.Publisher(self.noisy_topic, Odometry, queue_size=10)
        self.filtered_pub = rospy.Publisher(
            self.filtered_topic, Odometry, queue_size=10
        )
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.last_callback_time: Optional[float] = None
        self.total_count = 0
        self.dropped_count = 0
        self.rejected_count = 0
        self.accepted_count = 0

        rospy.Subscriber(self.input_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.logwarn(
            "Curvature odometry noise/filter: input=%s noisy=%s filtered=%s "
            "dropout=%.3f seed=%d",
            self.input_topic,
            self.noisy_topic,
            self.filtered_topic,
            self.dropout_probability,
            self.noise_model.seed,
        )

    def publish_status(self, state: str) -> None:
        self.status_pub.publish(
            String(
                data=(
                    "%s total=%d dropped=%d rejected=%d accepted=%d"
                    % (
                        state,
                        self.total_count,
                        self.dropped_count,
                        self.rejected_count,
                        self.accepted_count,
                    )
                )
            )
        )

    def odom_callback(self, source: Odometry) -> None:
        now = time.monotonic()
        dt = (
            1.0 / 30.0
            if self.last_callback_time is None
            else now - self.last_callback_time
        )
        self.last_callback_time = now
        dt = max(1e-3, min(dt, 1.0))
        self.total_count += 1

        if self.noise_model.rng.random() < self.dropout_probability:
            self.dropped_count += 1
            self.publish_status("dropout")
            return

        noisy_state = self.noise_model.apply(state_from_odometry(source), dt)
        self.noisy_pub.publish(odometry_with_state(source, noisy_state))

        filtered_state = self.filter.update(noisy_state, dt)
        if filtered_state is None:
            self.rejected_count += 1
            self.publish_status("rejected_jump")
            return

        self.accepted_count += 1
        self.filtered_pub.publish(odometry_with_state(source, filtered_state))
        self.publish_status("accepted")


if __name__ == "__main__":
    try:
        CurvatureOdometryNoiseFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
