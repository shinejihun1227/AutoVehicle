#!/usr/bin/env python3
"""Noisy GPS Odometry의 급격한 jump를 제거하고 median으로 완화한다.

이 노드는 EKF를 대체하지 않는다. 정상 GPS 측정은 EKF로 보내고,
물리적으로 불가능한 jump는 publish하지 않아 EKF가 IMU prediction을 계속하게 한다.
"""

from __future__ import annotations

import copy
import math
import time
from collections import deque
from statistics import median

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class GpsRobustFilter:
    def __init__(self) -> None:
        rospy.init_node("gps_robust_filter", anonymous=False)
        self.input_topic = rospy.get_param("~input_topic", "/localization/gps_noisy")
        self.output_topic = rospy.get_param("~output_topic", "/localization/gps_filtered")
        self.max_jump_m = float(rospy.get_param("~max_position_jump_m", 5.0))
        self.max_speed_mps = float(rospy.get_param("~max_speed_mps", 15.0))
        self.window_size = max(1, int(rospy.get_param("~median_window_size", 3)))
        self.status_pub = rospy.Publisher("/stability/gps_filter_status", String, queue_size=1, latch=True)
        self.publisher = rospy.Publisher(self.output_topic, Odometry, queue_size=20)
        rospy.Subscriber(self.input_topic, Odometry, self.callback, queue_size=20)
        self.window_x = deque(maxlen=self.window_size)
        self.window_y = deque(maxlen=self.window_size)
        self.last_x = None
        self.last_y = None
        self.last_stamp = None
        self.accepted = 0
        self.rejected = 0
        rospy.loginfo(
            "GPS robust filter: %s -> %s max_jump=%.2fm max_speed=%.2fm/s window=%d",
            self.input_topic, self.output_topic, self.max_jump_m, self.max_speed_mps, self.window_size,
        )

    @staticmethod
    def message_time(message: Odometry) -> float:
        stamp = message.header.stamp.to_sec()
        return stamp if stamp > 0.0 else time.monotonic()

    def callback(self, message: Odometry) -> None:
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        stamp = self.message_time(message)

        if self.last_x is not None and self.last_y is not None and self.last_stamp is not None:
            dt = stamp - self.last_stamp
            distance = math.hypot(x - self.last_x, y - self.last_y)
            speed = distance / dt if dt > 1e-3 else float("inf")
            if distance > self.max_jump_m or speed > self.max_speed_mps:
                self.rejected += 1
                self.publish_status("reject_jump distance=%.3f speed=%.3f" % (distance, speed))
                return

        self.window_x.append(x)
        self.window_y.append(y)
        filtered_x = float(median(self.window_x))
        filtered_y = float(median(self.window_y))
        self.last_x = filtered_x
        self.last_y = filtered_y
        self.last_stamp = stamp
        self.accepted += 1

        output = copy.deepcopy(message)
        output.pose.pose.position.x = filtered_x
        output.pose.pose.position.y = filtered_y
        output.pose.covariance[0] = max(float(output.pose.covariance[0]), 1e-4)
        output.pose.covariance[7] = max(float(output.pose.covariance[7]), 1e-4)
        self.publisher.publish(output)
        self.publish_status("accept")

    def publish_status(self, state: str) -> None:
        self.status_pub.publish(
            String(data="state=%s accepted=%d rejected=%d" % (state, self.accepted, self.rejected))
        )


if __name__ == "__main__":
    try:
        GpsRobustFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
