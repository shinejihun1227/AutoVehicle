#!/usr/bin/env python3
"""MORAI GPS 수신 상태를 감시하고 blackout/recovery 상태를 발행한다.

원본 /gps를 감시한다. GPS 패킷이 일정 시간 들어오지 않거나 status가 유효하지
않으면 GPS_BLACKOUT으로 전환한다. 유효한 GPS가 다시 들어오면 여러 샘플을 확인한
뒤 GPS_OK로 전환하여 복구 순간의 단일 오측정을 정상 상태로 오인하지 않는다.
"""

from __future__ import annotations

import json
import time

import rospy
from morai_msgs.msg import GPSMessage
from morai_perception_msgs.msg import GpsHealth
from std_msgs.msg import String


GPS_OK = "GPS_OK"
GPS_BLACKOUT = "GPS_BLACKOUT"
GPS_RECOVERING = "GPS_RECOVERING"


class GpsBlackoutDetector:
    def __init__(self) -> None:
        rospy.init_node("gps_blackout_detector", anonymous=False)

        self.input_topic = rospy.get_param("~input_topic", "/gps")
        self.health_topic = rospy.get_param("~health_topic", "/localization/gps_health")
        self.status_topic = rospy.get_param(
            "~status_topic", "/stability/gps_blackout_status"
        )
        self.blackout_timeout = max(
            0.1, float(rospy.get_param("~blackout_timeout_sec", 0.5))
        )
        self.recovery_valid_samples = max(
            1, int(rospy.get_param("~recovery_valid_samples", 5))
        )
        self.publish_rate_hz = max(
            1.0, float(rospy.get_param("~publish_rate_hz", 10.0))
        )

        self.state = GPS_BLACKOUT
        self.reason = "no_valid_gps_yet"
        self.last_valid_receive_time = None
        self.last_packet_receive_time = None
        self.last_status = 0
        self.consecutive_valid = 0
        self.invalid_count = 0
        self.total_packets = 0

        self.health_pub = rospy.Publisher(
            self.health_topic, GpsHealth, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=1, latch=True
        )
        rospy.Subscriber(self.input_topic, GPSMessage, self.gps_callback, queue_size=20)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz), self.timer_callback
        )

        rospy.loginfo(
            "GPS blackout detector: input=%s health=%s timeout=%.2fs recovery_samples=%d",
            self.input_topic,
            self.health_topic,
            self.blackout_timeout,
            self.recovery_valid_samples,
        )
        self.publish_health()

    def set_state(self, state: str, reason: str) -> None:
        changed = self.state != state or self.reason != reason
        self.state = state
        self.reason = reason
        if changed:
            rospy.logwarn(
                "GPS health state=%s reason=%s consecutive_valid=%d",
                self.state,
                self.reason,
                self.consecutive_valid,
            )

    def gps_callback(self, message: GPSMessage) -> None:
        now = time.monotonic()
        self.total_packets += 1
        self.last_packet_receive_time = now
        self.last_status = int(message.status)

        if self.last_status <= 0:
            self.invalid_count += 1
            self.consecutive_valid = 0
            self.set_state(GPS_BLACKOUT, "invalid_gps_status")
            self.publish_health()
            return

        self.last_valid_receive_time = now
        self.consecutive_valid += 1

        if self.state == GPS_BLACKOUT:
            self.set_state(GPS_RECOVERING, "valid_gps_recovered")
        if self.state == GPS_RECOVERING:
            if self.consecutive_valid >= self.recovery_valid_samples:
                self.set_state(GPS_OK, "recovery_stable")
        elif self.state == GPS_OK:
            self.reason = "valid_gps"

        self.publish_health()

    def timer_callback(self, _event) -> None:
        if self.last_valid_receive_time is None:
            self.consecutive_valid = 0
            self.set_state(GPS_BLACKOUT, "no_valid_gps_yet")
        else:
            age = time.monotonic() - self.last_valid_receive_time
            if age > self.blackout_timeout:
                self.consecutive_valid = 0
                self.set_state(GPS_BLACKOUT, "gps_timeout")

        self.publish_health()

    def publish_health(self) -> None:
        now = time.monotonic()
        if self.last_valid_receive_time is None:
            age_sec = -1.0
            valid = False
        else:
            age_sec = max(0.0, now - self.last_valid_receive_time)
            valid = age_sec <= self.blackout_timeout and self.last_status > 0

        health = GpsHealth()
        health.header.stamp = rospy.Time.now()
        health.header.frame_id = "gps_link"
        health.state = self.state
        health.valid = valid
        health.blackout = self.state == GPS_BLACKOUT
        health.age_sec = age_sec
        health.consecutive_valid = self.consecutive_valid
        health.invalid_count = self.invalid_count
        health.reason = self.reason
        self.health_pub.publish(health)

        status = {
            "state": self.state,
            "valid": valid,
            "age_sec": round(age_sec, 3),
            "consecutive_valid": self.consecutive_valid,
            "invalid_count": self.invalid_count,
            "total_packets": self.total_packets,
            "reason": self.reason,
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))


if __name__ == "__main__":
    try:
        GpsBlackoutDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
