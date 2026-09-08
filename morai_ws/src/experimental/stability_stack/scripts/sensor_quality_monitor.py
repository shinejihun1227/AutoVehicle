#!/usr/bin/env python3
"""원본 GPS·IMU 입력의 freshness와 GPS blackout 상태를 감시한다.

이 노드는 센서 값을 변형하거나 인위적인 노이즈를 생성하지 않는다.
GPS blackout과 입력 지연만 SensorQuality로 전달해 EKF와 camera fallback이
동일한 상태 정보를 사용하도록 한다.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Dict, Optional

import rospy
from morai_perception_msgs.msg import GpsHealth, SensorQuality
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String


NORMAL = "NORMAL"
GPS_BLACKOUT = "GPS_BLACKOUT"
SENSOR_DEGRADED = "SENSOR_DEGRADED"


class SensorQualityMonitor:
    def __init__(self) -> None:
        rospy.init_node("sensor_quality_monitor", anonymous=False)

        self.gps_topic = rospy.get_param("~gps_topic", "/localization/gps")
        self.imu_topic = rospy.get_param("~imu_topic", "/Imu")
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

        self.lock = threading.RLock()
        self.last_gps_time: Optional[float] = None
        self.last_imu_time: Optional[float] = None
        self.last_health: Optional[GpsHealth] = None
        self.last_health_time = 0.0

        self.quality_pub = rospy.Publisher(
            self.quality_topic, SensorQuality, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=1, latch=True
        )
        rospy.Subscriber(self.gps_topic, Odometry, self.gps_callback, queue_size=20)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=50)
        rospy.Subscriber(
            self.health_topic, GpsHealth, self.health_callback, queue_size=20
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz), self.publish_quality
        )

        rospy.loginfo(
            "Sensor freshness monitor gps=%s imu=%s health=%s output=%s",
            self.gps_topic,
            self.imu_topic,
            self.health_topic,
            self.quality_topic,
        )

    def gps_callback(self, _message: Odometry) -> None:
        with self.lock:
            self.last_gps_time = time.monotonic()

    def imu_callback(self, _message: Imu) -> None:
        with self.lock:
            self.last_imu_time = time.monotonic()

    def health_callback(self, message: GpsHealth) -> None:
        with self.lock:
            self.last_health = message
            self.last_health_time = time.monotonic()

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
                math.inf
                if self.last_health_time <= 0.0
                else now - self.last_health_time
            )

            health_fresh = (
                self.last_health is not None
                and health_age <= self.sensor_timeout_sec
            )
            if health_fresh:
                gps_valid = bool(self.last_health.valid)
                gps_blackout = bool(self.last_health.blackout)
                gps_recovering = self.last_health.state == "GPS_RECOVERING"
                reason = str(self.last_health.reason)
            else:
                gps_valid = False
                gps_blackout = False
                gps_recovering = False
                reason = "gps_health_stale"

            gps_stale = gps_age > self.sensor_timeout_sec
            imu_stale = imu_age > self.sensor_timeout_sec
            if gps_blackout:
                state = GPS_BLACKOUT
                confidence = 0.0
            elif not health_fresh or gps_stale or imu_stale or gps_recovering:
                state = SENSOR_DEGRADED
                confidence = 0.2
                if health_fresh and gps_recovering:
                    reason = "gps_recovering"
                elif gps_stale:
                    reason = "gps_stale"
                elif imu_stale:
                    reason = "imu_stale"
            else:
                state = NORMAL
                confidence = 1.0
                reason = "sensors_fresh"

            return {
                "state": state,
                "gps_valid": gps_valid,
                "gps_blackout": gps_blackout,
                "gps_recovering": gps_recovering,
                "imu_stale": imu_stale,
                "confidence": confidence,
                "reason": reason,
                "gps_age_sec": gps_age,
                "imu_age_sec": imu_age,
                "gps_health_age_sec": health_age,
            }

    def publish_quality(self, _event) -> None:
        data = self.snapshot()
        message = SensorQuality()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "base_link"
        message.state = str(data["state"])
        message.gps_valid = bool(data["gps_valid"])
        message.gps_blackout = bool(data["gps_blackout"])
        message.gps_recovering = bool(data["gps_recovering"])
        message.imu_stale = bool(data["imu_stale"])
        message.confidence = float(data["confidence"])
        message.reason = str(data["reason"])
        self.quality_pub.publish(message)

        json_data = dict(data)
        for key in ("gps_age_sec", "imu_age_sec", "gps_health_age_sec"):
            if not math.isfinite(float(json_data[key])):
                json_data[key] = None
        self.status_pub.publish(
            String(data=json.dumps(json_data, ensure_ascii=False, sort_keys=True))
        )
        rospy.loginfo_throttle(
            2.0,
            "sensor freshness state=%s reason=%s gps_age=%.2f imu_age=%.2f",
            data["state"],
            data["reason"],
            float(data["gps_age_sec"]),
            float(data["imu_age_sec"]),
        )


if __name__ == "__main__":
    try:
        SensorQualityMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
