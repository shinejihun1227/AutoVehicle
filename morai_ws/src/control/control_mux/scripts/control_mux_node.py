#!/usr/bin/env python3
"""최종 MORAI 제어 명령 중재기.

Pure Pursuit는 /control/ctrl_cmd에 정상 주행 명령을 발행하고,
이 노드는 안전정지·신호등·신선도 조건을 검사한 뒤 /ctrl_cmd만 발행한다.
"""

from __future__ import annotations

import copy
import json
import math
import time
from typing import Optional

import rospy
from morai_msgs.msg import CtrlCmd
from morai_perception_msgs.msg import LaneDetection, SafetyStop, TrafficLight
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class ControlMux:
    def __init__(self) -> None:
        rospy.init_node("control_mux", anonymous=False)

        self.nominal_topic = rospy.get_param("~nominal_command_topic", "/control/ctrl_cmd")
        self.output_topic = rospy.get_param("~output_command_topic", "/ctrl_cmd")
        self.safety_topic = rospy.get_param("~safety_stop_topic", "/detection/fused_safety_stop")
        self.traffic_topic = rospy.get_param("~traffic_light_topic", "/detection/traffic_light")
        self.lane_topic = rospy.get_param("~lane_topic", "/detection/lane")
        self.localization_topic = rospy.get_param("~localization_topic", "/localization/odometry")

        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.nominal_timeout = float(rospy.get_param("~nominal_timeout_sec", 0.5))
        self.safety_timeout = float(rospy.get_param("~safety_timeout_sec", 0.5))
        self.traffic_timeout = float(rospy.get_param("~traffic_timeout_sec", 1.0))
        self.localization_timeout = float(rospy.get_param("~localization_timeout_sec", 0.5))
        self.require_fresh_safety = bool(rospy.get_param("~require_fresh_safety", True))
        self.require_fresh_localization = bool(rospy.get_param("~require_fresh_localization", True))
        self.stop_on_red = bool(rospy.get_param("~stop_on_red", True))
        self.stop_on_unknown_traffic = bool(rospy.get_param("~stop_on_unknown_traffic", False))
        self.require_fresh_traffic = bool(rospy.get_param("~require_fresh_traffic", False))
        self.lane_correction_enabled = bool(rospy.get_param("~lane_correction_enabled", False))
        self.lane_correction_gain = float(rospy.get_param("~lane_correction_gain", 0.0))
        self.lane_preview_distance = float(rospy.get_param("~lane_preview_distance_m", 4.0))
        self.max_steering_rad = float(rospy.get_param("~max_steering_rad", math.radians(40.0)))

        self.last_nominal: Optional[CtrlCmd] = None
        self.last_safety: Optional[SafetyStop] = None
        self.last_traffic: Optional[TrafficLight] = None
        self.last_lane: Optional[LaneDetection] = None
        self.last_nominal_time = 0.0
        self.last_safety_time = 0.0
        self.last_traffic_time = 0.0
        self.last_lane_time = 0.0
        self.last_localization_time = 0.0

        self.command_pub = rospy.Publisher(self.output_topic, CtrlCmd, queue_size=1)
        self.status_pub = rospy.Publisher("/control/mux_status", String, queue_size=1, latch=True)
        rospy.Subscriber(self.nominal_topic, CtrlCmd, self.nominal_callback, queue_size=10)
        rospy.Subscriber(self.safety_topic, SafetyStop, self.safety_callback, queue_size=10)
        rospy.Subscriber(self.traffic_topic, TrafficLight, self.traffic_callback, queue_size=10)
        rospy.Subscriber(self.lane_topic, LaneDetection, self.lane_callback, queue_size=10)
        rospy.Subscriber(self.localization_topic, Odometry, self.localization_callback, queue_size=10)

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self.publish_command
        )
        rospy.loginfo(
            "control_mux nominal=%s output=%s safety=%s require_fresh_safety=%s",
            self.nominal_topic,
            self.output_topic,
            self.safety_topic,
            self.require_fresh_safety,
        )

    def nominal_callback(self, msg: CtrlCmd) -> None:
        self.last_nominal = copy.deepcopy(msg)
        self.last_nominal_time = time.monotonic()

    def safety_callback(self, msg: SafetyStop) -> None:
        self.last_safety = msg
        self.last_safety_time = time.monotonic()

    def traffic_callback(self, msg: TrafficLight) -> None:
        self.last_traffic = msg
        self.last_traffic_time = time.monotonic()

    def lane_callback(self, msg: LaneDetection) -> None:
        self.last_lane = msg
        self.last_lane_time = time.monotonic()

    def localization_callback(self, _msg: Odometry) -> None:
        self.last_localization_time = time.monotonic()

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

    def apply_lane_correction(self, command: CtrlCmd) -> CtrlCmd:
        if not self.lane_correction_enabled or self.last_lane is None:
            return command
        if not self.last_lane.valid or self.last_lane.confidence <= 0.0:
            return command
        if time.monotonic() - self.last_lane_time > self.traffic_timeout:
            return command

        heading_error = float(self.last_lane.heading_error_rad)
        lateral_error = float(self.last_lane.lateral_offset_m)
        correction = self.lane_correction_gain * (
            heading_error + math.atan2(lateral_error, max(self.lane_preview_distance, 0.1))
        )
        if hasattr(command, "steering"):
            command.steering = clamp(
                float(command.steering) + correction,
                -self.max_steering_rad,
                self.max_steering_rad,
            )
        return command

    def publish_command(self, _event) -> None:
        now = time.monotonic()
        reasons = []

        if self.last_nominal is None or now - self.last_nominal_time > self.nominal_timeout:
            reasons.append("nominal_command_stale")

        if self.require_fresh_localization and (
            self.last_localization_time <= 0.0
            or now - self.last_localization_time > self.localization_timeout
        ):
            reasons.append("localization_stale")

        if self.require_fresh_safety and (
            self.last_safety is None or now - self.last_safety_time > self.safety_timeout
        ):
            reasons.append("safety_stop_stale")
        elif self.last_safety is not None and self.last_safety.stop_required:
            reasons.append(self.last_safety.reason or "obstacle_stop")

        if self.require_fresh_traffic and (
            self.last_traffic is None or now - self.last_traffic_time > self.traffic_timeout
        ):
            reasons.append("traffic_light_stale")
        elif self.last_traffic is not None and now - self.last_traffic_time <= self.traffic_timeout:
            state = self.last_traffic.state.lower()
            if self.stop_on_red and state == "red":
                reasons.append("red_light")
            if self.stop_on_unknown_traffic and state == "unknown":
                reasons.append("traffic_light_unknown")

        if reasons:
            command = self.stop_command()
            mode = "stop"
        else:
            command = self.apply_lane_correction(copy.deepcopy(self.last_nominal))
            mode = "nominal"

        self.command_pub.publish(command)
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {"mode": mode, "reasons": reasons}, ensure_ascii=False
                )
            )
        )
        rospy.loginfo_throttle(2.0, "control_mux mode=%s reasons=%s", mode, reasons)


if __name__ == "__main__":
    try:
        ControlMux()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
