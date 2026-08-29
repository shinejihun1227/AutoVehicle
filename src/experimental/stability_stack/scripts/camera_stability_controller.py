#!/usr/bin/env python3
"""전방 카메라 차선 결과로 Pure Pursuit 조향을 선택적으로 보정한다.

지도 기반 Pure Pursuit를 대체하지 않고, lane confidence가 충분할 때만 작은
보정값을 더한다. camera calibration과 sign 검증 전에는 enabled=false를 유지한다.
"""

from __future__ import annotations

import copy
import json
import math
import time
from typing import Optional

import rospy
from morai_msgs.msg import CtrlCmd
from morai_perception_msgs.msg import LaneDetection
from std_msgs.msg import String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class CameraStabilityController:
    def __init__(self) -> None:
        rospy.init_node("camera_stability_controller", anonymous=False)
        self.nominal_topic = rospy.get_param("~nominal_command_topic", "/control/ctrl_cmd")
        self.output_topic = rospy.get_param("~output_command_topic", "/control/camera_stable_cmd")
        self.lane_topic = rospy.get_param("~lane_topic", "/detection/lane")
        self.enabled = bool(rospy.get_param("~enabled", False))
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.nominal_timeout = float(rospy.get_param("~nominal_timeout_sec", 0.5))
        self.lane_timeout = float(rospy.get_param("~lane_timeout_sec", 0.3))
        self.min_confidence = float(rospy.get_param("~min_lane_confidence", 0.4))
        self.lateral_gain = float(rospy.get_param("~lateral_gain", 0.15))
        self.heading_gain = float(rospy.get_param("~heading_gain", 0.35))
        self.lateral_sign = float(rospy.get_param("~lateral_sign", 1.0))
        self.heading_sign = float(rospy.get_param("~heading_sign", 1.0))
        self.preview_distance = float(rospy.get_param("~preview_distance_m", 4.0))
        self.max_steering_rad = float(rospy.get_param("~max_steering_rad", math.radians(40.0)))
        self.max_steering_rate = float(rospy.get_param("~max_steering_rate_rad_s", 0.8))
        self.pass_through_on_stale_lane = bool(rospy.get_param("~pass_through_on_stale_lane", True))

        self.last_nominal: Optional[CtrlCmd] = None
        self.last_lane: Optional[LaneDetection] = None
        self.last_nominal_time = 0.0
        self.last_lane_time = 0.0
        self.last_output_steering = 0.0
        self.last_output_time = time.monotonic()
        self.publisher = rospy.Publisher(self.output_topic, CtrlCmd, queue_size=1)
        self.status_pub = rospy.Publisher("/stability/camera_status", String, queue_size=1, latch=True)
        rospy.Subscriber(self.nominal_topic, CtrlCmd, self.nominal_callback, queue_size=10)
        rospy.Subscriber(self.lane_topic, LaneDetection, self.lane_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self.publish)
        rospy.logwarn(
            "camera stability controller enabled=%s nominal=%s output=%s",
            self.enabled, self.nominal_topic, self.output_topic,
        )

    def nominal_callback(self, message: CtrlCmd) -> None:
        self.last_nominal = copy.deepcopy(message)
        self.last_nominal_time = time.monotonic()

    def lane_callback(self, message: LaneDetection) -> None:
        self.last_lane = message
        self.last_lane_time = time.monotonic()

    def publish(self, _event) -> None:
        now = time.monotonic()
        if self.last_nominal is None or now - self.last_nominal_time > self.nominal_timeout:
            self.publish_status("nominal_stale")
            return

        output = copy.deepcopy(self.last_nominal)
        if not self.enabled:
            self.publisher.publish(output)
            self.publish_status("disabled_pass_through")
            return

        correction = 0.0
        lane_used = False
        lane_fresh = self.last_lane is not None and now - self.last_lane_time <= self.lane_timeout
        if self.enabled and lane_fresh and self.last_lane.valid and self.last_lane.confidence >= self.min_confidence:
            lateral_term = self.lateral_sign * self.lateral_gain * float(self.last_lane.lateral_offset_m)
            heading_term = self.heading_sign * self.heading_gain * float(self.last_lane.heading_error_rad)
            correction = lateral_term + heading_term
            lane_used = True
        elif self.enabled and not self.pass_through_on_stale_lane and not lane_fresh:
            output = self.stop_command()

        if hasattr(output, "steering"):
            target = clamp(float(output.steering) + correction, -self.max_steering_rad, self.max_steering_rad)
            dt = max(1e-3, now - self.last_output_time)
            max_delta = self.max_steering_rate * dt
            output.steering = clamp(target, self.last_output_steering - max_delta, self.last_output_steering + max_delta)
            self.last_output_steering = float(output.steering)
        self.last_output_time = now
        self.publisher.publish(output)
        self.publish_status("lane_correction" if lane_used else "pass_through")

    def stop_command(self) -> CtrlCmd:
        output = CtrlCmd()
        if hasattr(output, "longlCmdType"):
            output.longlCmdType = 2
        if hasattr(output, "steering"):
            output.steering = 0.0
        if hasattr(output, "velocity"):
            output.velocity = 0.0
        if hasattr(output, "accel"):
            output.accel = 0.0
        if hasattr(output, "acceleration"):
            output.acceleration = 0.0
        if hasattr(output, "brake"):
            output.brake = 1.0
        return output

    def publish_status(self, mode: str) -> None:
        self.status_pub.publish(String(data=json.dumps({"mode": mode, "enabled": self.enabled})))


if __name__ == "__main__":
    try:
        CameraStabilityController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
