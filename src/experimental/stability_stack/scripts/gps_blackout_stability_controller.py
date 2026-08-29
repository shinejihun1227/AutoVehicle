#!/usr/bin/env python3
"""GPS blackout 중 Pure Pursuit 명령을 보수적으로 제한한다.

GPS blackout 자체가 장애물을 의미하지는 않으므로 기본 동작은 즉시 정지가
아니라 저속 제한이다. EKF는 IMU prediction을 계속 사용하고, 이 노드는 명령의
속도·조향 변화를 제한하여 localization 품질 저하가 제어기에 급격한 충격으로
전달되지 않도록 한다.
"""

from __future__ import annotations

import copy
import json
import math
import time
from typing import Optional

import rospy
from morai_msgs.msg import CtrlCmd
from morai_perception_msgs.msg import GpsHealth
from std_msgs.msg import String


GPS_OK = "GPS_OK"
GPS_BLACKOUT = "GPS_BLACKOUT"
GPS_RECOVERING = "GPS_RECOVERING"


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class GpsBlackoutStabilityController:
    def __init__(self) -> None:
        rospy.init_node("gps_blackout_stability_controller", anonymous=False)

        self.input_topic = rospy.get_param(
            "~input_command_topic", "/control/camera_stable_cmd"
        )
        self.output_topic = rospy.get_param(
            "~output_command_topic", "/control/blackout_stable_cmd"
        )
        self.health_topic = rospy.get_param(
            "~gps_health_topic", "/localization/gps_health"
        )
        self.enabled = bool(rospy.get_param("~enabled", True))
        self.rate_hz = max(1.0, float(rospy.get_param("~rate_hz", 20.0)))
        self.command_timeout = max(
            0.05, float(rospy.get_param("~command_timeout_sec", 0.5))
        )
        self.health_timeout = max(
            0.05, float(rospy.get_param("~health_timeout_sec", 0.5))
        )
        self.blackout_speed = max(
            0.0, float(rospy.get_param("~blackout_speed_mps", 1.0))
        )
        self.recovering_speed = max(
            0.0, float(rospy.get_param("~recovering_speed_mps", 1.5))
        )
        self.stop_on_blackout = bool(rospy.get_param("~stop_on_blackout", False))
        self.stop_on_health_stale = bool(
            rospy.get_param("~stop_on_health_stale", True)
        )
        self.max_steering_rad = float(
            rospy.get_param("~max_steering_rad", math.radians(40.0))
        )
        self.max_steering_rate = max(
            0.0, float(rospy.get_param("~max_steering_rate_rad_s", 0.5))
        )
        self.max_velocity_rate = max(
            0.0, float(rospy.get_param("~max_velocity_rate_mps2", 1.0))
        )

        self.last_command: Optional[CtrlCmd] = None
        self.last_health: Optional[GpsHealth] = None
        self.last_command_time = 0.0
        self.last_health_time = 0.0
        self.last_output_time = time.monotonic()
        self.last_output_steering = 0.0
        self.last_output_velocity = 0.0

        self.command_pub = rospy.Publisher(self.output_topic, CtrlCmd, queue_size=1)
        self.status_pub = rospy.Publisher(
            "/stability/gps_blackout_control_status",
            String,
            queue_size=1,
            latch=True,
        )
        rospy.Subscriber(self.input_topic, CtrlCmd, self.command_callback, queue_size=10)
        rospy.Subscriber(self.health_topic, GpsHealth, self.health_callback, queue_size=10)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.rate_hz), self.publish_command
        )

        rospy.loginfo(
            "GPS blackout stability: enabled=%s input=%s output=%s blackout_speed=%.2f recovering_speed=%.2f",
            self.enabled,
            self.input_topic,
            self.output_topic,
            self.blackout_speed,
            self.recovering_speed,
        )

    def command_callback(self, message: CtrlCmd) -> None:
        self.last_command = copy.deepcopy(message)
        self.last_command_time = time.monotonic()

    def health_callback(self, message: GpsHealth) -> None:
        self.last_health = message
        self.last_health_time = time.monotonic()

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

    def limit_steering(self, command: CtrlCmd, dt: float) -> None:
        if not hasattr(command, "steering"):
            return
        target = clamp(float(command.steering), -self.max_steering_rad, self.max_steering_rad)
        if self.max_steering_rate > 0.0:
            max_delta = self.max_steering_rate * dt
            target = clamp(
                target,
                self.last_output_steering - max_delta,
                self.last_output_steering + max_delta,
            )
        command.steering = target
        self.last_output_steering = target

    def limit_velocity(self, command: CtrlCmd, cap: Optional[float], dt: float) -> None:
        if not hasattr(command, "velocity"):
            return
        target = max(0.0, float(command.velocity))
        if cap is not None:
            target = min(target, cap)
        if self.max_velocity_rate > 0.0:
            max_delta = self.max_velocity_rate * dt
            target = clamp(
                target,
                self.last_output_velocity - max_delta,
                self.last_output_velocity + max_delta,
            )
        command.velocity = target
        self.last_output_velocity = target

    def publish_command(self, _event) -> None:
        now = time.monotonic()
        if self.last_command is None or now - self.last_command_time > self.command_timeout:
            self.publish_status("command_stale")
            return

        dt = max(1e-3, now - self.last_output_time)
        self.last_output_time = now

        if not self.enabled:
            output = copy.deepcopy(self.last_command)
            self.command_pub.publish(output)
            self.publish_status("disabled_pass_through")
            return

        health_fresh = (
            self.last_health is not None
            and now - self.last_health_time <= self.health_timeout
        )
        if not health_fresh:
            if self.stop_on_health_stale:
                self.command_pub.publish(self.stop_command())
                self.publish_status("gps_health_stale_stop")
            else:
                output = copy.deepcopy(self.last_command)
                self.limit_steering(output, dt)
                self.limit_velocity(output, self.blackout_speed, dt)
                self.command_pub.publish(output)
                self.publish_status("gps_health_stale_degraded")
            return

        state = self.last_health.state
        if state == GPS_BLACKOUT:
            if self.stop_on_blackout:
                self.command_pub.publish(self.stop_command())
                self.publish_status("gps_blackout_stop")
                return
            speed_cap = self.blackout_speed
            mode = "gps_blackout_degraded"
        elif state == GPS_RECOVERING:
            speed_cap = self.recovering_speed
            mode = "gps_recovering_degraded"
        elif state == GPS_OK:
            speed_cap = None
            mode = "gps_ok"
        else:
            self.command_pub.publish(self.stop_command())
            self.publish_status("unknown_gps_state_stop")
            return

        output = copy.deepcopy(self.last_command)
        self.limit_steering(output, dt)
        self.limit_velocity(output, speed_cap, dt)
        self.command_pub.publish(output)
        self.publish_status(mode)

    def publish_status(self, mode: str) -> None:
        state = self.last_health.state if self.last_health is not None else "UNKNOWN"
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "mode": mode,
                        "gps_state": state,
                        "enabled": self.enabled,
                        "blackout_speed_mps": self.blackout_speed,
                        "recovering_speed_mps": self.recovering_speed,
                    },
                    ensure_ascii=False,
                )
            )
        )


if __name__ == "__main__":
    try:
        GpsBlackoutStabilityController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
