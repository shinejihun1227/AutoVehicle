#!/usr/bin/env python3
"""정지선 접근을 accel/brake(type 1) 명령에 반영하는 제어 래퍼.

이 노드는 Pure Pursuit가 만든 nominal CtrlCmd를 직접 대체하지 않고 감싼다.
신호 정지 요청과 타임스탬프가 있는 정지선 검출이 동시에 유효할 때만
정지선까지 감속하도록 accel/brake를 조정하고, steering은 nominal 값을 유지한다.

정지선 검출이 없을 때는 기존 nominal 주행을 통과한다. 단, nominal 명령 자체가
오래되면 기존 최종 제어 경로와 동일하게 fail-safe 정지를 출력한다.
"""

from __future__ import annotations

import copy
import json
import math
import time
from dataclasses import dataclass
from typing import Optional

import rospy
from morai_msgs.msg import CtrlCmd
from morai_perception_msgs.msg import StopLineDetection, TrafficLight
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


MPS_TO_KPH = 3.6


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def finite(value: float) -> bool:
    return math.isfinite(float(value))


@dataclass
class StopLineDecision:
    mode: str
    reason: str
    accel: float = 0.0
    brake: float = 0.0
    desired_speed_mps: Optional[float] = None


class StopLineControllerCore:
    """ROS와 분리해 테스트 가능한 정지선 감속 계산부."""

    def __init__(
        self,
        max_decel_mps2: float = 1.5,
        reaction_time_sec: float = 0.3,
        hold_distance_m: float = 0.5,
        approach_distance_m: float = 20.0,
        trigger_margin_m: float = 1.0,
        brake_response_time_sec: float = 0.5,
        hold_speed_mps: float = 0.15,
        min_confidence: float = 0.5,
    ) -> None:
        self.max_decel_mps2 = max(0.1, float(max_decel_mps2))
        self.reaction_time_sec = max(0.0, float(reaction_time_sec))
        self.hold_distance_m = max(0.0, float(hold_distance_m))
        self.approach_distance_m = max(0.1, float(approach_distance_m))
        self.trigger_margin_m = max(0.0, float(trigger_margin_m))
        self.brake_response_time_sec = max(0.05, float(brake_response_time_sec))
        self.hold_speed_mps = max(0.0, float(hold_speed_mps))
        self.min_confidence = clamp(float(min_confidence), 0.0, 1.0)

    def decide(
        self,
        *,
        signal_stop_required: bool,
        stopline_valid: bool,
        stopline_distance_m: float,
        stopline_confidence: float,
        speed_mps: Optional[float],
    ) -> StopLineDecision:
        if not signal_stop_required:
            return StopLineDecision("nominal", "signal_clear")
        if not stopline_valid or not finite(stopline_distance_m):
            return StopLineDecision("nominal", "stopline_unavailable")
        distance = max(0.0, float(stopline_distance_m))
        confidence = clamp(float(stopline_confidence), 0.0, 1.0)
        if confidence < self.min_confidence:
            return StopLineDecision("nominal", "stopline_confidence_low")
        if distance > self.approach_distance_m + self.trigger_margin_m:
            return StopLineDecision("nominal", "stopline_outside_approach")

        # 차량 속도를 모르면 정지선 앞에서 보수적으로 정지한다. 실제 속도는
        # /localization/odometry에서 m/s로 받아 계산한다.
        if speed_mps is None or not finite(speed_mps):
            return StopLineDecision("stop", "speed_unavailable", accel=0.0, brake=1.0)
        speed = max(0.0, float(speed_mps))
        remaining = max(distance - self.hold_distance_m, 0.05)
        desired_speed = math.sqrt(2.0 * self.max_decel_mps2 * remaining)
        stopping_distance = (
            speed * self.reaction_time_sec
            + speed * speed / (2.0 * self.max_decel_mps2)
        )

        if distance <= self.hold_distance_m:
            return StopLineDecision(
                "hold_stop",
                "at_stopline",
                accel=0.0,
                brake=1.0,
                desired_speed_mps=0.0,
            )
        if speed <= self.hold_speed_mps and distance <= self.hold_distance_m + 0.5:
            return StopLineDecision(
                "hold_stop",
                "low_speed_near_stopline",
                accel=0.0,
                brake=1.0,
                desired_speed_mps=0.0,
            )

        # 정지선까지 남은 거리로 허용 속도를 만들고, 현재 속도가 이를 넘을
        # 때만 brake를 건다. 멀리 있을 때는 accel=0으로 더 가속하지 않는다.
        if speed > desired_speed:
            required_decel = (speed - desired_speed) / self.brake_response_time_sec
            brake = clamp(required_decel / self.max_decel_mps2, 0.0, 1.0)
        else:
            brake = 0.0
        if distance <= stopping_distance + self.trigger_margin_m:
            brake = max(brake, clamp(
                (stopping_distance - max(distance - self.trigger_margin_m, 0.0))
                / max(stopping_distance, 1e-3),
                0.0,
                1.0,
            ))
        return StopLineDecision(
            "approach_stopline",
            "red_or_stop_signal",
            accel=0.0,
            brake=brake,
            desired_speed_mps=desired_speed,
        )


class StopLineController:
    def __init__(self) -> None:
        rospy.init_node("stopline_controller", anonymous=False)
        self.enabled = bool(rospy.get_param("~enabled", True))
        self.nominal_topic = rospy.get_param("~nominal_command_topic", "/control/ctrl_cmd")
        self.output_topic = rospy.get_param("~output_command_topic", "/control/stopline_cmd")
        self.stopline_topic = rospy.get_param("~stopline_topic", "/perception/camera/stopline")
        self.signal_topic = rospy.get_param(
            "~signal_stop_topic", "/perception/traffic_light/stop_required"
        )
        self.signal_state_topic = rospy.get_param(
            "~signal_state_topic", "/perception/traffic_light/state"
        )
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/odometry")
        self.status_topic = rospy.get_param("~status_topic", "/control/stopline_status")
        self.rate_hz = max(1.0, float(rospy.get_param("~rate_hz", 20.0)))
        self.nominal_timeout_sec = max(0.05, float(rospy.get_param("~nominal_timeout_sec", 0.5)))
        self.stopline_timeout_sec = max(0.05, float(rospy.get_param("~stopline_timeout_sec", 0.5)))
        self.signal_timeout_sec = max(0.05, float(rospy.get_param("~signal_timeout_sec", 0.7)))
        self.odom_timeout_sec = max(0.05, float(rospy.get_param("~odom_timeout_sec", 0.5)))
        self.longl_cmd_type = int(rospy.get_param("~longl_cmd_type", 1))
        self.core = StopLineControllerCore(
            max_decel_mps2=rospy.get_param("~max_decel_mps2", 1.5),
            reaction_time_sec=rospy.get_param("~reaction_time_sec", 0.3),
            hold_distance_m=rospy.get_param("~hold_distance_m", 0.5),
            approach_distance_m=rospy.get_param("~approach_distance_m", 20.0),
            trigger_margin_m=rospy.get_param("~trigger_margin_m", 1.0),
            brake_response_time_sec=rospy.get_param("~brake_response_time_sec", 0.5),
            hold_speed_mps=rospy.get_param("~hold_speed_mps", 0.15),
            min_confidence=rospy.get_param("~min_confidence", 0.5),
        )

        self.last_nominal: Optional[CtrlCmd] = None
        self.last_nominal_time = 0.0
        self.last_stopline: Optional[StopLineDetection] = None
        self.last_stopline_time = 0.0
        self.last_signal_stop = False
        self.last_signal_time = 0.0
        self.last_signal_state: Optional[TrafficLight] = None
        self.last_signal_state_time = 0.0
        self.last_odom: Optional[Odometry] = None
        self.last_odom_time = 0.0

        self.output_pub = rospy.Publisher(self.output_topic, CtrlCmd, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(self.nominal_topic, CtrlCmd, self.nominal_callback, queue_size=10)
        rospy.Subscriber(self.stopline_topic, StopLineDetection, self.stopline_callback, queue_size=10)
        rospy.Subscriber(self.signal_topic, Bool, self.signal_callback, queue_size=10)
        rospy.Subscriber(
            self.signal_state_topic, TrafficLight, self.signal_state_callback, queue_size=10
        )
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.publish_command)

        rospy.loginfo(
            "Stopline controller nominal=%s stopline=%s signal=%s output=%s enabled=%s",
            self.nominal_topic, self.stopline_topic, self.signal_topic,
            self.output_topic, self.enabled,
        )

    def nominal_callback(self, message: CtrlCmd) -> None:
        self.last_nominal = copy.deepcopy(message)
        self.last_nominal_time = time.monotonic()

    def stopline_callback(self, message: StopLineDetection) -> None:
        self.last_stopline = message
        self.last_stopline_time = time.monotonic()

    def signal_callback(self, message: Bool) -> None:
        self.last_signal_stop = bool(message.data)
        self.last_signal_time = time.monotonic()

    def signal_state_callback(self, message: TrafficLight) -> None:
        self.last_signal_state = message
        self.last_signal_state_time = time.monotonic()

    def signal_stop_required(self, now: float) -> tuple[bool, bool]:
        """timestamped TrafficLight를 우선하고 기존 Bool을 fallback으로 사용한다."""
        if (
            self.last_signal_state is not None
            and now - self.last_signal_state_time <= self.signal_timeout_sec
        ):
            state = str(self.last_signal_state.state).strip().upper()
            if self.last_signal_state.valid and state in ("RED", "YELLOW", "AMBER", "RED_STOP", "YELLOW_STOP", "UNKNOWN_STOP"):
                return True, True
            if self.last_signal_state.valid and state == "GREEN":
                return False, True
        bool_fresh = now - self.last_signal_time <= self.signal_timeout_sec
        return bool(self.last_signal_stop and bool_fresh), bool_fresh

    def odom_callback(self, message: Odometry) -> None:
        self.last_odom = message
        self.last_odom_time = time.monotonic()

    def measured_speed_mps(self) -> Optional[float]:
        if self.last_odom is None or time.monotonic() - self.last_odom_time > self.odom_timeout_sec:
            return None
        vx = float(self.last_odom.twist.twist.linear.x)
        vy = float(self.last_odom.twist.twist.linear.y)
        if not finite(vx) or not finite(vy):
            return None
        return max(0.0, math.hypot(vx, vy))

    @staticmethod
    def stop_command(longl_cmd_type: int) -> CtrlCmd:
        command = CtrlCmd()
        command.longlCmdType = longl_cmd_type
        command.steering = 0.0
        command.velocity = 0.0
        command.accel = 0.0
        command.brake = 1.0
        return command

    def stopline_is_fresh(self, now: float) -> bool:
        return bool(
            self.last_stopline is not None
            and now - self.last_stopline_time <= self.stopline_timeout_sec
        )

    def publish_status(self, mode: str, reason: str, decision: StopLineDecision) -> None:
        self.status_pub.publish(String(data=json.dumps({
            "enabled": self.enabled,
            "mode": mode,
            "reason": reason,
            "decision": decision.mode,
            "decision_reason": decision.reason,
            "brake": decision.brake,
            "accel": decision.accel,
            "desired_speed_kph": (
                None if decision.desired_speed_mps is None
                else decision.desired_speed_mps * MPS_TO_KPH
            ),
            "signal_stop_required": self.last_signal_stop,
            "stopline_distance_m": (
                None if self.last_stopline is None
                else float(self.last_stopline.distance_m)
            ),
        }, ensure_ascii=False, sort_keys=True)))

    def publish_command(self, _event) -> None:
        now = time.monotonic()
        nominal_fresh = (
            self.last_nominal is not None
            and now - self.last_nominal_time <= self.nominal_timeout_sec
        )
        if not nominal_fresh:
            command = self.stop_command(self.longl_cmd_type)
            self.output_pub.publish(command)
            self.publish_status("stop", "nominal_stale", StopLineDecision("stop", "nominal_stale", brake=1.0))
            return

        output = copy.deepcopy(self.last_nominal)
        if not self.enabled:
            self.output_pub.publish(output)
            self.publish_status("pass_through", "disabled", StopLineDecision("nominal", "disabled"))
            return

        signal_stop_required, signal_fresh = self.signal_stop_required(now)
        stopline_fresh = self.stopline_is_fresh(now)
        stopline = self.last_stopline
        decision = self.core.decide(
            signal_stop_required=(signal_stop_required and signal_fresh),
            stopline_valid=bool(stopline_fresh and stopline is not None and stopline.valid),
            stopline_distance_m=(float(stopline.distance_m) if stopline is not None else math.nan),
            stopline_confidence=(float(stopline.confidence) if stopline is not None else 0.0),
            speed_mps=self.measured_speed_mps(),
        )

        if decision.mode in ("approach_stopline", "hold_stop", "stop"):
            output.longlCmdType = self.longl_cmd_type
            output.accel = clamp(decision.accel, 0.0, 1.0)
            output.brake = clamp(decision.brake, 0.0, 1.0)
            if output.brake > 0.0:
                output.accel = 0.0
        self.output_pub.publish(output)
        self.publish_status(decision.mode, decision.reason, decision)


if __name__ == "__main__":
    try:
        StopLineController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
