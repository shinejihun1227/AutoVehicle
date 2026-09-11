#!/usr/bin/env python3
"""Standalone lamp-only scheduler; final driving uses maneuver_fusion_node."""

from __future__ import annotations

import math
import socket
import struct
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String

from turn_signal_controller.scheduler import (
    LEFT,
    OFF,
    RIGHT,
    TurnSignalScheduler,
    parse_maneuvers,
)


LAMP_PACKET_FORMAT = "<13s i 3i b b 2s"
LAMP_HEADER = b"#LampControl$"
LAMP_TAIL = b"\r\n"
DIRECTION_CODE = {OFF: 0, LEFT: 1, RIGHT: 2}


def build_lamp_packet(direction: str) -> bytes:
    """MORAI TurnSignalLampControl 33바이트 패킷을 생성한다."""

    if direction not in DIRECTION_CODE:
        raise ValueError(f"지원하지 않는 방향지시등 상태: {direction}")
    return struct.pack(
        LAMP_PACKET_FORMAT,
        LAMP_HEADER,
        2,
        0,
        0,
        0,
        DIRECTION_CODE[direction],
        0,
        LAMP_TAIL,
    )


class TurnSignalController:
    def __init__(self) -> None:
        rospy.init_node("turn_signal_controller", anonymous=False)

        self.remote_ip = rospy.get_param("~remote_ip", "192.168.0.148")
        self.remote_port = int(rospy.get_param("~remote_port", 9097))
        self.progress_topic = rospy.get_param(
            "~progress_topic", "/experimental/curvature_progress"
        )
        self.odometry_topic = rospy.get_param(
            "~odometry_topic", "/localization/odometry"
        )
        self.progress_timeout_sec = max(
            0.1, float(rospy.get_param("~progress_timeout_sec", 1.0))
        )
        self.refresh_sec = max(
            0.05, float(rospy.get_param("~refresh_sec", 0.5))
        )
        default_duration_sec = max(
            0.1, float(rospy.get_param("~default_duration_sec", 8.0))
        )
        maneuvers = parse_maneuvers(
            rospy.get_param("~maneuvers", []),
            default_duration_sec=default_duration_sec,
        )
        self.scheduler = TurnSignalScheduler(
            maneuvers,
            lead_time_sec=float(rospy.get_param("~lead_time_sec", 5.0)),
            min_prediction_speed_mps=float(
                rospy.get_param("~min_prediction_speed_mps", 0.5)
            ),
            progress_reset_m=float(rospy.get_param("~progress_reset_m", 5.0)),
        )

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.latest_progress = None
        self.latest_progress_wall_time = None
        self.latest_speed_mps = 0.0
        self.current_direction = OFF
        self.last_send_wall_time = 0.0

        self.state_pub = rospy.Publisher(
            rospy.get_param("~state_topic", "/control/turn_signal_state"),
            String,
            queue_size=1,
            latch=True,
        )
        rospy.Subscriber(self.progress_topic, Float64, self.progress_callback, queue_size=5)
        rospy.Subscriber(self.odometry_topic, Odometry, self.odometry_callback, queue_size=5)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)
        rospy.on_shutdown(self.shutdown)

        if maneuvers:
            rospy.loginfo(
                "Turn signal controller: events=%d lead_time=%.2fs destination=%s:%d",
                len(maneuvers),
                self.scheduler.lead_time_sec,
                self.remote_ip,
                self.remote_port,
            )
        else:
            rospy.logwarn(
                "Turn signal controller가 실행되었지만 maneuver 이벤트가 비어 있다. "
                "~maneuvers에 start_s_m/direction을 등록해야 자동 점등한다."
            )
        self.send_direction(OFF, force=True)

    def progress_callback(self, message: Float64) -> None:
        value = float(message.data)
        if math.isfinite(value):
            self.latest_progress = value
            self.latest_progress_wall_time = time.monotonic()

    def odometry_callback(self, message: Odometry) -> None:
        velocity = message.twist.twist.linear
        speed = math.hypot(float(velocity.x), float(velocity.y))
        if math.isfinite(speed):
            self.latest_speed_mps = max(0.0, speed)

    def send_direction(self, direction: str, force: bool = False) -> None:
        now = time.monotonic()
        if (
            not force
            and direction == self.current_direction
            and now - self.last_send_wall_time < self.refresh_sec
        ):
            return
        try:
            packet = build_lamp_packet(direction)
            self.socket.sendto(packet, (self.remote_ip, self.remote_port))
        except (OSError, ValueError) as exc:
            rospy.logerr_throttle(5.0, "MORAI 방향지시등 UDP 송신 오류: %s", exc)
            return
        self.current_direction = direction
        self.last_send_wall_time = now
        self.state_pub.publish(String(direction))

    def timer_callback(self, _event) -> None:
        if (
            self.latest_progress is None
            or self.latest_progress_wall_time is None
            or time.monotonic() - self.latest_progress_wall_time
            > self.progress_timeout_sec
        ):
            self.send_direction(OFF)
            return

        decision = self.scheduler.update(
            self.latest_progress,
            self.latest_speed_mps,
            time.monotonic(),
        )
        self.send_direction(decision.direction)
        rospy.loginfo_throttle(
            2.0,
            "Turn signal state=%s phase=%s event=%s eta=%s progress=%.2f speed=%.2f",
            decision.direction,
            decision.phase,
            decision.maneuver_id or "-",
            "-" if decision.eta_sec is None else f"{decision.eta_sec:.2f}s",
            self.latest_progress,
            self.latest_speed_mps,
        )

    def shutdown(self) -> None:
        try:
            self.send_direction(OFF, force=True)
            self.socket.close()
        except OSError:
            pass


if __name__ == "__main__":
    try:
        TurnSignalController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
