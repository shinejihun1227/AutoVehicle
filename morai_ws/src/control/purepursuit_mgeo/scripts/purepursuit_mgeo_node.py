#!/usr/bin/env python3
"""MGeo local ENU pose를 이용해 MORAI CtrlCmd Pure Pursuit를 실행한다."""

from __future__ import annotations

import math
from typing import Optional

import rospy
from geometry_msgs.msg import PointStamped
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from purepursuit_mgeo.longitudinal_controller import MPS_TO_KPH, SpeedPIController
from purepursuit_mgeo.path import MgeoPurePursuit, PathPoint, load_mgeo_path


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class PurePursuitNode:
    def __init__(self) -> None:
        rospy.init_node("purepursuit_mgeo", anonymous=False)

        path_file = rospy.get_param("~path_file")
        self.points = load_mgeo_path(path_file)
        target_speed_kph = rospy.get_param("~target_speed_kph", None)
        if target_speed_kph is None:
            # 기존 m/s 파라미터를 사용하는 launch와의 호환성
            target_speed_kph = float(rospy.get_param("~target_speed_mps", 2.0)) * MPS_TO_KPH
        self.target_speed_kph = max(0.0, float(target_speed_kph))
        self.max_steering = float(
            rospy.get_param("~max_steering_rad", math.radians(40.0))
        )
        self.rate_hz = float(rospy.get_param("~control_rate_hz", 20.0))
        self.enable_control = bool(rospy.get_param("~enable_control", False))
        self.longl_cmd_type = int(rospy.get_param("~longl_cmd_type", 1))
        self.steering_sign = float(rospy.get_param("~steering_sign", 1.0))
        self.speed_kp = max(0.0, float(rospy.get_param("~speed_kp", 0.8)))
        self.speed_ki = max(0.0, float(rospy.get_param("~speed_ki", 0.05)))
        self.max_accel_mps2 = max(
            1e-6, float(rospy.get_param("~max_accel_mps2", 1.0))
        )
        self.max_decel_mps2 = max(
            1e-6, float(rospy.get_param("~max_decel_mps2", 1.5))
        )
        self.speed_controller = SpeedPIController(
            kp=self.speed_kp,
            ki=self.speed_ki,
            max_accel_mps2=self.max_accel_mps2,
            max_decel_mps2=self.max_decel_mps2,
            integral_limit_kph_s=max(
                0.0, float(rospy.get_param("~speed_integral_limit_kph_s", 10.8))
            ),
            speed_error_deadband_kph=max(
                0.0, float(rospy.get_param("~speed_error_deadband_kph", 0.1))
            ),
        )

        wheelbase = float(rospy.get_param("~wheelbase_m", 3.0))
        lookahead_min = float(rospy.get_param("~lookahead_min_m", 4.0))
        lookahead_gain = float(rospy.get_param("~lookahead_gain", 0.35))
        goal_tolerance = float(rospy.get_param("~goal_tolerance_m", 1.5))
        self.controller = MgeoPurePursuit(
            self.points,
            wheelbase,
            lookahead_min,
            lookahead_gain,
            goal_tolerance,
            self.steering_sign,
        )

        self.pose_topic = rospy.get_param("~pose_topic", "/localization/odometry")
        # 정상 주행 명령은 control_mux가 최종 /ctrl_cmd로 중재한다.
        self.command_topic = rospy.get_param("~command_topic", "/control/ctrl_cmd")
        self.lookahead_topic = rospy.get_param("~lookahead_topic", "/control/lookahead_point")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.latest_odom: Optional[Odometry] = None

        rospy.Subscriber(self.pose_topic, Odometry, self.odom_callback, queue_size=10)
        self.command_pub = rospy.Publisher(self.command_topic, CtrlCmd, queue_size=1)
        self.lookahead_pub = rospy.Publisher(self.lookahead_topic, PointStamped, queue_size=1)
        self.steering_preview_pub = rospy.Publisher("/control/steering_preview", Float64, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self.control_callback)

        rospy.logwarn(
            "Pure Pursuit 제어=%s path=%s points=%d wheelbase=%.3f lookahead_min=%.3f",
            self.enable_control,
            path_file,
            len(self.points),
            wheelbase,
            lookahead_min,
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def control_callback(self, _event: rospy.timer.TimerEvent) -> None:
        if self.latest_odom is None:
            rospy.logwarn_throttle(5.0, "Pure Pursuit가 /localization/odometry를 기다리는 중이다.")
            return

        pose = self.latest_odom.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        speed_mps = math.hypot(
            self.latest_odom.twist.twist.linear.x,
            self.latest_odom.twist.twist.linear.y,
        )
        steering, stop, target, target_index, lookahead = self.controller.compute(
            pose.position.x,
            pose.position.y,
            yaw,
            speed_mps,
        )
        steering = max(-self.max_steering, min(self.max_steering, steering))

        target_msg = PointStamped()
        target_msg.header.stamp = rospy.Time.now()
        target_msg.header.frame_id = self.map_frame
        target_msg.point.x = target.x
        target_msg.point.y = target.y
        target_msg.point.z = target.z
        self.lookahead_pub.publish(target_msg)
        self.steering_preview_pub.publish(Float64(steering))

        if self.enable_control:
            command = self.make_command(
                steering,
                self.target_speed_kph,
                speed_mps * MPS_TO_KPH,
                stop,
                1.0 / max(self.rate_hz, 1.0),
            )
            self.command_pub.publish(command)

        rospy.loginfo_throttle(
            2.0,
            "Pure Pursuit index=%d lookahead=%.2f steering=%.4f stop=%s",
            target_index,
            lookahead,
            steering,
            stop,
        )

    def make_command(
        self,
        steering: float,
        target_speed_kph: float,
        measured_speed_kph: float,
        stop: bool,
        dt: float,
    ) -> CtrlCmd:
        command = CtrlCmd()
        if hasattr(command, "longlCmdType"):
            command.longlCmdType = self.longl_cmd_type
        if hasattr(command, "steering"):
            command.steering = 0.0 if stop else steering

        pedal = self.speed_controller.update(
            target_speed_kph,
            measured_speed_kph,
            dt,
            stop=stop,
        )
        if self.longl_cmd_type == 1:
            if hasattr(command, "brake"):
                command.brake = pedal.brake
            if hasattr(command, "accel"):
                command.accel = pedal.accel
            if hasattr(command, "acceleration"):
                # 종방향 type 1에서는 accel/brake 필드를 사용한다.
                command.acceleration = 0.0
            if hasattr(command, "velocity"):
                # type 1에서 velocity는 사용하지 않으며 혼동을 막기 위해 0으로 둔다.
                command.velocity = 0.0
        else:
            # 구형 설정으로 type 2를 명시한 경우에만 속도 명령 호환을 유지한다.
            self.speed_controller.reset()
            if hasattr(command, "brake"):
                command.brake = 1.0 if stop else 0.0
            if hasattr(command, "accel"):
                command.accel = 0.0
            if hasattr(command, "acceleration"):
                command.acceleration = 0.0
            if hasattr(command, "velocity"):
                command.velocity = (
                    0.0
                    if stop
                    else max(0.0, target_speed_kph) / MPS_TO_KPH
                )
        return command


if __name__ == "__main__":
    try:
        PurePursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
