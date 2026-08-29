#!/usr/bin/env python3
"""가상 nominal 명령·localization·safety_stop으로 control_mux를 검증한다."""

from __future__ import annotations

import rospy
from morai_msgs.msg import CtrlCmd
from morai_perception_msgs.msg import SafetyStop
from nav_msgs.msg import Odometry


class ControlMuxTestPublisher:
    def __init__(self) -> None:
        rospy.init_node("control_mux_test_publisher", anonymous=False)
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.stop_after_sec = float(rospy.get_param("~stop_after_sec", 5.0))
        self.clear_after_sec = float(rospy.get_param("~clear_after_sec", -1.0))
        self.publish_nominal = bool(rospy.get_param("~publish_nominal", True))
        self.publish_odom = bool(rospy.get_param("~publish_odom", True))
        self.start_time = rospy.Time.now().to_sec()
        self.nominal_pub = rospy.Publisher("/control/ctrl_cmd", CtrlCmd, queue_size=1)
        self.safety_pub = rospy.Publisher("/detection/fused_safety_stop", SafetyStop, queue_size=1)
        self.odom_pub = rospy.Publisher("/localization/odometry", Odometry, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self.publish)

    def publish(self, _event) -> None:
        now = rospy.Time.now()
        elapsed = now.to_sec() - self.start_time

        if self.publish_nominal:
            nominal = CtrlCmd()
            if hasattr(nominal, "longlCmdType"):
                nominal.longlCmdType = 2
            if hasattr(nominal, "velocity"):
                nominal.velocity = 1.0
            if hasattr(nominal, "steering"):
                nominal.steering = 0.0
            self.nominal_pub.publish(nominal)

        stop = elapsed >= self.stop_after_sec and (
            self.clear_after_sec < 0.0 or elapsed < self.clear_after_sec
        )
        safety = SafetyStop()
        safety.header.stamp = now
        safety.header.frame_id = "base_link"
        safety.stop_required = stop
        safety.distance_m = 3.0 if stop else -1.0
        safety.confidence = 1.0 if stop else 0.0
        safety.reason = "test_obstacle" if stop else "test_clear"
        self.safety_pub.publish(safety)

        if self.publish_odom:
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(odom)


if __name__ == "__main__":
    try:
        ControlMuxTestPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
