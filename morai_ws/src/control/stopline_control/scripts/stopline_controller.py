#!/usr/bin/env python3
"""Apply timestamp-checked, latched stop-line limits to nominal type-1 control."""

import copy
import json
import math
import threading
import time

import rospy
from morai_msgs.msg import CtrlCmd
from morai_perception_msgs.msg import StopLineDetection, TrafficLight
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from stopline_control.core import Decision, Sample, StopLineControllerCore, clamp, finite


def limit_command(nominal, decision):
    """An overlay may strengthen nominal braking but never cancel it."""
    output = copy.deepcopy(nominal)
    output.longlCmdType = 1
    output.velocity = 0.0
    output.acceleration = 0.0
    output.accel = min(clamp(float(nominal.accel), 0.0, 1.0), decision.accel_limit)
    output.brake = max(clamp(float(nominal.brake), 0.0, 1.0), decision.brake)
    if output.brake > 0.0:
        output.accel = 0.0
    return output


class StopLineController:
    def __init__(self):
        rospy.init_node("stopline_controller", anonymous=False)
        self.lock = threading.RLock()
        self.enabled = bool(rospy.get_param("~enabled", True))
        if int(rospy.get_param("~longl_cmd_type", 1)) != 1:
            raise ValueError("stopline_control requires accel/brake longl_cmd_type=1")
        self.nominal_timeout = float(rospy.get_param("~nominal_timeout_sec", 0.5))
        self.odom_timeout = float(rospy.get_param("~odom_timeout_sec", 0.5))
        self.stopline_frame = rospy.get_param("~stopline_frame", "base_link")
        defaults = dict(max_decel_mps2=1.5, planning_decel_mps2=1.0,
                        reaction_time_sec=0.3, hold_distance_m=0.5,
                        approach_distance_m=20.0, trigger_margin_m=1.0,
                        front_reference_offset_m=0.0, min_confidence=0.5,
                        signal_min_confidence=0.5, stopline_timeout_sec=0.8,
                        signal_timeout_sec=0.8, green_confirmation_sec=0.3,
                        max_dead_reckoning_sec=8.0, max_dead_reckoning_m=12.0,
                        max_update_gap_sec=0.5)
        self.core = StopLineControllerCore(**{
            key: float(rospy.get_param("~" + key, value)) for key, value in defaults.items()
        })
        self.nominal = None
        self.nominal_at = None
        self.odom = None
        self.last_odom_stamp = 0.0
        self.output_pub = rospy.Publisher(rospy.get_param("~output_command_topic", "/control/stopline_cmd"), CtrlCmd, queue_size=1)
        self.status_pub = rospy.Publisher(rospy.get_param("~status_topic", "/control/stopline_status"), String, queue_size=1, latch=True)
        rospy.Subscriber(rospy.get_param("~nominal_command_topic", "/control/ctrl_cmd"), CtrlCmd, self.nominal_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param("~stopline_topic", "/perception/camera/stopline"), StopLineDetection, self.stopline_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param("~signal_state_topic", "/perception/traffic_light/state"), TrafficLight, self.signal_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/localization/odometry"), Odometry, self.odom_callback, queue_size=1)
        # Optional legacy publishers may request a stop. Bool(false) never
        # proves a green light and cannot release a stop.
        if rospy.get_param("~allow_legacy_stop_request", False):
            rospy.Subscriber(rospy.get_param("~signal_stop_topic", "/perception/traffic_light/stop_required"), Bool, self.legacy_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, float(rospy.get_param("~rate_hz", 20.0)))), self.publish_command)

    def nominal_callback(self, message):
        with self.lock:
            self.nominal = copy.deepcopy(message)
            self.nominal_at = time.monotonic()

    def stopline_callback(self, message):
        with self.lock:
            if message.header.frame_id != self.stopline_frame:
                rospy.logwarn_throttle(5.0, "Stopline frame mismatch: %s expected %s", message.header.frame_id, self.stopline_frame)
                return
            self.core.observe_line(message.distance_m, message.confidence, message.valid,
                                   message.header.stamp.to_sec(), time.monotonic(), rospy.get_time())

    def signal_callback(self, message):
        with self.lock:
            self.core.observe_signal(message.state, message.confidence, message.valid,
                                     message.header.stamp.to_sec(), time.monotonic(), rospy.get_time())

    def legacy_callback(self, message):
        if message.data:
            with self.lock:
                self.core.stop_requested = True

    def odom_callback(self, message):
        stamp, now, ros_now = message.header.stamp.to_sec(), time.monotonic(), rospy.get_time()
        speed = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        sample = Sample(stamp, now, speed)
        with self.lock:
            if stamp > self.last_odom_stamp and finite(speed) and sample.fresh(ros_now, now, self.odom_timeout):
                self.odom = sample
                self.last_odom_stamp = stamp

    def publish_command(self, _event):
        with self.lock:
            now, ros_now = time.monotonic(), rospy.get_time()
            speed = (self.odom.value if self.odom is not None
                     and self.odom.fresh(ros_now, now, self.odom_timeout) else None)
            decision = self.core.update(now, ros_now, speed) if self.enabled else Decision("NOMINAL", "disabled")
            if decision.reason == "clock_reset":
                self.odom = None
                self.last_odom_stamp = 0.0
            nominal = self.nominal
            fresh = nominal is not None and self.nominal_at is not None and 0 <= now - self.nominal_at <= self.nominal_timeout
            valid = fresh and all(finite(float(getattr(nominal, key))) for key in ("accel", "brake", "steering"))
            if not valid or (self.enabled and nominal.longlCmdType != 1):
                nominal = CtrlCmd()
                decision = Decision("SAFE_STOP", "nominal_stale_or_invalid", 0.0, 1.0, 0.0)
            output = (copy.deepcopy(nominal) if not self.enabled and valid
                      else limit_command(nominal, decision))
            self.output_pub.publish(output)
            self.status_pub.publish(String(data=json.dumps({
                "enabled": self.enabled, "mode": decision.mode, "reason": decision.reason,
                "stop_requested": self.core.stop_requested, "holding": self.core.holding,
                "distance_m": decision.distance_m, "target_speed_kph": decision.target_speed_kph,
                "measured_speed_kph": None if speed is None else speed * 3.6,
                "accel": output.accel, "brake": output.brake,
                "front_reference_offset_m": self.core.front_reference_offset_m,
            }, allow_nan=False, sort_keys=True)))


if __name__ == "__main__":
    try:
        StopLineController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
