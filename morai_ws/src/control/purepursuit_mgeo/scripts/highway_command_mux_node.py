#!/usr/bin/env python3
"""Forward the highway controller's command in the highway section, the main one elsewhere.

final_ws_curvature_signal_highway.launch: the team's curvature controller
drives the whole route on ~main_command_topic and the highway controller
follows highway_lane_strategy on ~highway_command_topic.  This node publishes
one of them on ~output_topic, the fusion's nominal command, as
purepursuit_mgeo.highway_section.mux_choice decides from highway_lane_strategy's
state: the highway one from the zone entry to ~highway_end_s.
"""
import json
import threading

import rospy
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import String

from purepursuit_mgeo.highway_section import mux_choice


class HighwayCommandMux:
    def __init__(self):
        self.highway_end_s = float(rospy.get_param("~highway_end_s", 1741.721))
        self.timeout_s = float(rospy.get_param("~timeout_s", 0.5))
        self.lock = threading.Lock()
        self.state = None
        self.state_at = None
        self.highway_at = None
        self.use_highway = False
        self.output = rospy.Publisher(rospy.get_param("~output_topic", "/control/ctrl_cmd"),
                                      CtrlCmd, queue_size=1)
        rospy.Subscriber(rospy.get_param("~state_topic", "/highway_lane_strategy/state"),
                         String, self._state_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~main_command_topic", "/control/ctrl_cmd_main"),
                         CtrlCmd, self._main_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~highway_command_topic", "/control/ctrl_cmd_highway"),
                         CtrlCmd, self._highway_cb, queue_size=1)

    def _state_cb(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.state, self.state_at = state, rospy.get_time()

    def _decide(self, now):
        state_age = float("inf") if self.state_at is None else now - self.state_at
        command_age = float("inf") if self.highway_at is None else now - self.highway_at
        use, reason = mux_choice(self.state, state_age, command_age, self.highway_end_s,
                                 self.timeout_s)
        if use != self.use_highway:
            rospy.logwarn("HIGHWAY_MUX %s controller (%s)", "highway" if use else "main", reason)
            self.use_highway = use
        return use

    def _main_cb(self, msg):
        with self.lock:
            use = self._decide(rospy.get_time())
        if not use:
            self.output.publish(msg)

    def _highway_cb(self, msg):
        with self.lock:
            self.highway_at = rospy.get_time()
            use = self._decide(self.highway_at)
        if use:
            self.output.publish(msg)


if __name__ == "__main__":
    rospy.init_node("highway_command_mux")
    HighwayCommandMux()
    rospy.spin()
