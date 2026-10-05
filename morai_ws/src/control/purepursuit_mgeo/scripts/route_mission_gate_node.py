#!/usr/bin/env python3
"""Publish route-scoped highway and calibrated roundabout requests.

The global route's arc length is the location authority.  Camera detection
confirms highway activation; a route interval alone never authorizes a lane
change.  An uncalibrated roundabout remains disabled.
"""

from __future__ import annotations

import json
import math

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String

from purepursuit_mgeo.mission_regions import load_mission_regions
from purepursuit_mgeo.route_geometry import RoutePolyline


class RouteMissionGate:
    def __init__(self) -> None:
        rospy.init_node("route_mission_gate", anonymous=False)
        self.path_file = rospy.get_param("~path_file")
        self.config_file = rospy.get_param("~config_file")
        self.route = RoutePolyline.from_path_file(self.path_file)
        self.config = load_mission_regions(self.config_file, self.path_file, self.route.length)
        self.progress_topic = rospy.get_param("~progress_topic", "/experimental/curvature_progress")
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/odometry")
        self.highway_topic = rospy.get_param("~highway_topic", "/perception/camera/highway_environment")
        self.highway_state_topic = rospy.get_param("~highway_state_topic", "/highway_lane_strategy/state")
        self.highway_route_topic = rospy.get_param("~highway_route_topic", "/planning/highway_route_active")
        self.highway_request_topic = rospy.get_param("~highway_request_topic", "/planning/highway_lane_change_request")
        self.handoff_topic = rospy.get_param("~handoff_topic", "/planning/highway_handoff_due")
        self.merge_request_topic = rospy.get_param("~merge_request_topic", "/planning/merge_request")
        self.timeout_s = float(rospy.get_param("~input_timeout_s", 0.6))
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        if self.timeout_s <= 0.0 or self.rate_hz <= 0.0:
            raise ValueError("input_timeout_s and rate_hz must be positive")

        self.progress = None
        self.progress_at = None
        self.odom = None
        self.odom_at = None
        self.camera_highway = False
        self.camera_at = None
        self.highway_state = None
        self.highway_state_at = None
        self.handoff_reached = False
        self.highway_route_pub = rospy.Publisher(self.highway_route_topic, Bool, queue_size=1)
        self.highway_request_pub = rospy.Publisher(self.highway_request_topic, Bool, queue_size=1)
        self.handoff_pub = rospy.Publisher(self.handoff_topic, Bool, queue_size=1)
        self.merge_request_pub = rospy.Publisher(self.merge_request_topic, Bool, queue_size=1)
        self.ready_pub = rospy.Publisher("~ready", Bool, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)
        rospy.Subscriber(self.progress_topic, Float64, self._progress_cb, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber(self.highway_topic, Bool, self._highway_cb, queue_size=1)
        rospy.Subscriber(self.highway_state_topic, String, self._highway_state_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick)
        rospy.logwarn("Route mission regions loaded: highway %.2f..%.2f m; roundabout calibrated=%s",
                      self.config["highway"]["start_s_m"], self.config["highway"]["end_s_m"],
                      self.config["roundabout"]["enabled"])

    def _progress_cb(self, msg: Float64) -> None:
        value = float(msg.data)
        self.progress = value if math.isfinite(value) else None
        self.progress_at = rospy.Time.now()

    def _odom_cb(self, msg: Odometry) -> None:
        # Mission regions and route projection are all in MGeo map coordinates.
        # An empty frame is not enough evidence to activate a route-specific feature.
        if msg.header.frame_id != "map":
            return
        self.odom = msg
        self.odom_at = rospy.Time.now()

    def _highway_cb(self, msg: Bool) -> None:
        self.camera_highway = bool(msg.data)
        self.camera_at = rospy.Time.now()

    def _highway_state_cb(self, msg: String) -> None:
        try:
            state = json.loads(msg.data).get("state")
        except (TypeError, ValueError, AttributeError):
            state = None
        self.highway_state = state if isinstance(state, str) else None
        self.highway_state_at = rospy.Time.now()

    def _fresh(self, stamp, now) -> bool:
        return stamp is not None and 0.0 <= (now - stamp).to_sec() <= self.timeout_s

    def _odom_source_fresh(self, now) -> bool:
        if self.odom is None or self.odom.header.stamp.to_sec() <= 0.0:
            return False
        age = (now - self.odom.header.stamp).to_sec()
        return 0.0 <= age <= self.timeout_s

    def _tick(self, _event) -> None:
        now = rospy.Time.now()
        fresh = (self._fresh(self.progress_at, now) and self._fresh(self.odom_at, now)
                 and self._odom_source_fresh(now))
        if fresh:
            fresh = self.progress is not None and 0.0 <= self.progress <= self.route.length
        route_offset = None
        if fresh:
            pose = self.odom.pose.pose.position
            point = self.route.point_at(self.progress)
            route_offset = math.hypot(float(pose.x) - point[0], float(pose.y) - point[1])
            fresh = math.isfinite(route_offset)
        highway = self.config["highway"]
        roundabout = self.config["roundabout"]
        if fresh:
            fresh = route_offset <= max(
                float(highway["max_route_offset_m"]),
                float(roundabout["max_route_offset_m"]),
            )
        in_roundabout_s = bool(
            fresh and roundabout["enabled"]
            and roundabout["request_start_s_m"] <= self.progress < roundabout["request_end_s_m"]
        )
        if in_roundabout_s and route_offset > roundabout["max_route_offset_m"]:
            fresh = False
        if (fresh and highway["handoff_enabled"]
                and self.progress >= highway["handoff_s_m"]):
            self.handoff_reached = True
        handoff_due = bool(fresh and highway["handoff_enabled"] and self.handoff_reached)
        committed_change = bool(
            self._fresh(self.highway_state_at, now) and self.highway_state == "LANE_CHANGE"
        )
        handoff_unconfigured_at_end = bool(
            fresh and not highway["handoff_enabled"] and self.progress >= highway["end_s_m"]
            and not committed_change
        )
        in_highway = bool(fresh and not self.handoff_reached
                          and highway["start_s_m"] <= self.progress < highway["end_s_m"]
                          and route_offset <= highway["max_route_offset_m"])
        camera_ready = self._fresh(self.camera_at, now) and self.camera_highway
        # The road camera, not the route file, decides that the sensed lane is
        # actually suitable for a highway strategy. A missing camera in the
        # configured highway zone makes the single command source stop.
        ready = bool(fresh and not handoff_unconfigured_at_end
                     and (not in_highway or camera_ready))
        merge_request = bool(
            ready and in_roundabout_s
        )
        self.highway_route_pub.publish(Bool(data=in_highway and ready))
        self.highway_request_pub.publish(Bool(data=in_highway and camera_ready and ready))
        self.handoff_pub.publish(Bool(data=handoff_due))
        self.merge_request_pub.publish(Bool(data=merge_request))
        self.ready_pub.publish(Bool(data=ready))
        self.status_pub.publish(String(data=json.dumps({
            "ready": ready, "progress_s_m": self.progress if fresh else None,
            "route_offset_m": route_offset if route_offset is not None and math.isfinite(route_offset) else None,
            "highway_route": in_highway,
            "camera_highway": camera_ready, "highway_request": in_highway and camera_ready and ready,
            "handoff_configured": bool(highway["handoff_enabled"]), "handoff_due": handoff_due,
            "handoff_unconfigured_at_end": handoff_unconfigured_at_end,
            "committed_lane_change": committed_change,
            "merge_request": merge_request, "roundabout_calibrated": bool(roundabout["enabled"]),
            "roundabout_entry_s_m": roundabout.get("entry_s_m"),
        }, separators=(",", ":"))))


def main() -> None:
    try:
        RouteMissionGate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
