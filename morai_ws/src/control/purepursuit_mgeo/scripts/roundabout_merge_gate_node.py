#!/usr/bin/env python3
"""Map-calibrated roundabout entry timing gate.

The route chooses HOW to enter. This gate chooses WHEN by comparing the ego
vehicle's conservative occupancy interval with circulating LiDAR tracks on a
configured ordered MGeo link chain. Without a calibrated location it never
authorizes a requested merge. It publishes no drive command.
"""

from __future__ import annotations

import json
import math
from typing import List, Optional, Tuple

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String

from lidar_perception.msg import LidarObstacleArray
from purepursuit_mgeo.mission_regions import load_mission_regions
from purepursuit_mgeo.route_geometry import RoutePolyline, travel_time


def _speed(msg: Odometry) -> float:
    velocity = msg.twist.twist.linear
    return math.hypot(float(velocity.x), float(velocity.y))


def _nearest_span(spans, progress):
    if not spans:
        return None
    return min(spans, key=lambda span: max(span[0] - progress, 0.0, progress - span[1]))


def intervals_overlap(a: Tuple[float, float], b: Tuple[float, float], margin_s: float) -> bool:
    return a[0] <= b[1] + margin_s and b[0] <= a[1] + margin_s


class RoundaboutMergeGate:
    IDLE = "IDLE"
    WAIT = "WAIT_GAP"
    CONFIRM = "CONFIRM_CLEAR"
    COMMITTED = "COMMITTED_GO"
    SENSOR_STOP = "SENSOR_STOP"
    EMERGENCY = "COMMITTED_HAZARD"

    def __init__(self) -> None:
        rospy.init_node("roundabout_merge_gate", anonymous=False)
        self.request_topic = rospy.get_param("~request_topic", "/planning/merge_request")
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/odometry")
        self.progress_topic = rospy.get_param("~progress_topic", "/experimental/curvature_progress")
        self.obstacle_topic = rospy.get_param("~obstacle_topic", "/perception/lidar/tracked_obstacles_map")
        self.path_file = rospy.get_param("~path_file")
        self.link_set_file = rospy.get_param("~link_set_file")
        self.config_file = rospy.get_param("~config_file")
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.5))
        self.progress_timeout_s = float(rospy.get_param("~progress_timeout_s", 0.5))
        self.obstacle_timeout_s = float(rospy.get_param("~obstacle_timeout_s", 0.6))
        self.request_timeout_s = float(rospy.get_param("~request_timeout_s", 0.6))
        self.clear_confirm_s = float(rospy.get_param("~clear_confirm_s", 0.8))
        self.prediction_horizon_s = float(rospy.get_param("~prediction_horizon_s", 8.0))
        self.temporal_margin_s = float(rospy.get_param("~temporal_margin_s", 1.0))
        self.track_speed_uncertainty_mps = float(rospy.get_param("~track_speed_uncertainty_mps", 0.6))
        self.track_corridor_m = float(rospy.get_param("~track_corridor_m", 3.5))
        self.unknown_track_range_m = float(rospy.get_param("~unknown_track_range_m", 18.0))
        self.extra_obstacle_radius_m = float(rospy.get_param("~extra_obstacle_radius_m", 0.5))
        self.vehicle_length_m = float(rospy.get_param("~vehicle_length_m", 4.635))
        self.vehicle_width_m = float(rospy.get_param("~vehicle_width_m", 1.892))
        self.entry_speed_mps = float(rospy.get_param("~entry_speed_mps", 2.0))
        self.fast_accel_mps2 = float(rospy.get_param("~fast_accel_mps2", 1.0))
        self.slow_accel_mps2 = float(rospy.get_param("~slow_accel_mps2", 0.5))
        self.launch_delay_min_s = float(rospy.get_param("~launch_delay_min_s", 0.0))
        self.launch_delay_max_s = float(rospy.get_param("~launch_delay_max_s", 0.5))
        if min(self.rate_hz, self.odom_timeout_s, self.progress_timeout_s, self.obstacle_timeout_s,
               self.request_timeout_s, self.prediction_horizon_s, self.entry_speed_mps,
               self.fast_accel_mps2, self.slow_accel_mps2,
               self.vehicle_length_m, self.vehicle_width_m) <= 0.0:
            raise ValueError("roundabout timing/speed parameters must be positive")
        if min(self.clear_confirm_s, self.temporal_margin_s,
               self.track_corridor_m, self.unknown_track_range_m,
               self.extra_obstacle_radius_m, self.track_speed_uncertainty_mps,
               self.launch_delay_min_s) < 0.0 or self.launch_delay_max_s < self.launch_delay_min_s:
            raise ValueError("invalid roundabout safety margin or launch delay")
        if self.slow_accel_mps2 > self.fast_accel_mps2:
            raise ValueError("slow_accel_mps2 must not exceed fast_accel_mps2")

        self.route = RoutePolyline.from_path_file(self.path_file)
        self.config = load_mission_regions(self.config_file, self.path_file, self.route.length)
        self.region = self.config["roundabout"]
        self.calibrated = bool(self.region["enabled"])
        self.circulating = None
        self.conflict_xy = None
        self.conflict_radius_m = None
        self.ego_span = None
        self.traffic_conflict_s = None
        if self.calibrated:
            self.conflict_xy = tuple(float(v) for v in self.region["conflict_xy_map"])
            self.conflict_radius_m = float(self.region["conflict_radius_m"])
            self.circulating = RoutePolyline.from_mgeo_links(
                self.link_set_file, self.region["circulating_link_ids"]
            )
            ego_spans = self.route.circle_spans(
                self.conflict_xy,
                self.conflict_radius_m + 0.5 * math.hypot(self.vehicle_length_m, self.vehicle_width_m),
            )
            self.ego_span = _nearest_span(ego_spans, float(self.region["conflict_s_m"]))
            if (self.ego_span is None
                    or not self.ego_span[0] <= float(self.region["conflict_s_m"]) <= self.ego_span[1]
                    or self.region["request_end_s_m"] <= self.ego_span[1] + 1.0):
                raise ValueError("roundabout ego route/conflict point or request_end_s_m mismatch")
            traffic_spans = self.circulating.circle_spans(self.conflict_xy, self.conflict_radius_m)
            if not traffic_spans:
                raise ValueError("circulating links do not cross the configured conflict zone")
            self.traffic_conflict_s = self.circulating.project(*self.conflict_xy)["s_m"]

        self.requested = False
        self.request_at = None
        self.progress: Optional[float] = None
        self.progress_at = None
        self.latest_odom: Optional[Odometry] = None
        self.odom_at = None
        self.latest_obstacles: Optional[LidarObstacleArray] = None
        self.obstacles_at = None
        self.clear_since = None
        self.committed = False
        self.state = self.IDLE
        self.stop_pub = rospy.Publisher("~stop_required", Bool, queue_size=1)
        self.allowed_pub = rospy.Publisher("~allowed", Bool, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)
        rospy.Subscriber(self.request_topic, Bool, self._request_cb, queue_size=1)
        rospy.Subscriber(self.progress_topic, Float64, self._progress_cb, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber(self.obstacle_topic, LidarObstacleArray, self._obstacle_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._timer_cb)
        rospy.logwarn("Roundabout merge gate calibrated=%s request=%s", self.calibrated, self.request_topic)

    def _request_cb(self, msg: Bool) -> None:
        requested = bool(msg.data)
        if self.requested and not requested:
            self.committed = False
            self.clear_since = None
            self.state = self.IDLE
        self.requested = requested
        self.request_at = rospy.Time.now()

    def _progress_cb(self, msg: Float64) -> None:
        value = float(msg.data)
        self.progress = value if math.isfinite(value) else None
        self.progress_at = rospy.Time.now()

    def _odom_cb(self, msg: Odometry) -> None:
        if msg.header.frame_id != "map":
            return
        self.latest_odom = msg
        self.odom_at = rospy.Time.now()

    def _obstacle_cb(self, msg: LidarObstacleArray) -> None:
        if msg.header.frame_id != "map":
            return
        self.latest_obstacles = msg
        self.obstacles_at = rospy.Time.now()

    @staticmethod
    def _fresh(stamp, timeout_s, now) -> bool:
        return stamp is not None and 0.0 <= (now - stamp).to_sec() <= timeout_s

    @staticmethod
    def _source_fresh(message, timeout_s, now) -> bool:
        if message is None or message.header.stamp.to_sec() <= 0.0:
            return False
        age = (now - message.header.stamp).to_sec()
        return 0.0 <= age <= timeout_s

    def _ego_interval(self) -> Optional[Tuple[float, float]]:
        if self.progress is None or not math.isfinite(float(self.progress)):
            return None
        progress = float(self.progress)
        if not 0.0 <= progress <= self.route.length:
            return None
        pose = self.latest_odom.pose.pose.position
        if not math.isfinite(float(pose.x)) or not math.isfinite(float(pose.y)):
            return None
        route_x, route_y = self.route.point_at(progress)
        route_offset = math.hypot(float(pose.x) - route_x, float(pose.y) - route_y)
        if (not math.isfinite(route_offset)
                or route_offset > float(self.region["max_route_offset_m"])):
            return None
        if not self.region["request_start_s_m"] - 3.0 <= progress < self.region["request_end_s_m"]:
            return None
        speed = _speed(self.latest_odom)
        if not math.isfinite(speed):
            return None
        start, end = self.ego_span
        entry = self.launch_delay_min_s + travel_time(
            max(0.0, start - progress), speed, self.fast_accel_mps2, self.entry_speed_mps
        )
        exit_time = self.launch_delay_max_s + travel_time(
            max(0.0, end - progress), speed, self.slow_accel_mps2, self.entry_speed_mps
        )
        return entry, max(entry, exit_time)

    def _traffic_interval(self, obstacle) -> Optional[Tuple[float, float]]:
        x, y = float(obstacle.center_x_map), float(obstacle.center_y_map)
        vx, vy = float(obstacle.velocity_x_map), float(obstacle.velocity_y_map)
        length, width = float(obstacle.length), float(obstacle.width)
        if not all(math.isfinite(v) for v in (x, y, vx, vy, length, width)):
            return (0.0, self.prediction_horizon_s)
        footprint = 0.5 * math.hypot(max(0.1, length), max(0.1, width))
        radius = self.conflict_radius_m + footprint + self.extra_obstacle_radius_m
        distance_to_conflict = math.hypot(x - self.conflict_xy[0], y - self.conflict_xy[1])
        if distance_to_conflict <= radius:
            return (0.0, self.prediction_horizon_s)
        match = self.circulating.project(x, y)
        if match["distance_m"] > self.track_corridor_m:
            # An unassociated nearby track may be crossing another arm.
            return ((0.0, self.prediction_horizon_s)
                    if distance_to_conflict <= self.unknown_track_range_m else None)
        spans = self.circulating.circle_spans(self.conflict_xy, radius)
        span = _nearest_span(spans, self.traffic_conflict_s)
        if span is None or match["s_m"] > span[1]:
            return None
        along = vx * match["tangent"][0] + vy * match["tangent"][1]
        if along <= 0.25:
            return ((0.0, self.prediction_horizon_s)
                    if span[0] - match["s_m"] <= self.unknown_track_range_m else None)
        distance_in = max(0.0, span[0] - match["s_m"])
        distance_out = max(0.0, span[1] - match["s_m"])
        earliest = distance_in / (along + self.track_speed_uncertainty_mps)
        latest = distance_out / max(0.25, along - self.track_speed_uncertainty_mps)
        if earliest > self.prediction_horizon_s:
            return None
        return earliest, min(self.prediction_horizon_s, latest)

    def _blocking_objects(self, ego_interval) -> List[dict]:
        blockers = []
        for obstacle in self.latest_obstacles.obstacles:
            interval = self._traffic_interval(obstacle)
            if interval is None or not intervals_overlap(ego_interval, interval, self.temporal_margin_s):
                continue
            speed = math.hypot(float(obstacle.velocity_x_map), float(obstacle.velocity_y_map))
            blockers.append({
                "id": int(obstacle.id),
                "traffic_entry_s": round(interval[0], 2),
                "traffic_exit_s": round(interval[1], 2),
                "speed_mps": round(speed, 2) if math.isfinite(speed) else None,
            })
        return blockers

    def _timer_cb(self, _event) -> None:
        now = rospy.Time.now()
        stop = False
        allowed = False
        reason = "idle"
        blockers = []
        ego_interval = None
        request_fresh = self._fresh(self.request_at, self.request_timeout_s, now)
        sensors_fresh = (self._fresh(self.odom_at, self.odom_timeout_s, now)
                         and self._fresh(self.progress_at, self.progress_timeout_s, now)
                         and self._fresh(self.obstacles_at, self.obstacle_timeout_s, now)
                         and self._source_fresh(self.latest_odom, self.odom_timeout_s, now)
                         and self._source_fresh(self.latest_obstacles, self.obstacle_timeout_s, now))
        if not self.requested:
            self.state = self.IDLE
            self.committed = False
            self.clear_since = None
        elif not self.calibrated:
            self.state, stop, reason = self.SENSOR_STOP, True, "roundabout_not_calibrated"
            self.clear_since = None
        elif not request_fresh or not sensors_fresh:
            self.state, stop, reason = self.SENSOR_STOP, True, "request_or_sensor_stale"
            self.clear_since = None
            self.committed = False
        else:
            ego_interval = self._ego_interval()
            if ego_interval is None:
                self.state, stop, reason = self.SENSOR_STOP, True, "ego_route_or_position_invalid"
                self.clear_since = None
                self.committed = False
            else:
                blockers = self._blocking_objects(ego_interval)
                if self.committed:
                    if blockers:
                        self.state, stop, reason = self.EMERGENCY, True, "committed_collision_predicted"
                        self.committed = False
                        self.clear_since = None
                    else:
                        self.state, allowed, reason = self.COMMITTED, True, "committed_clear"
                elif blockers:
                    self.state, stop, reason = self.WAIT, True, "predicted_conflict"
                    self.clear_since = None
                else:
                    if self.clear_since is None:
                        self.clear_since = now
                    if (now - self.clear_since).to_sec() < self.clear_confirm_s:
                        self.state, stop, reason = self.CONFIRM, True, "confirming_clear_gap"
                    else:
                        self.committed = True
                        self.state, allowed, reason = self.COMMITTED, True, "gap_accepted"
                        rospy.logwarn("ROUNDABOUT MERGE COMMITTED: clear occupancy interval confirmed")
        self.stop_pub.publish(Bool(data=stop))
        self.allowed_pub.publish(Bool(data=allowed))
        self.status_pub.publish(String(data=json.dumps({
            "state": self.state, "reason": reason, "requested": self.requested,
            "request_fresh": request_fresh, "sensor_fresh": sensors_fresh,
            "route_progress_s_m": self.progress if sensors_fresh else None,
            "calibrated": self.calibrated, "committed": self.committed,
            "stop_required": stop, "allowed": allowed,
            "ego_interval_s": None if ego_interval is None else [round(v, 2) for v in ego_interval],
            "blockers": blockers, "conflict_xy_map": self.conflict_xy,
            "entry_s_m": self.region.get("entry_s_m"),
        }, ensure_ascii=False, separators=(",", ":"))))
        rospy.loginfo_throttle(1.0, "MergeGate state=%s reason=%s blockers=%d", self.state, reason, len(blockers))


def main() -> None:
    try:
        RoundaboutMergeGate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
