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
import threading
from typing import List, Optional, Tuple
from types import SimpleNamespace

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
        # Keep recent LiDAR frames as anonymous motion hypotheses. This only
        # affects roundabout conflict prediction; the existing gap-confirmation
        # timer and command interface remain unchanged.
        self.observation_history_s = float(rospy.get_param("~observation_history_s", 0.8))
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
        if not 0.0 < self.observation_history_s <= 3.0:
            raise ValueError("observation_history_s must be in (0, 3] seconds")
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
        self.circulating_closed = False
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
            self.circulating_closed = math.dist(
                self.circulating.points[0], self.circulating.points[-1]
            ) <= 2.0

        self.requested = False
        self.request_at = None
        self.progress: Optional[float] = None
        self.progress_at = None
        self.latest_odom: Optional[Odometry] = None
        self.odom_at = None
        self.latest_obstacles: Optional[LidarObstacleArray] = None
        self.obstacles_at = None
        self.observation_lock = threading.RLock()
        self.traffic_span_cache = {}
        self.latest_obstacle_frame = None
        self.last_obstacle_source_stamp = None
        self.observation_history = []
        self.max_observation_frames = max(
            4, min(120, int(math.ceil(self.rate_hz * self.observation_history_s * 2.0)))
        )
        self.flow_hypotheses = []
        self.last_flow_frame_stamp = None
        self.last_flow_update_s = None
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
        with self.observation_lock:
            if requested != self.requested:
                self._clear_observation_memory()
            if (requested and not self.requested and self.calibrated
                    and self.latest_obstacle_frame is not None):
                stamp, observations = self.latest_obstacle_frame
                age = rospy.Time.now().to_sec() - stamp
                if 0.0 <= age <= self.observation_history_s:
                    self.observation_history.append((stamp, observations))
                    self._prune_observation_history(rospy.Time.now().to_sec())
                    self._ingest_flow_frame(stamp, observations)
            if self.requested and not requested:
                self.committed = False
                self.clear_since = None
                self.state = self.IDLE
            self.requested = requested
            self.request_at = rospy.Time.now()

    def _clear_observation_memory(self) -> None:
        """Reset anonymous traffic memory when the regional mission toggles."""
        self.observation_history.clear()
        self.flow_hypotheses.clear()
        self.last_flow_frame_stamp = None
        self.last_flow_update_s = None

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
        now = rospy.Time.now()
        stamp = msg.header.stamp.to_sec()
        age = now.to_sec() - stamp
        if (not math.isfinite(stamp) or stamp <= 0.0 or age < 0.0
                or age > self.obstacle_timeout_s):
            return
        observations = tuple({
            "id": int(obstacle.id),
            "x": float(obstacle.center_x_map),
            "y": float(obstacle.center_y_map),
            "vx": float(obstacle.velocity_x_map),
            "vy": float(obstacle.velocity_y_map),
            "length": float(obstacle.length),
            "width": float(obstacle.width),
        } for obstacle in msg.obstacles)
        with self.observation_lock:
            if (self.last_obstacle_source_stamp is not None
                    and stamp <= self.last_obstacle_source_stamp):
                # Duplicate/out-of-order packets must not refresh either sensor
                # freshness or the lifetime of a remembered vehicle.
                return
            self.latest_obstacles = msg
            self.obstacles_at = now
            self.last_obstacle_source_stamp = stamp
            self.latest_obstacle_frame = (stamp, observations)
            if self.requested and self.calibrated:
                self.observation_history.append((stamp, observations))
                self._prune_observation_history(now.to_sec())
                self._ingest_flow_frame(stamp, observations)

    @staticmethod
    def _fresh(stamp, timeout_s, now) -> bool:
        return stamp is not None and 0.0 <= (now - stamp).to_sec() <= timeout_s

    @staticmethod
    def _source_fresh(message, timeout_s, now) -> bool:
        if message is None or message.header.stamp.to_sec() <= 0.0:
            return False
        age = (now - message.header.stamp).to_sec()
        return 0.0 <= age <= timeout_s

    def _prune_observation_history(self, now_s: float) -> None:
        cutoff = float(now_s) - self.observation_history_s
        while self.observation_history and self.observation_history[0][0] < cutoff:
            self.observation_history.pop(0)
        if len(self.observation_history) > self.max_observation_frames:
            del self.observation_history[:-self.max_observation_frames]

    @staticmethod
    def _as_obstacle(observation) -> SimpleNamespace:
        return SimpleNamespace(
            id=observation["id"],
            center_x_map=observation["x"],
            center_y_map=observation["y"],
            velocity_x_map=observation["vx"],
            velocity_y_map=observation["vy"],
            length=observation["length"],
            width=observation["width"],
        )

    def _observation_at_time(self, observation, age_s: float):
        """Project one observation to now without depending on its tracker ID."""
        obstacle = self._as_obstacle(observation)
        age_s = max(0.0, float(age_s))
        if age_s <= 1e-6:
            return obstacle
        try:
            match = self.circulating.project(obstacle.center_x_map, obstacle.center_y_map)
        except (ValueError, ArithmeticError):
            return obstacle
        if match["distance_m"] <= self.track_corridor_m:
            tangent = match["tangent"]
            along = obstacle.velocity_x_map * tangent[0] + obstacle.velocity_y_map * tangent[1]
            if along > 0.0:
                progress = match["s_m"] + along * age_s
                if self.circulating_closed:
                    progress %= self.circulating.length
                elif progress > self.circulating.length:
                    return None
                x, y = self.circulating.point_at(progress)
                projected = self.circulating.project(
                    x, y, expected_s=progress, window_m=2.0
                )
                tangent = projected["tangent"]
                obstacle.center_x_map = x
                obstacle.center_y_map = y
                obstacle.velocity_x_map = tangent[0] * along
                obstacle.velocity_y_map = tangent[1] * along
                return obstacle
        # A track on another approach is not assigned to the circulating path.
        # Keep its short-horizon Cartesian motion so it can still guard the zone.
        obstacle.center_x_map += obstacle.velocity_x_map * age_s
        obstacle.center_y_map += obstacle.velocity_y_map * age_s
        return obstacle

    def _flow_progress(self, progress: float, delta_s: float) -> float:
        value = float(progress) + max(0.0, float(delta_s))
        if self.circulating_closed:
            return value % self.circulating.length
        return min(self.circulating.length, value)

    def _flow_arc_distance(self, a: float, b: float) -> float:
        distance = abs(float(a) - float(b))
        if self.circulating_closed:
            distance = min(distance, self.circulating.length - distance)
        return distance

    def _traffic_span(self, radius: float):
        # Dimensions can vary slightly across detections. Round the cached
        # radius upward so reuse never shrinks the occupied conflict region.
        radius_key = math.ceil(float(radius) * 10.0) / 10.0
        if radius_key in self.traffic_span_cache:
            return self.traffic_span_cache[radius_key]
        spans = self.circulating.circle_spans(self.conflict_xy, radius_key)
        if not spans:
            return None
        selected = _nearest_span(spans, self.traffic_conflict_s)
        if (self.circulating_closed and len(spans) > 1
                and spans[0][0] <= 1e-5
                and spans[-1][1] >= self.circulating.length - 1e-5
                and (selected == spans[0] or selected == spans[-1])):
            # A conflict disc can straddle the arbitrary start/end seam of a
            # closed MGeo link chain. Treat both pieces as one wrapped interval.
            selected = (spans[-1][0], spans[0][1] + self.circulating.length)
        self.traffic_span_cache[radius_key] = selected
        return selected

    def _flow_span(self, length: float, width: float):
        footprint = 0.5 * math.hypot(max(0.1, float(length)), max(0.1, float(width)))
        return self._traffic_span(
            self.conflict_radius_m + footprint + self.extra_obstacle_radius_m
        )

    def _advance_flow_to(self, stamp_s: float) -> None:
        if self.last_flow_update_s is None:
            self.last_flow_update_s = float(stamp_s)
            return
        delta_s = max(0.0, float(stamp_s) - self.last_flow_update_s)
        if delta_s > 0.0:
            for hypothesis in self.flow_hypotheses:
                hypothesis["s"] = self._flow_progress(
                    hypothesis["s"], hypothesis["speed"] * delta_s
                )
        self.last_flow_update_s = max(self.last_flow_update_s, float(stamp_s))

    def _release_passed_hypotheses(self) -> None:
        remaining = []
        for hypothesis in self.flow_hypotheses:
            span = self._flow_span(hypothesis["length"], hypothesis["width"])
            if span is None:
                remaining.append(hypothesis)
                continue
            # A closed circulating path wraps from its end to the beginning.
            # Release only while the vehicle is on the downstream side of this
            # conflict arc; a later lap can then be observed as a new approach.
            if span[1] > self.circulating.length:
                wrapped_end = span[1] - self.circulating.length
                passed = wrapped_end < hypothesis["s"] < span[0]
            else:
                passed = span[1] < hypothesis["s"] < self.circulating.length
            if passed:
                continue
            remaining.append(hypothesis)
        self.flow_hypotheses = remaining

    def _candidate_flow_tracks(self, observations):
        candidates = []
        for observation in observations:
            obstacle = self._as_obstacle(observation)
            values = (obstacle.center_x_map, obstacle.center_y_map,
                      obstacle.velocity_x_map, obstacle.velocity_y_map,
                      obstacle.length, obstacle.width)
            if not all(math.isfinite(value) for value in values):
                continue
            match = self.circulating.project(obstacle.center_x_map, obstacle.center_y_map)
            if match["distance_m"] > self.track_corridor_m:
                continue
            along = (obstacle.velocity_x_map * match["tangent"][0]
                     + obstacle.velocity_y_map * match["tangent"][1])
            candidates.append({
                "id": obstacle.id,
                "s": match["s_m"],
                "speed": max(0.0, min(25.0, along)),
                "length": max(0.1, obstacle.length),
                "width": max(0.1, obstacle.width),
            })
        # Collapse same-frame fragments at nearly identical route positions.
        # This is deliberately geometric; tracker IDs are diagnostic only.
        candidates.sort(key=lambda item: item["s"])
        compacted = []
        for candidate in candidates:
            duplicate = next((item for item in compacted
                              if self._flow_arc_distance(item["s"], candidate["s"]) <= 1.5), None)
            if duplicate is None:
                compacted.append(candidate)
            else:
                duplicate["length"] = max(duplicate["length"], candidate["length"])
                duplicate["width"] = max(duplicate["width"], candidate["width"])
                duplicate["speed"] = max(duplicate["speed"], candidate["speed"])
        return compacted

    def _ingest_flow_frame(self, stamp_s: float, observations) -> None:
        """Associate anonymous observations by predicted route position."""
        stamp_s = float(stamp_s)
        if self.last_flow_frame_stamp is not None and stamp_s <= self.last_flow_frame_stamp:
            return
        self._advance_flow_to(stamp_s)
        self._release_passed_hypotheses()
        candidates = self._candidate_flow_tracks(observations)
        pairs = []
        for candidate_index, candidate in enumerate(candidates):
            for hypothesis_index, hypothesis in enumerate(self.flow_hypotheses):
                distance = self._flow_arc_distance(candidate["s"], hypothesis["s"])
                age = max(0.0, stamp_s - hypothesis["last_seen"])
                gate = min(12.0, max(5.0, 2.0 + 0.5 * hypothesis["speed"] * age))
                if distance <= gate:
                    pairs.append((distance, candidate_index, hypothesis_index))
        pairs.sort()
        assigned_candidates, assigned_hypotheses = set(), set()
        for _, candidate_index, hypothesis_index in pairs:
            if candidate_index in assigned_candidates or hypothesis_index in assigned_hypotheses:
                continue
            candidate = candidates[candidate_index]
            hypothesis = self.flow_hypotheses[hypothesis_index]
            measured_speed = max(0.0, min(25.0, candidate["speed"]))
            # Limit one-frame speed jumps, then smooth sensor noise.
            measured_speed = min(hypothesis["speed"] + 1.5,
                                 max(0.0, hypothesis["speed"] - 1.5, measured_speed))
            hypothesis["speed"] = 0.65 * hypothesis["speed"] + 0.35 * measured_speed
            hypothesis["s"] = candidate["s"]
            hypothesis["last_seen"] = stamp_s
            hypothesis["id"] = candidate["id"]
            hypothesis["length"] = max(hypothesis["length"], candidate["length"])
            hypothesis["width"] = max(hypothesis["width"], candidate["width"])
            assigned_candidates.add(candidate_index)
            assigned_hypotheses.add(hypothesis_index)
        for candidate_index, candidate in enumerate(candidates):
            if candidate_index in assigned_candidates:
                continue
            self.flow_hypotheses.append({
                **candidate,
                "last_seen": stamp_s,
            })
        self.last_flow_frame_stamp = stamp_s

    def _flow_interval(self, hypothesis, now_s: float):
        age = max(0.0, float(now_s) - self.last_flow_update_s)
        progress = self._flow_progress(hypothesis["s"], hypothesis["speed"] * age)
        span = self._flow_span(hypothesis["length"], hypothesis["width"])
        if span is None:
            return None
        if self.circulating_closed and span[1] > self.circulating.length:
            if progress <= span[1] - self.circulating.length:
                progress += self.circulating.length
        if self.circulating_closed:
            if progress < span[0]:
                distance_in, distance_out = span[0] - progress, span[1] - progress
            elif progress <= span[1]:
                distance_in, distance_out = 0.0, span[1] - progress
            else:
                distance_in = self.circulating.length - progress + span[0]
                distance_out = self.circulating.length - progress + span[1]
        else:
            if progress > span[1]:
                return None
            distance_in = max(0.0, span[0] - progress)
            distance_out = max(0.0, span[1] - progress)
        if hypothesis["speed"] <= 0.25:
            x, y = self.circulating.point_at(progress)
            if math.hypot(x - self.conflict_xy[0], y - self.conflict_xy[1]) <= self.unknown_track_range_m:
                return (0.0, self.prediction_horizon_s)
            return None
        fastest = hypothesis["speed"] + self.track_speed_uncertainty_mps
        if distance_in / fastest > self.prediction_horizon_s:
            return None
        earliest = distance_in / fastest
        latest = distance_out / max(0.25, hypothesis["speed"] - self.track_speed_uncertainty_mps)
        return earliest, min(self.prediction_horizon_s, latest)

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
        span = self._traffic_span(radius)
        progress = match["s_m"]
        if self.circulating_closed and span is not None and span[1] > self.circulating.length:
            if progress <= span[1] - self.circulating.length:
                progress += self.circulating.length
        if span is None:
            return None
        if not self.circulating_closed and progress > span[1]:
            return None
        along = vx * match["tangent"][0] + vy * match["tangent"][1]
        if along <= 0.25:
            return ((0.0, self.prediction_horizon_s)
                    if distance_to_conflict <= self.unknown_track_range_m else None)
        if self.circulating_closed:
            if progress < span[0]:
                distance_in, distance_out = span[0] - progress, span[1] - progress
            elif progress <= span[1]:
                distance_in, distance_out = 0.0, span[1] - progress
            else:
                distance_in = self.circulating.length - progress + span[0]
                distance_out = self.circulating.length - progress + span[1]
        else:
            distance_in = max(0.0, span[0] - progress)
            distance_out = max(0.0, span[1] - progress)
        earliest = distance_in / (along + self.track_speed_uncertainty_mps)
        latest = distance_out / max(0.25, along - self.track_speed_uncertainty_mps)
        if earliest > self.prediction_horizon_s:
            return None
        return earliest, min(self.prediction_horizon_s, latest)

    def _blocking_objects(self, ego_interval) -> List[dict]:
        now_s = rospy.Time.now().to_sec()
        with self.observation_lock:
            self._prune_observation_history(now_s)
            blocker_windows = []

            def add_if_conflicting(obstacle, interval, source, speed):
                if interval is None or not intervals_overlap(
                        ego_interval, interval, self.temporal_margin_s):
                    return
                if not math.isfinite(speed):
                    speed = None
                # Repeated frames and tracker ID changes describe the same occupied
                # time window. Coalesce those windows for status output; the gate
                # decision still sees every individual interval above.
                for blocker in blocker_windows:
                    existing = (blocker["traffic_entry_s"], blocker["traffic_exit_s"])
                    if intervals_overlap(existing, interval, 0.25):
                        blocker["traffic_entry_s"] = round(min(existing[0], interval[0]), 2)
                        blocker["traffic_exit_s"] = round(max(existing[1], interval[1]), 2)
                        blocker["source_count"] += 1
                        if speed is not None:
                            old_speed = blocker["speed_mps"]
                            blocker["speed_mps"] = round(
                                speed if old_speed is None else max(old_speed, speed), 2
                            )
                        if source not in blocker["sources"]:
                            blocker["sources"].append(source)
                        return
                blocker_windows.append({
                    "id": int(obstacle.id),
                    "traffic_entry_s": round(interval[0], 2),
                    "traffic_exit_s": round(interval[1], 2),
                    "speed_mps": round(speed, 2) if speed is not None else None,
                    "source_count": 1,
                    "sources": [source],
                })

            for stamp_s, observations in self.observation_history:
                age_s = now_s - stamp_s
                if age_s < 0.0 or age_s > self.observation_history_s:
                    continue
                for observation in observations:
                    obstacle = self._observation_at_time(observation, age_s)
                    if obstacle is None:
                        continue
                    interval = self._traffic_interval(obstacle)
                    speed = math.hypot(obstacle.velocity_x_map, obstacle.velocity_y_map)
                    add_if_conflicting(obstacle, interval, "observation_history", speed)

            for hypothesis in self.flow_hypotheses:
                interval = self._flow_interval(hypothesis, now_s)
                obstacle = SimpleNamespace(id=hypothesis["id"])
                add_if_conflicting(obstacle, interval, "occlusion_prediction", hypothesis["speed"])
            return blocker_windows

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
