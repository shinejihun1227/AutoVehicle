#!/usr/bin/env python3
"""대회 규정 검증용 독립 mission evaluator.

이 노드는 /ctrl_cmd를 구독하거나 발행하지 않는다. 즉, 실제 대회 주행 런치에
포함하지 않고 roslaunch로 별도 실행해 rosbag/MORAI 토픽의 규정 준수 결과만
기록한다.
"""

from __future__ import annotations

import json
import math
import os
import time
from typing import Any, Iterable, List, Optional

import rospy
from morai_msgs.msg import EgoVehicleStatus
from morai_perception_msgs.msg import SensorQuality
from std_msgs.msg import Bool, Float64, String

from mission_evaluator.rules import CollisionRule, LaneContactRule, RegionManager, SpeedRule, StartRule

try:
    from morai_msgs.msg import CollisionData
except ImportError:
    CollisionData = None


MPS_TO_KPH = 3.6


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _iter_values(value: Any, depth: int = 0) -> Iterable[Any]:
    if depth > 3 or value is None:
        return
    if isinstance(value, (list, tuple)):
        for item in value:
            yield from _iter_values(item, depth + 1)
        return
    yield value


def extract_collision_keys(message: Any) -> List[str]:
    """MORAI 배포판별 CollisionData 배열 필드 차이를 흡수한다."""
    containers = []
    for name in ("data", "_data", "collisions", "objects"):
        if hasattr(message, name):
            value = getattr(message, name)
            if isinstance(value, (list, tuple)):
                containers.extend(value)
    if not containers:
        containers = [message]

    keys = []
    for item in _iter_values(containers):
        found = None
        for name in ("obj_id", "object_id", "unique_id", "id"):
            if hasattr(item, name):
                candidate = getattr(item, name)
                if candidate not in (None, 0, "", "0"):
                    found = str(candidate)
                    break
        if found is not None:
            key = "object:" + found
            if key not in keys:
                keys.append(key)
    return keys


def calculate_route_length(path_file: str) -> float:
    """x y [z] 행으로 된 현재 MGeo 경로의 누적 길이를 계산한다."""
    if not path_file or not os.path.isfile(path_file):
        return 0.0
    points = []
    with open(path_file, "r", encoding="utf-8") as stream:
        for line in stream:
            values = line.split()
            if len(values) < 2:
                continue
            try:
                points.append((float(values[0]), float(values[1])))
            except ValueError:
                continue
    return sum(
        math.hypot(x1 - x0, y1 - y0)
        for (x0, y0), (x1, y1) in zip(points, points[1:])
    )


class MissionEvaluator:
    def __init__(self) -> None:
        rospy.init_node("mission_evaluator", anonymous=False)
        rules = rospy.get_param("~rules", {})
        route_length_m = float(rospy.get_param("~route_length_m", 0.0))
        path_file = rospy.get_param("~path_file", "")
        if route_length_m <= 0.0:
            route_length_m = calculate_route_length(path_file)
        start = rules.get("start", {})
        speed = rules.get("speed", {})
        lane = rules.get("lane", {})
        collision = rules.get("collision", {})
        self.regions = RegionManager(rules.get("regions", {}))
        self.start_rule = StartRule(
            start.get("deadline_sec", 60.0), start.get("progress_ratio", 0.05),
            start.get("start_speed_kph", 0.5), route_length_m,
        )
        self.speed_rule = SpeedRule(
            speed.get("limit_kph", 60.0), speed.get("immediate_penalty_sec", 15.0),
            speed.get("repeat_interval_sec", 3.0),
        )
        self.lane_rule = LaneContactRule(
            lane.get("contact_duration_sec", 3.0), lane.get("repeat_interval_sec", 3.0),
            lane.get("penalty_sec", 5.0),
        )
        self.collision_rule = CollisionRule(
            collision.get("penalty_sec", 15.0), collision.get("release_timeout_sec", 0.5),
        )
        self.progress_topic = rospy.get_param("~progress_topic", "/experimental/curvature_progress")
        self.ego_topic = rospy.get_param("~ego_topic", "/Ego_topic")
        self.quality_topic = rospy.get_param("~quality_topic", "/localization/sensor_quality")
        self.lane_contact_topic = rospy.get_param("~lane_contact_topic", "/mission/lane_contact")
        self.collision_topic = rospy.get_param("~collision_topic", "/CollisionData")
        self.status_topic = rospy.get_param("~status_topic", "/mission/status")
        self.event_topic = rospy.get_param("~event_topic", "/mission/penalty_event")
        self.summary_topic = rospy.get_param("~summary_topic", "/mission/summary")
        self.region_topic = rospy.get_param("~region_topic", "/mission/current_region")
        self.rate_hz = max(1.0, float(rospy.get_param("~rate_hz", 20.0)))
        self.last_progress: Optional[float] = None
        self.last_speed_kph: Optional[float] = None
        self.last_quality: Optional[SensorQuality] = None
        self.last_lane_contact = False
        self.lane_contact_seen = False
        self.collision_keys: List[str] = []
        self.total_penalty_sec = 0.0
        self.events = []
        self.last_ego_time = 0.0
        self.last_progress_time = 0.0
        self.last_quality_time = 0.0
        self.last_lane_contact_time = 0.0
        self.last_collision_time = 0.0
        self.collision_available = CollisionData is not None

        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=5, latch=True)
        self.event_pub = rospy.Publisher(self.event_topic, String, queue_size=20)
        self.summary_pub = rospy.Publisher(self.summary_topic, String, queue_size=1, latch=True)
        self.region_pub = rospy.Publisher(self.region_topic, String, queue_size=5)
        rospy.Subscriber(self.progress_topic, Float64, self.progress_callback, queue_size=10)
        rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.ego_callback, queue_size=10)
        rospy.Subscriber(self.quality_topic, SensorQuality, self.quality_callback, queue_size=10)
        rospy.Subscriber(self.lane_contact_topic, Bool, self.lane_contact_callback, queue_size=10)
        if CollisionData is not None:
            rospy.Subscriber(self.collision_topic, CollisionData, self.collision_callback, queue_size=20)
        else:
            rospy.logwarn("morai_msgs/CollisionData를 찾지 못해 충돌 평가는 비활성화됩니다.")
        if route_length_m <= 0.0:
            rospy.logwarn("경로 길이를 계산하지 못했습니다. start 5%% 판정은 CONFIG_MISSING 상태입니다.")
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.evaluate)

    def progress_callback(self, message: Float64) -> None:
        self.last_progress = float(message.data) if finite(message.data) else None
        self.last_progress_time = time.monotonic()

    def ego_callback(self, message: EgoVehicleStatus) -> None:
        vx = float(message.velocity.x)
        vy = float(message.velocity.y)
        speed = math.hypot(vx, vy)
        self.last_speed_kph = speed * MPS_TO_KPH if finite(speed) else None
        self.last_ego_time = time.monotonic()

    def quality_callback(self, message: SensorQuality) -> None:
        self.last_quality = message
        self.last_quality_time = time.monotonic()

    def lane_contact_callback(self, message: Bool) -> None:
        self.last_lane_contact = bool(message.data)
        self.lane_contact_seen = True
        self.last_lane_contact_time = time.monotonic()

    def collision_callback(self, message: Any) -> None:
        self.collision_keys = extract_collision_keys(message)
        self.last_collision_time = time.monotonic()

    def gps_blackout(self) -> bool:
        if self.last_quality is None or time.monotonic() - self.last_quality_time > 0.7:
            return False
        return bool(self.last_quality.gps_blackout or self.last_quality.state == "GPS_BLACKOUT")

    def add_events(self, events) -> None:
        for event in events:
            self.total_penalty_sec += float(event.penalty_sec)
            self.events.append(event.as_dict())
            self.event_pub.publish(String(data=json.dumps(event.as_dict(), ensure_ascii=False, sort_keys=True)))

    def evaluate(self, _event) -> None:
        now = rospy.get_time()
        blackout_region = self.regions.active("blackout", self.last_progress)
        highway_exception = self.regions.active("highway_exception", self.last_progress)
        lane_exempt = blackout_region or self.gps_blackout()

        start_event = self.start_rule.update(now, self.last_progress, self.last_speed_kph)
        if start_event is not None:
            self.add_events([start_event])
        self.add_events(self.speed_rule.update(now, self.last_speed_kph or 0.0, highway_exception))
        lane_contact_fresh = self.lane_contact_seen and time.monotonic() - self.last_lane_contact_time <= 0.7
        self.add_events(self.lane_rule.update(now, self.last_lane_contact if lane_contact_fresh else False, lane_exempt))
        collision_fresh = time.monotonic() - self.last_collision_time <= 0.7
        self.add_events(self.collision_rule.update(now, self.collision_keys if collision_fresh else []))

        active_regions = self.regions.active_names(self.last_progress)
        self.region_pub.publish(String(data=json.dumps({
            "progress_m": self.last_progress,
            "active_regions": active_regions,
            "highway_speed_exception": highway_exception,
            "blackout_region": blackout_region,
            "gps_blackout": self.gps_blackout(),
            "lane_penalty_exempt": lane_exempt,
        }, ensure_ascii=False, sort_keys=True)))
        status = {
            "start_state": self.start_rule.state,
            "start_elapsed_sec": (
                None if self.start_rule.departure_time is None
                else now - self.start_rule.departure_time
            ),
            "start_target_progress_m": self.start_rule.target_progress_m,
            "progress_m": self.last_progress,
            "speed_kph": self.last_speed_kph,
            "speed_limit_exception": highway_exception,
            "gps_blackout": self.gps_blackout(),
            "blackout_region": blackout_region,
            "lane_penalty_exempt": lane_exempt,
            "lane_contact_input_available": self.lane_contact_seen,
            "collision_input_available": self.collision_available,
            "total_penalty_sec": self.total_penalty_sec,
            "event_count": len(self.events),
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False, sort_keys=True)))
        self.summary_pub.publish(String(data=json.dumps({
            "total_penalty_sec": self.total_penalty_sec,
            "events": self.events[-100:],
            "start_state": self.start_rule.state,
        }, ensure_ascii=False, sort_keys=True)))


if __name__ == "__main__":
    try:
        MissionEvaluator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
