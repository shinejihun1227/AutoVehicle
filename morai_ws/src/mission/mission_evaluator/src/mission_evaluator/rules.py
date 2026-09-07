"""ROS-independent mission rule state machines."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional


@dataclass
class PenaltyEvent:
    rule: str
    penalty_sec: float
    reason: str
    timestamp_sec: float
    details: Dict[str, object]

    def as_dict(self) -> Dict[str, object]:
        return {
            "rule": self.rule,
            "penalty_sec": self.penalty_sec,
            "reason": self.reason,
            "timestamp_sec": self.timestamp_sec,
            "details": self.details,
        }


class RegionManager:
    def __init__(self, regions: Optional[Dict[str, Iterable[dict]]] = None) -> None:
        self.regions = {}
        for name, entries in (regions or {}).items():
            normalized = []
            for entry in entries or []:
                if "start_progress_m" not in entry or "end_progress_m" not in entry:
                    continue
                start = float(entry["start_progress_m"])
                end = float(entry["end_progress_m"])
                if end < start:
                    start, end = end, start
                normalized.append((start, end))
            self.regions[str(name)] = normalized

    def active(self, name: str, progress_m: Optional[float]) -> bool:
        if progress_m is None:
            return False
        return any(start <= float(progress_m) <= end for start, end in self.regions.get(name, []))

    def active_names(self, progress_m: Optional[float]) -> List[str]:
        return [name for name in self.regions if self.active(name, progress_m)]


class StartRule:
    WAITING = "WAITING"
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"

    def __init__(self, deadline_sec: float = 60.0, progress_ratio: float = 0.05,
                 start_speed_kph: float = 0.5, route_length_m: float = 0.0) -> None:
        self.deadline_sec = max(0.0, float(deadline_sec))
        self.progress_ratio = max(0.0, float(progress_ratio))
        self.start_speed_kph = max(0.0, float(start_speed_kph))
        self.route_length_m = max(0.0, float(route_length_m))
        self.state = self.WAITING
        self.departure_time: Optional[float] = None

    @property
    def target_progress_m(self) -> Optional[float]:
        if self.route_length_m <= 0.0:
            return None
        return self.route_length_m * self.progress_ratio

    def update(self, now_sec: float, progress_m: Optional[float], speed_kph: Optional[float]) -> Optional[PenaltyEvent]:
        now = float(now_sec)
        speed = 0.0 if speed_kph is None else max(0.0, float(speed_kph))
        if self.state == self.WAITING and speed >= self.start_speed_kph:
            self.departure_time = now
            self.state = self.RUNNING
        if self.state != self.RUNNING or self.departure_time is None:
            return None
        target = self.target_progress_m
        if target is not None and progress_m is not None and float(progress_m) >= target:
            self.state = self.SUCCESS
            return None
        if now - self.departure_time > self.deadline_sec:
            self.state = self.FAILED
            return PenaltyEvent(
                "start", 0.0, "start_progress_deadline_missed", now,
                {"deadline_sec": self.deadline_sec, "progress_m": progress_m, "target_progress_m": target},
            )
        return None


class SpeedRule:
    def __init__(self, limit_kph: float = 60.0, immediate_penalty_sec: float = 15.0,
                 repeat_interval_sec: float = 3.0) -> None:
        self.limit_kph = float(limit_kph)
        self.immediate_penalty_sec = float(immediate_penalty_sec)
        self.repeat_interval_sec = max(0.01, float(repeat_interval_sec))
        self.over_start: Optional[float] = None
        self.next_repeat: Optional[float] = None

    def update(self, now_sec: float, speed_kph: float, exception_active: bool = False) -> List[PenaltyEvent]:
        now = float(now_sec)
        if exception_active or float(speed_kph) <= self.limit_kph:
            self.over_start = None
            self.next_repeat = None
            return []
        if self.over_start is None:
            self.over_start = now
            self.next_repeat = now + self.repeat_interval_sec
            return [PenaltyEvent("speed", self.immediate_penalty_sec, "speed_limit_exceeded", now, {
                "speed_kph": float(speed_kph), "limit_kph": self.limit_kph,
            })]
        events = []
        while self.next_repeat is not None and now >= self.next_repeat:
            events.append(PenaltyEvent("speed", self.immediate_penalty_sec,
                                       "speed_limit_exceeded_sustained", self.next_repeat, {
                                           "speed_kph": float(speed_kph), "limit_kph": self.limit_kph,
                                           "sustained_sec": self.next_repeat - self.over_start,
                                       }))
            self.next_repeat += self.repeat_interval_sec
        return events


class LaneContactRule:
    def __init__(self, contact_duration_sec: float = 3.0, repeat_interval_sec: float = 3.0,
                 penalty_sec: float = 5.0) -> None:
        self.contact_duration_sec = max(0.01, float(contact_duration_sec))
        self.repeat_interval_sec = max(0.01, float(repeat_interval_sec))
        self.penalty_sec = float(penalty_sec)
        self.contact_start: Optional[float] = None
        self.next_penalty: Optional[float] = None

    def update(self, now_sec: float, contact: bool, exempt: bool = False) -> List[PenaltyEvent]:
        now = float(now_sec)
        if exempt or not contact:
            self.contact_start = None
            self.next_penalty = None
            return []
        if self.contact_start is None:
            self.contact_start = now
            self.next_penalty = now + self.contact_duration_sec
            return []
        events = []
        while self.next_penalty is not None and now >= self.next_penalty:
            events.append(PenaltyEvent("lane", self.penalty_sec, "lane_contact_sustained",
                                       self.next_penalty, {
                                           "contact_duration_sec": self.next_penalty - self.contact_start,
                                       }))
            self.next_penalty += self.repeat_interval_sec
        return events


class CollisionRule:
    def __init__(self, penalty_sec: float = 15.0, release_timeout_sec: float = 0.5) -> None:
        self.penalty_sec = float(penalty_sec)
        self.release_timeout_sec = max(0.0, float(release_timeout_sec))
        self.active_since: Dict[str, float] = {}
        self.last_seen: Dict[str, float] = {}

    def update(self, now_sec: float, object_keys: Iterable[str]) -> List[PenaltyEvent]:
        now = float(now_sec)
        current = {str(key) for key in object_keys if str(key)}
        events = []
        for key in current:
            previous_seen = self.last_seen.get(key)
            recontact = previous_seen is not None and now - previous_seen > self.release_timeout_sec
            if key not in self.active_since or recontact:
                self.active_since[key] = now
                events.append(PenaltyEvent("collision", self.penalty_sec,
                                           "collision_detected" if not recontact else "collision_recontact",
                                           now, {"object_key": key}))
            self.last_seen[key] = now
        for key, last_seen in list(self.last_seen.items()):
            if key not in current and now - last_seen > self.release_timeout_sec:
                self.active_since.pop(key, None)
        return events
