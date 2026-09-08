"""ROS와 독립적인 방향지시등 이벤트 스케줄러."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Optional


OFF = "OFF"
LEFT = "LEFT"
RIGHT = "RIGHT"
VALID_DIRECTIONS = {LEFT, RIGHT}


def normalize_direction(value: object) -> str:
    text = str(value).strip().upper()
    aliases = {
        "L": LEFT,
        "LEFT": LEFT,
        "좌": LEFT,
        "좌회전": LEFT,
        "좌측": LEFT,
        "R": RIGHT,
        "RIGHT": RIGHT,
        "우": RIGHT,
        "우회전": RIGHT,
        "우측": RIGHT,
    }
    direction = aliases.get(text, text)
    if direction not in VALID_DIRECTIONS:
        raise ValueError(f"방향지시등 방향은 LEFT 또는 RIGHT여야 한다: {value!r}")
    return direction


@dataclass(frozen=True)
class Maneuver:
    """경로상의 하나의 차선 변경 또는 좌우회전 이벤트."""

    identifier: str
    start_s_m: float
    direction: str
    end_s_m: Optional[float] = None
    duration_sec: float = 8.0
    kind: str = "lane_change"

    def __post_init__(self) -> None:
        if not self.identifier:
            raise ValueError("maneuver identifier가 비어 있다.")
        if not math.isfinite(self.start_s_m) or self.start_s_m < 0.0:
            raise ValueError("maneuver start_s_m은 0 이상 유한한 값이어야 한다.")
        if self.direction not in VALID_DIRECTIONS:
            raise ValueError(f"지원하지 않는 방향이다: {self.direction}")
        if self.end_s_m is not None:
            if not math.isfinite(self.end_s_m) or self.end_s_m < self.start_s_m:
                raise ValueError("maneuver end_s_m은 start_s_m 이상이어야 한다.")
        if not math.isfinite(self.duration_sec) or self.duration_sec <= 0.0:
            raise ValueError("maneuver duration_sec은 0보다 커야 한다.")


@dataclass(frozen=True)
class SignalDecision:
    direction: str = OFF
    maneuver_id: Optional[str] = None
    phase: str = "idle"
    eta_sec: Optional[float] = None


def parse_maneuvers(raw: object, default_duration_sec: float = 8.0) -> List[Maneuver]:
    """ROS XmlRpc/YAML 리스트를 검증된 Maneuver 목록으로 변환한다."""

    if raw is None:
        return []
    if not isinstance(raw, (list, tuple)):
        raise ValueError("~maneuvers는 리스트여야 한다.")

    maneuvers: List[Maneuver] = []
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError(f"maneuvers[{index}]는 dictionary여야 한다.")
        start_value = item.get("start_s_m", item.get("distance_m"))
        if start_value is None:
            raise ValueError(f"maneuvers[{index}]에 start_s_m이 없다.")
        start_s_m = float(start_value)
        direction = normalize_direction(item.get("direction", ""))
        identifier = str(
            item.get(
                "id",
                f"{item.get('kind', 'maneuver')}_{direction.lower()}_{start_s_m:.3f}",
            )
        )
        end_value = item.get("end_s_m")
        end_s_m = None if end_value is None else float(end_value)
        duration_sec = float(item.get("duration_sec", default_duration_sec))
        kind = str(item.get("kind", "lane_change"))
        maneuvers.append(
            Maneuver(
                identifier=identifier,
                start_s_m=start_s_m,
                direction=direction,
                end_s_m=end_s_m,
                duration_sec=duration_sec,
                kind=kind,
            )
        )

    return sorted(maneuvers, key=lambda maneuver: maneuver.start_s_m)


class TurnSignalScheduler:
    """진행거리와 속도로 이벤트 시작 5초 전 방향지시등을 켠다.

    ``start_s_m``은 차선 변경 또는 회전 조작을 시작할 경로 누적거리다.
    ``end_s_m``이 있으면 차량이 그 위치를 지날 때 끄고, 없으면
    ``duration_sec`` 동안 유지한다. 속도가 낮을 때도 이벤트를 놓치지 않도록
    ``min_prediction_speed_mps``를 ETA 계산의 하한으로 사용한다.
    """

    def __init__(
        self,
        maneuvers: Iterable[Maneuver],
        lead_time_sec: float = 5.0,
        min_prediction_speed_mps: float = 0.5,
        progress_reset_m: float = 5.0,
    ) -> None:
        self.maneuvers = list(maneuvers)
        self.lead_time_sec = max(0.0, float(lead_time_sec))
        self.min_prediction_speed_mps = max(1e-3, float(min_prediction_speed_mps))
        self.progress_reset_m = max(0.0, float(progress_reset_m))
        self.completed_ids = set()
        self.active: Optional[Maneuver] = None
        self.active_since: Optional[float] = None
        self.last_progress_s: Optional[float] = None

    def reset(self) -> None:
        self.completed_ids.clear()
        self.active = None
        self.active_since = None
        self.last_progress_s = None

    def _complete_active(self) -> None:
        if self.active is not None:
            self.completed_ids.add(self.active.identifier)
        self.active = None
        self.active_since = None

    def _event_finished(self, event: Maneuver, progress_s: float, now: float) -> bool:
        if event.end_s_m is not None:
            return progress_s >= event.end_s_m
        return self.active_since is not None and now - self.active_since >= event.duration_sec

    def update(self, progress_s: float, speed_mps: float, now: float) -> SignalDecision:
        progress = float(progress_s)
        speed = max(0.0, float(speed_mps))
        timestamp = float(now)
        if not math.isfinite(progress) or not math.isfinite(speed) or not math.isfinite(timestamp):
            return SignalDecision()

        if (
            self.last_progress_s is not None
            and progress < self.last_progress_s - self.progress_reset_m
        ):
            # 새 주행을 시작하거나 시뮬레이터가 reset된 경우 이벤트를 다시 사용한다.
            self.reset()
        self.last_progress_s = progress

        if self.active is not None and self._event_finished(self.active, progress, timestamp):
            self._complete_active()

        if self.active is not None:
            return SignalDecision(
                direction=self.active.direction,
                maneuver_id=self.active.identifier,
                phase="active",
                eta_sec=0.0,
            )

        # end_s_m이 있는 과거 이벤트는 시작 전에 노드가 올라온 경우에도 건너뛴다.
        for event in self.maneuvers:
            if event.identifier in self.completed_ids:
                continue
            if event.end_s_m is not None and progress >= event.end_s_m:
                self.completed_ids.add(event.identifier)
                continue

            remaining = max(0.0, event.start_s_m - progress)
            eta = remaining / max(speed, self.min_prediction_speed_mps)
            if progress >= event.start_s_m or eta <= self.lead_time_sec:
                self.active = event
                self.active_since = timestamp
                return SignalDecision(
                    direction=event.direction,
                    maneuver_id=event.identifier,
                    phase="active" if progress >= event.start_s_m else "lead",
                    eta_sec=eta,
                )

            return SignalDecision(
                direction=OFF,
                maneuver_id=event.identifier,
                phase="armed",
                eta_sec=eta,
            )

        return SignalDecision()
