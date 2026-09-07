#!/usr/bin/env python3
"""차선 검출 결과의 주행 사용 가능성을 계산한다.

ROI 차선 모델은 argmax 세그멘테이션 결과를 내므로 모델 확률 하나만으로
조향 가능 여부를 판단하지 않는다. 좌우 차선의 가시성, 차선 폭, 검출 픽셀
수, 곡선 길이, 프레임 간 연속성을 합쳐 0~1 품질 점수를 만든다.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Dict, Optional


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def finite(value: Optional[float]) -> bool:
    return value is not None and math.isfinite(float(value))


class LaneQualityEstimator:
    """한 프레임의 차선 결과를 주행용 품질 지표로 변환한다."""

    def __init__(
        self,
        expected_lane_width_m: float = 3.3,
        lane_width_tolerance_m: float = 0.8,
        history_size: int = 5,
    ) -> None:
        self.expected_lane_width_m = max(0.1, float(expected_lane_width_m))
        self.lane_width_tolerance_m = max(0.1, float(lane_width_tolerance_m))
        self.history: Deque[Dict[str, float]] = deque(
            maxlen=max(1, int(history_size))
        )
        self.good_streak = 0
        self.bad_streak = 0

    @staticmethod
    def _line_score(line) -> float:
        if line is None:
            return 0.0
        point_score = clamp(float(getattr(line, "n_points", 0)) / 220.0, 0.0, 1.0)
        x_range = getattr(line, "x_range", (0.0, 0.0))
        span = max(0.0, float(x_range[1]) - float(x_range[0]))
        span_score = clamp(span / 10.0, 0.0, 1.0)
        return 0.5 * point_score + 0.5 * span_score

    def update(self, result) -> Dict[str, object]:
        left = result.ego_left
        right = result.ego_right
        lateral_error = result.lateral_error()
        heading_error = result.heading_error()
        left_visible = left is not None
        right_visible = right is not None
        both_visible = left_visible and right_visible

        lane_width_m = None
        if both_visible:
            left_y = left.y_at(7.0)
            right_y = right.y_at(7.0)
            if finite(left_y) and finite(right_y):
                lane_width_m = abs(float(left_y) - float(right_y))

        line_score = 0.5 * (
            self._line_score(left) + self._line_score(right)
        ) if both_visible else self._line_score(left or right)

        if both_visible and lane_width_m is not None:
            width_score = clamp(
                1.0
                - abs(lane_width_m - self.expected_lane_width_m)
                / self.lane_width_tolerance_m,
                0.0,
                1.0,
            )
        elif left_visible or right_visible:
            # 한쪽 차선은 폭을 검증할 수 없으므로 보조 조향까지만 허용한다.
            width_score = 0.45
        else:
            width_score = 0.0

        visibility_score = 1.0 if both_visible else 0.55 if (left_visible or right_visible) else 0.0

        continuity_score = 1.0
        if self.history and finite(lateral_error) and finite(heading_error):
            previous = self.history[-1]
            lateral_delta = abs(float(lateral_error) - previous["lateral_error"])
            heading_delta = abs(float(heading_error) - previous["heading_error"])
            continuity_score = 0.5 * (
                clamp(1.0 - lateral_delta / 0.75, 0.0, 1.0)
                + clamp(1.0 - heading_delta / 0.30, 0.0, 1.0)
            )
            if lane_width_m is not None and finite(previous.get("lane_width_m")):
                continuity_score = (
                    0.8 * continuity_score
                    + 0.2
                    * clamp(
                        1.0
                        - abs(lane_width_m - previous["lane_width_m"])
                        / 0.8,
                        0.0,
                        1.0,
                    )
                )

        valid = bool(
            finite(lateral_error)
            and finite(heading_error)
            and (left_visible or right_visible)
            and line_score >= 0.25
        )
        confidence = (
            0.35 * visibility_score
            + 0.25 * line_score
            + 0.20 * width_score
            + 0.20 * continuity_score
            if valid
            else 0.0
        )
        confidence = clamp(confidence, 0.0, 1.0)

        if valid and confidence >= 0.55:
            self.good_streak += 1
            self.bad_streak = 0
            self.history.append(
                {
                    "lateral_error": float(lateral_error),
                    "heading_error": float(heading_error),
                    "lane_width_m": float(lane_width_m)
                    if lane_width_m is not None
                    else math.nan,
                }
            )
        else:
            self.bad_streak += 1
            self.good_streak = 0

        return {
            "valid": valid,
            "confidence": confidence,
            "lateral_error": float(lateral_error)
            if finite(lateral_error)
            else None,
            "heading_error": float(heading_error)
            if finite(heading_error)
            else None,
            "lane_width_m": lane_width_m,
            "left_visible": left_visible,
            "right_visible": right_visible,
            "both_visible": both_visible,
            "line_score": line_score,
            "continuity_score": continuity_score,
            "good_streak": self.good_streak,
            "bad_streak": self.bad_streak,
        }
