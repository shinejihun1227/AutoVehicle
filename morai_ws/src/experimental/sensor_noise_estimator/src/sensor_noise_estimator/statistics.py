"""외부 의존성이 작은 robust 통계 함수."""

from __future__ import annotations

import math
from statistics import median
from typing import Iterable, List, Tuple


ROBUST_SCALE = 1.4826


def finite_values(values: Iterable[float]) -> List[float]:
    return [float(value) for value in values if math.isfinite(float(value))]


def robust_center_sigma(values: Iterable[float]) -> Tuple[float, float]:
    """median과 MAD 기반 표준편차 추정치를 반환한다.

    GPS outlier와 IMU spike가 섞여도 일반 표준편차보다 덜 흔들리도록 MAD를
    사용한다. 값이 완전히 일정하면 sigma=0을 유지한다.
    """

    clean = finite_values(values)
    if not clean:
        return math.nan, math.nan
    center = float(median(clean))
    deviations = [abs(value - center) for value in clean]
    sigma = ROBUST_SCALE * float(median(deviations))
    return center, sigma


def robust_speed_from_gps(
    samples: Iterable[Tuple[float, float, float]],
) -> float:
    """GPS 샘플 (timestamp, x, y)의 최근 이동속도 robust 추정치."""

    ordered = sorted(samples, key=lambda item: item[0])
    speeds = []
    for previous, current in zip(ordered, ordered[1:]):
        dt = current[0] - previous[0]
        if dt <= 1e-6:
            continue
        distance = math.hypot(current[1] - previous[1], current[2] - previous[2])
        speeds.append(distance / dt)
    if not speeds:
        return math.nan
    return float(median(speeds))


def sample_rate_and_max_gap(samples: Iterable[Tuple[float, ...]]) -> Tuple[float, float]:
    """timestamp가 첫 원소인 샘플에서 평균 rate와 최대 gap을 계산한다."""

    stamps = sorted(float(sample[0]) for sample in samples if math.isfinite(float(sample[0])))
    if len(stamps) < 2:
        return math.nan, math.nan
    gaps = [right - left for left, right in zip(stamps, stamps[1:]) if right > left]
    if not gaps:
        return math.nan, math.nan
    duration = stamps[-1] - stamps[0]
    rate = (len(stamps) - 1) / duration if duration > 1e-6 else math.nan
    return float(rate), float(max(gaps))
