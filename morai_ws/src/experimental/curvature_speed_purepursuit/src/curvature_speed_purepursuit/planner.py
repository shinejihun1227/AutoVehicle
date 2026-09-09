"""ROS에 의존하지 않는 곡률·속도 프로파일 계산 모듈.

이 모듈은 기존 purepursuit_mgeo 패키지를 가져오지 않는다. 따라서
기존 경로 처리·제어 코드와 독립적으로 단위 테스트할 수 있다.
"""

from __future__ import annotations

import math
from bisect import bisect_right
from dataclasses import dataclass
from statistics import median
from typing import Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True)
class PathProjection:
    """차량 위치를 경로 선분에 투영한 결과."""

    segment_index: int
    ratio: float
    progress_s: float
    distance_m: float


def load_path_file(path_file: str) -> List[PathPoint]:
    """공백·쉼표로 구분된 x y [z] 경로를 읽는다."""

    points: List[PathPoint] = []
    with open(path_file, "r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.replace(",", " ").split()
            if len(fields) < 2:
                continue
            try:
                x = float(fields[0])
                y = float(fields[1])
                z = float(fields[2]) if len(fields) >= 3 else 0.0
            except ValueError as exc:
                raise ValueError(
                    f"경로 파일 {line_number}번째 줄을 읽을 수 없다: {raw_line!r}"
                ) from exc
            points.append(PathPoint(x, y, z))

    if len(points) < 2:
        raise ValueError(f"경로에 사용할 점이 부족하다: {path_file}")
    return points


def clean_consecutive_duplicates(
    points: Sequence[PathPoint], epsilon_m: float = 1e-6
) -> List[PathPoint]:
    """연속 중복점을 제거한다.

    첫 점과 마지막 점이 같은 폐곡선의 종료 표시는 유지한다. 즉, 이 함수는
    연속으로 반복된 점만 제거하며 비연속적인 첫·마지막 점은 삭제하지 않는다.
    """

    if not points:
        return []

    cleaned = [points[0]]
    for point in points[1:]:
        previous = cleaned[-1]
        if math.hypot(point.x - previous.x, point.y - previous.y) <= epsilon_m:
            continue
        cleaned.append(point)

    if len(cleaned) < 2:
        raise ValueError("중복점 제거 후 경로점이 부족하다.")
    return cleaned


def cumulative_arc_lengths(points: Sequence[PathPoint]) -> List[float]:
    """XY 평면 기준 누적 경로거리 s를 계산한다."""

    if len(points) < 2:
        raise ValueError("누적거리를 계산하려면 경로점이 2개 이상 필요하다.")

    distances = [0.0]
    for previous, current in zip(points, points[1:]):
        distance = math.hypot(current.x - previous.x, current.y - previous.y)
        if distance <= 1e-9:
            raise ValueError("연속 중복점이 남아 있어 누적거리를 계산할 수 없다.")
        distances.append(distances[-1] + distance)
    return distances


def _cross_2d(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * by - ay * bx


def three_point_curvature(
    points: Sequence[PathPoint], index: int, half_window_points: int = 1
) -> float:
    """세 점의 외접원을 이용해 부호 있는 곡률을 계산한다.

    양수는 map ENU에서 좌회전, 음수는 우회전이다. 끝점은 가운데 점을
    기준으로 양쪽 점을 확보할 수 없으므로 0을 반환한다.
    """

    window = max(1, int(half_window_points))
    left_index = index - window
    right_index = index + window
    if left_index < 0 or right_index >= len(points):
        return 0.0

    left = points[left_index]
    center = points[index]
    right = points[right_index]
    v1x = center.x - left.x
    v1y = center.y - left.y
    v2x = right.x - center.x
    v2y = right.y - center.y
    a = math.hypot(v1x, v1y)
    b = math.hypot(v2x, v2y)
    c = math.hypot(right.x - left.x, right.y - left.y)
    if min(a, b, c) <= 1e-9:
        return 0.0
    return 2.0 * _cross_2d(v1x, v1y, v2x, v2y) / (a * b * c)


def median_smooth(values: Sequence[float], window_size: int) -> List[float]:
    """곡률 배열에 1차원 median smoothing을 적용한다."""

    if not values:
        return []
    window = max(1, int(window_size))
    if window % 2 == 0:
        window += 1
    radius = window // 2
    smoothed: List[float] = []
    for index in range(len(values)):
        start = max(0, index - radius)
        end = min(len(values), index + radius + 1)
        smoothed.append(float(median(values[start:end])))
    return smoothed


def curvature_profile(
    points: Sequence[PathPoint],
    half_window_points: int = 1,
    smoothing_window: int = 5,
) -> List[float]:
    """경로 전체의 3점 기반 곡률과 median smoothing 결과를 반환한다."""

    raw = [
        three_point_curvature(points, index, half_window_points)
        for index in range(len(points))
    ]
    return median_smooth(raw, smoothing_window)


def interpolate_by_s(
    points: Sequence[PathPoint], s_values: Sequence[float], query_s: float
) -> Tuple[PathPoint, int]:
    """누적거리 s 위치의 경로점을 선형 보간한다."""

    if len(points) != len(s_values) or len(points) < 2:
        raise ValueError("points와 s_values의 길이가 올바르지 않다.")

    query = max(0.0, min(float(query_s), s_values[-1]))
    if query >= s_values[-1]:
        return points[-1], len(points) - 2

    segment = max(0, min(len(points) - 2, bisect_right(s_values, query) - 1))
    segment_length = s_values[segment + 1] - s_values[segment]
    ratio = (query - s_values[segment]) / max(segment_length, 1e-9)
    start = points[segment]
    end = points[segment + 1]
    return (
        PathPoint(
            start.x + ratio * (end.x - start.x),
            start.y + ratio * (end.y - start.y),
            start.z + ratio * (end.z - start.z),
        ),
        segment,
    )


def nearest_projection(
    points: Sequence[PathPoint],
    s_values: Sequence[float],
    x: float,
    y: float,
    start_segment: int = 0,
    end_segment: Optional[int] = None,
) -> PathProjection:
    """차량 위치를 지정 범위의 경로 선분에 투영한다."""

    if len(points) < 2:
        raise ValueError("투영할 경로점이 부족하다.")
    first = max(0, min(int(start_segment), len(points) - 2))
    last = len(points) - 2 if end_segment is None else max(
        first, min(int(end_segment), len(points) - 2)
    )

    best: Optional[PathProjection] = None
    for index in range(first, last + 1):
        start = points[index]
        end = points[index + 1]
        dx = end.x - start.x
        dy = end.y - start.y
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            continue
        ratio = ((x - start.x) * dx + (y - start.y) * dy) / length_sq
        ratio = max(0.0, min(1.0, ratio))
        projected_x = start.x + ratio * dx
        projected_y = start.y + ratio * dy
        distance = math.hypot(x - projected_x, y - projected_y)
        progress = s_values[index] + ratio * (s_values[index + 1] - s_values[index])
        candidate = PathProjection(index, ratio, progress, distance)
        if best is None or candidate.distance_m < best.distance_m:
            best = candidate

    if best is None:
        raise ValueError("유효한 경로 선분을 찾지 못했다.")
    return best


def profile_value_at_s(
    s_values: Sequence[float], values: Sequence[float], query_s: float
) -> float:
    """누적거리 s 위치의 프로파일 값을 선형 보간한다."""

    if len(s_values) != len(values) or not values:
        raise ValueError("프로파일 길이가 올바르지 않다.")
    query = max(0.0, min(float(query_s), s_values[-1]))
    if query >= s_values[-1]:
        return float(values[-1])
    index = max(0, min(len(values) - 2, bisect_right(s_values, query) - 1))
    ds = s_values[index + 1] - s_values[index]
    ratio = (query - s_values[index]) / max(ds, 1e-9)
    return float(values[index] + ratio * (values[index + 1] - values[index]))


def max_abs_curvature_ahead(
    s_values: Sequence[float],
    curvatures: Sequence[float],
    query_s: float,
    preview_distance_m: float,
    sample_step_m: float = 0.5,
) -> float:
    """Return the largest absolute curvature in a forward path preview.

    Curvature is calculated at discrete path points, while the controller runs
    at an arbitrary arc-length position.  Sampling the interpolated profile
    avoids missing a short, sharp bend between two control cycles.  The
    returned value is always finite and nonnegative for a valid profile.
    """

    if len(s_values) != len(curvatures) or len(s_values) < 2:
        raise ValueError("곡률 미리보기 입력 길이가 올바르지 않다.")
    if not all(math.isfinite(float(value)) for value in s_values):
        raise ValueError("곡률 미리보기 누적거리가 유한하지 않다.")
    if not all(math.isfinite(float(value)) for value in curvatures):
        raise ValueError("곡률 미리보기 값이 유한하지 않다.")

    start = max(0.0, min(float(query_s), float(s_values[-1])))
    distance = max(0.0, float(preview_distance_m))
    end = min(float(s_values[-1]), start + distance)
    step = max(1e-3, float(sample_step_m))

    sample_count = max(1, int(math.ceil((end - start) / step)))
    sample_positions = [
        start + (end - start) * index / sample_count
        for index in range(sample_count + 1)
    ]
    # Always include discrete path points as well.  A sharp bend can sit
    # between two timer ticks and would otherwise be underestimated by the
    # regular preview grid.
    first_point = bisect_right(s_values, start)
    for point_s in s_values[first_point:]:
        if point_s > end:
            break
        sample_positions.append(float(point_s))

    maximum = 0.0
    for sample_s in sample_positions:
        maximum = max(maximum, abs(profile_value_at_s(s_values, curvatures, sample_s)))
    return maximum


def adaptive_lookahead_m(
    speed_mps: float,
    base_lookahead_m: float,
    speed_gain_s: float,
    preview_curvature_abs_m_inv: float,
    curvature_gain_m: float = 6.0,
    tight_min_lookahead_m: float = 2.2,
    max_lookahead_m: float = 12.0,
) -> float:
    """Choose a speed- and curvature-aware Pure Pursuit lookahead distance.

    A long preview is stable on a straight road, but it cuts across an urban
    corner when the same distance is used in a tight bend.  The nominal
    distance is attenuated by ``1 + curvature_gain * |kappa|`` and clamped to
    safe bounds so that the controller keeps enough preview to avoid noisy
    steering while still fitting a sharp turn.
    """

    values = (
        speed_mps,
        base_lookahead_m,
        speed_gain_s,
        preview_curvature_abs_m_inv,
        curvature_gain_m,
        tight_min_lookahead_m,
        max_lookahead_m,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("lookahead 파라미터는 유한해야 한다.")
    if base_lookahead_m < 0.0 or speed_gain_s < 0.0 or curvature_gain_m < 0.0:
        raise ValueError("lookahead 파라미터는 음수가 될 수 없다.")

    lower = max(1e-3, float(tight_min_lookahead_m))
    upper = max(lower, float(max_lookahead_m))
    nominal = max(lower, float(base_lookahead_m) + float(speed_gain_s) * max(0.0, float(speed_mps)))
    attenuation = 1.0 + float(curvature_gain_m) * abs(float(preview_curvature_abs_m_inv))
    return max(lower, min(upper, nominal / attenuation))


def curvature_speed_mps(
    curvature: float, lateral_accel_limit_mps2: float, curvature_epsilon: float = 1e-6,
) -> Optional[float]:
    """Unsigned curvature sets speed through a_y = v^2 * |kappa|.

    None means a straight segment has no curvature speed restriction. The
    caller must still apply the overall speed limit and stopping constraints.
    """
    if (not all(math.isfinite(v) for v in (curvature, lateral_accel_limit_mps2, curvature_epsilon))
            or lateral_accel_limit_mps2 <= 0 or curvature_epsilon <= 0):
        raise ValueError("Curvature must be finite and lateral acceleration/epsilon positive")
    magnitude = abs(curvature)
    return None if magnitude <= curvature_epsilon else math.sqrt(lateral_accel_limit_mps2 / magnitude)


def build_speed_profile(
    s_values: Sequence[float],
    curvatures: Sequence[float],
    max_speed_mps: float,
    lateral_accel_limit_mps2: float,
    max_accel_mps2: float,
    max_decel_mps2: float,
    initial_speed_mps: Optional[float] = None,
    final_speed_mps: float = 0.0,
    curvature_epsilon: float = 1e-6,
) -> List[float]:
    """곡률 제한과 가·감속 제한을 반영한 한 바퀴 속도 프로파일을 만든다.

    먼저 곡률별 속도 상한과 전방 가속 가능 속도를 계산하고, 마지막 종료점의
    목표속도에서 역방향으로 제동 가능 속도를 계산한다. 따라서 마지막 점은
    순환점이 아니라 정지해야 하는 종료점으로 취급된다.
    """

    if len(s_values) != len(curvatures) or len(s_values) < 2:
        raise ValueError("속도 프로파일 입력 길이가 올바르지 않다.")
    max_speed = max(0.0, float(max_speed_mps))
    lateral_limit = max(1e-6, float(lateral_accel_limit_mps2))
    max_accel = max(1e-6, float(max_accel_mps2))
    max_decel = max(1e-6, float(max_decel_mps2))

    profile: List[float] = []
    for curvature in curvatures:
        curve_speed = curvature_speed_mps(float(curvature), lateral_limit, curvature_epsilon)
        profile.append(max_speed if curve_speed is None else min(max_speed, curve_speed))

    profile[-1] = min(profile[-1], max(0.0, float(final_speed_mps)))
    if initial_speed_mps is not None:
        profile[0] = min(profile[0], max(0.0, float(initial_speed_mps)))

    # 앞쪽으로 가속할 수 있는 한계를 적용한다.
    for index in range(1, len(profile)):
        ds = max(0.0, s_values[index] - s_values[index - 1])
        reachable = math.sqrt(max(0.0, profile[index - 1] ** 2 + 2.0 * max_accel * ds))
        profile[index] = min(profile[index], reachable)

    # 종료점 속도에서 역방향으로 제동 가능한 한계를 적용한다.
    for index in range(len(profile) - 2, -1, -1):
        ds = max(0.0, s_values[index + 1] - s_values[index])
        reachable = math.sqrt(max(0.0, profile[index + 1] ** 2 + 2.0 * max_decel * ds))
        profile[index] = min(profile[index], reachable)

    return profile
