"""Map-frame route projection and conflict-zone geometry (no ROS dependency)."""

from __future__ import annotations

import bisect
import json
import math
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


def _xy(point) -> Tuple[float, float]:
    x, y = float(point[0]), float(point[1])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("route coordinates must be finite")
    return x, y


class RoutePolyline:
    def __init__(self, points: Iterable[Sequence[float]]) -> None:
        self.points: List[Tuple[float, float]] = []
        self.s: List[float] = []
        for raw in points:
            point = _xy(raw)
            if self.points:
                step = math.hypot(point[0] - self.points[-1][0], point[1] - self.points[-1][1])
                if step < 1e-6:
                    continue
                self.s.append(self.s[-1] + step)
            else:
                self.s.append(0.0)
            self.points.append(point)
        if len(self.points) < 2:
            raise ValueError("route needs at least two distinct XY points")
        self.length = self.s[-1]

    @classmethod
    def from_path_file(cls, filename: str) -> "RoutePolyline":
        points = []
        for raw in Path(filename).read_text(encoding="utf-8").splitlines():
            line = raw.split("#", 1)[0].strip()
            if line:
                fields = line.replace(",", " ").split()
                if len(fields) < 2:
                    raise ValueError("invalid route line: " + raw)
                points.append((float(fields[0]), float(fields[1])))
        return cls(points)

    @classmethod
    def from_mgeo_links(cls, filename: str, link_ids: Sequence[str]) -> "RoutePolyline":
        if not link_ids:
            raise ValueError("circulating_link_ids must be configured")
        links = {str(item["idx"]): item for item in json.loads(Path(filename).read_text(encoding="utf-8"))}
        points = []
        previous_link = None
        for link_id in link_ids:
            if str(link_id) not in links:
                raise ValueError("unknown MGeo link: " + str(link_id))
            link = links[str(link_id)]
            if (previous_link is not None and previous_link.get("to_node_idx")
                    and link.get("from_node_idx")
                    and previous_link["to_node_idx"] != link["from_node_idx"]):
                raise ValueError("circulating MGeo links are not connected in travel order")
            segment = [_xy(point) for point in link["points"]]
            if len(segment) < 2:
                raise ValueError("MGeo link has too few points: " + str(link_id))
            if points and math.hypot(points[-1][0] - segment[0][0], points[-1][1] - segment[0][1]) > 2.0:
                raise ValueError("circulating links are not joined in travel order")
            points.extend(segment)
            previous_link = link
        return cls(points)

    def point_at(self, progress: float) -> Tuple[float, float]:
        s = max(0.0, min(float(progress), self.length))
        index = max(0, min(len(self.points) - 2, bisect.bisect_right(self.s, s) - 1))
        fraction = (s - self.s[index]) / (self.s[index + 1] - self.s[index])
        a, b = self.points[index], self.points[index + 1]
        return (a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1]))

    def project(self, x: float, y: float, expected_s=None, window_m=None) -> dict:
        x, y = _xy((x, y))
        if expected_s is None or window_m is None:
            first, last = 0, len(self.points) - 2
        else:
            lower = max(0.0, float(expected_s) - float(window_m))
            upper = min(self.length, float(expected_s) + float(window_m))
            first = max(0, bisect.bisect_right(self.s, lower) - 1)
            last = min(len(self.points) - 2, bisect.bisect_right(self.s, upper))
        best = None
        for index in range(first, last + 1):
            a, b = self.points[index], self.points[index + 1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            length_sq = dx * dx + dy * dy
            fraction = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / length_sq))
            px, py = a[0] + fraction * dx, a[1] + fraction * dy
            distance = math.hypot(x - px, y - py)
            progress = self.s[index] + fraction * (self.s[index + 1] - self.s[index])
            tie = 0.0 if expected_s is None else abs(progress - float(expected_s))
            candidate = (distance, tie, progress, index, dx, dy)
            if best is None or candidate < best:
                best = candidate
        distance, _, progress, index, dx, dy = best
        length = math.hypot(dx, dy)
        return {"s_m": progress, "distance_m": distance, "segment": index,
                "tangent": (dx / length, dy / length)}

    def circle_spans(self, center: Sequence[float], radius: float) -> List[Tuple[float, float]]:
        """Arc-length intervals of this polyline inside a map-frame disc."""
        cx, cy = _xy(center)
        radius = float(radius)
        if not math.isfinite(radius) or radius <= 0.0:
            raise ValueError("conflict radius must be positive and finite")
        spans = []
        for index, (a, b) in enumerate(zip(self.points, self.points[1:])):
            dx, dy = b[0] - a[0], b[1] - a[1]
            px, py = a[0] - cx, a[1] - cy
            aa = dx * dx + dy * dy
            bb = 2.0 * (px * dx + py * dy)
            cc = px * px + py * py - radius * radius
            disc = bb * bb - 4.0 * aa * cc
            if disc < 0.0:
                continue
            root = math.sqrt(max(0.0, disc))
            lo = max(0.0, (-bb - root) / (2.0 * aa))
            hi = min(1.0, (-bb + root) / (2.0 * aa))
            if lo <= hi:
                segment_m = self.s[index + 1] - self.s[index]
                spans.append((self.s[index] + lo * segment_m, self.s[index] + hi * segment_m))
        merged = []
        for lo, hi in spans:
            if merged and lo <= merged[-1][1] + 1e-5:
                merged[-1] = (merged[-1][0], max(hi, merged[-1][1]))
            else:
                merged.append((lo, hi))
        return merged


def travel_time(distance_m: float, initial_speed_mps: float,
                acceleration_mps2: float, speed_limit_mps: float) -> float:
    """Time to cover distance with constant acceleration then a speed cap."""
    distance = max(0.0, float(distance_m))
    speed = max(0.0, float(initial_speed_mps))
    accel = float(acceleration_mps2)
    cap = float(speed_limit_mps)
    if accel <= 0.0 or cap <= 0.0:
        raise ValueError("acceleration and speed limit must be positive")
    if distance == 0.0:
        return 0.0
    speed = min(speed, cap)
    accel_distance = max(0.0, (cap * cap - speed * speed) / (2.0 * accel))
    if distance <= accel_distance:
        return (math.sqrt(speed * speed + 2.0 * accel * distance) - speed) / accel
    return (cap - speed) / accel + (distance - accel_distance) / cap
