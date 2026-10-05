"""Pure helpers for turning a camera stop-line range into a map landmark."""

from __future__ import annotations

import math


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return yaw from a finite, nonzero ROS quaternion."""
    values = (float(x), float(y), float(z), float(w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        raise ValueError("quaternion must be nonzero")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def project_stopline(route, x: float, y: float, yaw: float, distance_m: float,
                     route_progress_m: float, window_m: float = 30.0) -> dict:
    """Project a base_link-forward stop-line range into map XY and route s.

    ``distance_m`` is measured from the same base_link origin used by the
    stop-line controller. ``route_progress_m`` is the global route projection
    at the matched odometry timestamp, so repeated route geometry can be
    disambiguated with a local search window.
    """
    x, y, yaw, distance_m, route_progress_m, window_m = map(float, (
        x, y, yaw, distance_m, route_progress_m, window_m))
    if not all(math.isfinite(value) for value in
               (x, y, yaw, distance_m, route_progress_m, window_m)):
        raise ValueError("stop-line projection inputs must be finite")
    if distance_m < 0.0 or route_progress_m < 0.0 or window_m <= 0.0:
        raise ValueError("distance/progress must be nonnegative and window positive")

    line_x = x + distance_m * math.cos(yaw)
    line_y = y + distance_m * math.sin(yaw)
    expected_s = route_progress_m + distance_m
    match = route.project(line_x, line_y, expected_s=expected_s,
                          window_m=max(window_m, distance_m + 5.0))
    return {
        "map_xy": [line_x, line_y],
        "route_s_m": float(match["s_m"]),
        "route_offset_m": float(match["distance_m"]),
        "expected_route_s_m": expected_s,
    }
