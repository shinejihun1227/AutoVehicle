"""Route intent + camera permission + continuously transmitted indicators.

This module does not generate a new driving path. Turns follow the loaded
route; manual lane-change events describe a maneuver already in that route.
"""

import math
import struct
from dataclasses import dataclass

from curvature_speed_purepursuit.planner import interpolate_by_s


def build_lamp_packet(direction):
    code = {"OFF": 0, "LEFT": 1, "RIGHT": 2}[direction]
    return struct.pack("<13s i 3i b b 2s", b"#LampControl$", 2, 0, 0, 0, code, 0, b"\r\n")


def signal_permits(state, direction, right_on_green=True):
    permissions = {
        "GREEN": {"STRAIGHT"} | ({"RIGHT"} if right_on_green else set()),
        "GREEN_LEFT": {"STRAIGHT", "LEFT"} | ({"RIGHT"} if right_on_green else set()),
        "GREEN_RIGHT": {"STRAIGHT", "RIGHT"},
        "LEFT": {"LEFT"}, "RED_LEFT": {"LEFT"},
        "RIGHT": {"RIGHT"}, "RED_RIGHT": {"RIGHT"},
    }
    return direction in permissions.get(str(state).upper(), set())


@dataclass(frozen=True)
class RouteIntent:
    direction: str
    end_s_m: float
    heading_change_rad: float


def route_intent(points, s_values, line_s, preview_m=30.0, threshold_deg=25.0):
    """Compare entry/exit route tangents around a CAMERA-observed stop line.

    A bend without a stop-line observation does not create a turn event.
    Distance horizon is configurable; explicit route events override it.
    """
    end = line_s + preview_m
    if line_s < 5.0 or end + 3.0 > s_values[-1]:
        return RouteIntent("UNKNOWN", min(end, s_values[-1]), 0.0)
    def heading(start, finish):
        a, _ = interpolate_by_s(points, s_values, start)
        b, _ = interpolate_by_s(points, s_values, finish)
        return math.atan2(b.y - a.y, b.x - a.x)
    delta = heading(end - 3.0, end + 3.0) - heading(line_s - 5.0, line_s)
    delta = math.atan2(math.sin(delta), math.cos(delta))
    degrees = math.degrees(delta)
    direction = ("UNKNOWN" if abs(degrees) > 150.0 else "LEFT" if degrees >= threshold_deg
                 else "RIGHT" if degrees <= -threshold_deg else "STRAIGHT")
    return RouteIntent(direction, end, delta)


class IndicatorLead:
    """Count lead time only across successful, fresh, same-direction UDP sends.

    UDP success is local transmission, NOT simulator lamp acknowledgement.
    Require both simulation and monotonic elapsed time; pause/replay cannot
    silently complete the timer while simulation time stands still.
    """
    def __init__(self, lead_sec=5.0, max_gap_sec=0.6):
        if not math.isfinite(lead_sec) or lead_sec < 5.0:
            raise ValueError("indicator lead time must be at least five seconds")
        self.lead_sec, self.max_gap_sec = lead_sec, max_gap_sec
        self.reset()

    def reset(self):
        self.direction = "OFF"
        self.started = self.last = None

    def sent(self, direction, now, ros_now, success=True):
        if not success or direction == "OFF":
            self.reset()
            return
        if (direction != self.direction or self.last is None
                or not 0 <= now - self.last[0] <= self.max_gap_sec
                or ros_now < self.last[1]):
            self.started = (now, ros_now)
        self.direction, self.last = direction, (now, ros_now)

    def ready(self, direction, now, ros_now):
        return bool(self.started is not None and direction == self.direction
                    and 0 <= now - self.last[0] <= self.max_gap_sec
                    and now - self.started[0] >= self.lead_sec
                    and ros_now - self.started[1] >= self.lead_sec)
