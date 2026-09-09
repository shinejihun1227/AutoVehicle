"""Route-based turn speed and exit alignment checks; no steering/ROS writer.

The loaded route remains the steering authority. Limits are initial tuning
values; normalized brake does not guarantee a measured vehicle deceleration.
"""

import math
from bisect import bisect_left, bisect_right
from dataclasses import dataclass

from curvature_speed_purepursuit.planner import interpolate_by_s, three_point_curvature
from stopline_control.core import Decision, clamp, finite


def angle_error(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def quaternion_yaw(orientation):
    values = [getattr(orientation, key, None) for key in ("x", "y", "z", "w")]
    if not all(finite(v) for v in values):
        return None
    norm = math.sqrt(sum(v * v for v in values))
    if not math.isfinite(norm) or abs(norm - 1.) > .1:
        return None
    x, y, z, w = (v / norm for v in values)
    return math.atan2(2. * (w * z + x * y), 1. - 2. * (y * y + z * z))


@dataclass(frozen=True)
class TurnMotion:
    phase: str = "NONE"
    fault: str = ""
    speed_limit_kph: object = None
    curve_speed_kph: object = None
    heading_error_deg: object = None
    lateral_error_m: object = None
    exit_heading_error_deg: object = None
    exit_ready: bool = False
    accel_limit: float = 1.0
    brake: float = 0.0

    def constrain(self, decision):
        """Keep stop/permission state and strengthen longitudinal limits only."""
        restrictive = self.accel_limit < decision.accel_limit or self.brake > decision.brake
        target = decision.target_speed_kph
        if self.speed_limit_kph is not None:
            target = self.speed_limit_kph if target is None else min(target, self.speed_limit_kph)
        return Decision(decision.mode, "turn_speed_envelope" if restrictive else decision.reason,
                        min(decision.accel_limit, self.accel_limit),
                        max(decision.brake, self.brake), target, decision.distance_m)


class TurnMotionPlanner:
    def __init__(self, points, s_values, left_speed_kph=15., right_speed_kph=10.,
                 lateral_accel_mps2=1.2, max_heading_error_deg=60.,
                 max_lateral_error_m=1.5, exit_heading_error_deg=15.,
                 exit_lateral_error_m=.75, exit_overrun_m=3.,
                 planning_decel_mps2=1., max_decel_mps2=1.5, reaction_time_sec=.3):
        config = locals().copy()
        for key in ("self", "points", "s_values"):
            config.pop(key)
        if (not all(finite(v) and v > 0 for v in config.values())
                or not exit_heading_error_deg <= max_heading_error_deg < 90.
                or exit_lateral_error_m > max_lateral_error_m
                or planning_decel_mps2 > max_decel_mps2):
            raise ValueError("Invalid turn speed, alignment or deceleration parameters")
        self.__dict__.update(config)
        self.points, self.s_values = points, s_values
        # Resample in metres: varying original point density must not change
        # the curvature window. Keep this static work outside the control tick.
        self.sample_s = [s_values[0] + i for i in range(int(s_values[-1] - s_values[0]) + 1)]
        if self.sample_s[-1] < s_values[-1]:
            self.sample_s.append(s_values[-1])
        sampled = [interpolate_by_s(points, s_values, s)[0] for s in self.sample_s]
        self.curvature = [abs(three_point_curvature(sampled, i)) for i in range(len(sampled))]

    def heading(self, progress):
        # A short symmetric tangent also works at the first/last path point.
        start = max(self.s_values[0], min(progress - 1., self.s_values[-1] - 1.))
        end = min(self.s_values[-1], max(progress + 1., self.s_values[0] + 1.))
        a, _ = interpolate_by_s(self.points, self.s_values, start)
        b, _ = interpolate_by_s(self.points, self.s_values, end)
        return math.atan2(b.y - a.y, b.x - a.x)

    def evaluate(self, event, progress, speed, x, y, yaw, crossing_s):
        if not event or event["kind"] != "turn" or event["direction"] not in ("LEFT", "RIGHT"):
            return TurnMotion()
        phase = ("EXIT_ALIGNMENT" if finite(progress) and progress >= event["end"]
                 else "TURNING" if event.get("committed") else "APPROACH")
        if not all(finite(v) for v in (progress, speed, x, y, yaw, crossing_s)) or speed < 0:
            return TurnMotion(phase, "turn_pose_unavailable", accel_limit=0., brake=1.)
        point, _ = interpolate_by_s(self.points, self.s_values, progress)
        lateral = math.hypot(x - point.x, y - point.y)
        heading_error = math.degrees(angle_error(yaw, self.heading(progress)))
        exit_error = math.degrees(angle_error(yaw, self.heading(event["end"])))
        exit_ready = (progress >= event["end"] and abs(exit_error) <= self.exit_heading_error_deg
                      and lateral <= self.exit_lateral_error_m)
        fault = ("turn_route_heading_mismatch" if abs(heading_error) > self.max_heading_error_deg
                 else "turn_route_lateral_error" if lateral > self.max_lateral_error_m
                 else "turn_exit_alignment_unconfirmed" if not exit_ready
                 and progress > event["end"] + self.exit_overrun_m else "")
        # Use the tightest remaining bend; no smoothing across a sharp turn.
        start = max(event["start"], min(progress, event["end"]))
        lo = max(0, bisect_left(self.sample_s, start) - 1)
        hi = min(len(self.sample_s), bisect_right(self.sample_s, event["end"]) + 1)
        curvature = max(self.curvature[lo:hi], default=0.)
        limit = (self.left_speed_kph if event["direction"] == "LEFT" else self.right_speed_kph) / 3.6
        if curvature > 1e-6:
            limit = min(limit, math.sqrt(self.lateral_accel_mps2 / curvature))
        remaining = max(0., crossing_s - progress)
        effective = max(0., remaining - speed * self.reaction_time_sec)
        target = math.sqrt(limit * limit + 2. * self.planning_decel_mps2 * effective)
        accel_limit = clamp((target - speed) * 3.6 * .2, 0., 1.)
        brake = 0.
        if speed > limit:
            required = (speed * speed - limit * limit) / (2. * max(effective, .05))
            if remaining > .05 and required >= self.planning_decel_mps2:
                brake = clamp(required / self.max_decel_mps2, 0., 1.)
            elif remaining <= .05:
                brake = clamp((speed - limit) / self.max_decel_mps2, 0., 1.)
        if brake > 0 or fault:
            accel_limit = 0.
        return TurnMotion(phase, fault, target * 3.6, limit * 3.6, heading_error,
                          lateral, exit_error, exit_ready, accel_limit, 1. if fault else brake)
