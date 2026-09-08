"""Bounded dead reckoning and explicit-green release for stop-line control.

Distances refer to the configured ego origin; front_reference_offset_m converts
them to the front-wheel/front-bumper reference. Pedal/deceleration mapping is a
calibration assumption, not a guarantee of vehicle acceleration.
"""

import math
from dataclasses import dataclass


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


@dataclass(frozen=True)
class Sample:
    stamp: float
    received: float
    value: object

    def fresh(self, ros_now, now, timeout):
        return (
            all(finite(v) for v in (self.stamp, self.received, ros_now, now))
            and self.stamp > 0.0
            and -0.05 <= ros_now - self.stamp <= timeout
            and 0.0 <= now - self.received <= timeout
        )


@dataclass(frozen=True)
class Decision:
    mode: str
    reason: str
    accel_limit: float = 1.0
    brake: float = 0.0
    target_speed_kph: object = None
    distance_m: object = None


class StopLineControllerCore:
    def __init__(self, max_decel_mps2=1.5, planning_decel_mps2=1.0,
                 reaction_time_sec=0.3, hold_distance_m=0.5,
                 approach_distance_m=20.0, trigger_margin_m=1.0,
                 front_reference_offset_m=0.0, min_confidence=0.5,
                 signal_min_confidence=0.5, stopline_timeout_sec=0.8,
                 signal_timeout_sec=0.8, green_confirmation_sec=0.3,
                 max_dead_reckoning_sec=8.0, max_dead_reckoning_m=12.0,
                 max_update_gap_sec=0.5):
        values = locals().copy()
        values.pop("self")
        if not all(finite(v) and v >= 0 for v in values.values()):
            raise ValueError("Stopline parameters must be finite and non-negative")
        if min(max_decel_mps2, planning_decel_mps2, stopline_timeout_sec,
               signal_timeout_sec, max_update_gap_sec) <= 0:
            raise ValueError("Deceleration and timeouts must be positive")
        if planning_decel_mps2 > max_decel_mps2:
            raise ValueError("planning_decel_mps2 must not exceed max_decel_mps2")
        if min_confidence > 1 or signal_min_confidence > 1:
            raise ValueError("Confidence thresholds must be in [0, 1]")
        self.__dict__.update(values)
        self.line = None
        self.signal = None
        self.distance = None
        self.distance_at = None
        self.distance_travelled = 0.0
        self.consumed_line_stamp = None
        self.last_line_stamp = 0.0
        self.last_signal_stamp = 0.0
        self.stop_requested = False
        self.holding = False
        self.green_since = None
        self.green_samples = 0
        self.previous_now = None
        self.previous_ros = None
        self.previous_speed = None

    def observe_line(self, distance_m, confidence, valid, stamp, received, ros_now):
        sample = Sample(stamp, received, distance_m)
        if not sample.fresh(ros_now, received, self.stopline_timeout_sec):
            return False
        if stamp <= self.last_line_stamp:
            return False
        self.last_line_stamp = stamp
        if not valid or not finite(distance_m) or distance_m < 0:
            return False
        if not finite(confidence) or not self.min_confidence <= confidence <= 1:
            return False
        # An invalid frame never erases the previously tracked stop line.
        self.line = sample
        signal_green = (self.signal is not None and self.signal.value == "GREEN"
                        and self.signal.fresh(ros_now, received, self.signal_timeout_sec))
        if not signal_green:
            # Preserve the stop intent even before odometry is available or
            # when another signal callback arrives before the next timer tick.
            self.stop_requested = True
        return True

    def observe_signal(self, state, confidence, valid, stamp, received, ros_now):
        sample = Sample(stamp, received, "UNKNOWN")
        if (not sample.fresh(ros_now, received, self.signal_timeout_sec)
                or stamp <= self.last_signal_stamp):
            return False
        self.last_signal_stamp = stamp
        known = valid and finite(confidence) and self.signal_min_confidence <= confidence <= 1
        state = str(state).upper() if known else "UNKNOWN"
        if state not in ("GREEN", "RED", "YELLOW", "AMBER", "RED_STOP", "YELLOW_STOP"):
            state = "UNKNOWN"
        previous = self.signal
        self.signal = Sample(stamp, received, state)
        if state == "GREEN":
            if (self.green_since is None or previous is None or previous.value != "GREEN"
                    or stamp - previous.stamp > self.signal_timeout_sec
                    or received - previous.received > self.signal_timeout_sec):
                self.green_since = stamp
                self.green_samples = 0
            self.green_samples += 1
        else:
            self.green_since = None
            self.green_samples = 0
            pending_line = (self.line is not None
                            and self.line.stamp != self.consumed_line_stamp
                            and self.line.fresh(ros_now, received, self.stopline_timeout_sec))
            ahead = self._tracked(received) and self.distance >= -self.hold_distance_m
            if state != "UNKNOWN" or pending_line or ahead:
                self.stop_requested = True
        return True

    def _tracked(self, now):
        return (self.distance is not None and self.distance_at is not None
                and 0 <= now - self.distance_at <= self.max_dead_reckoning_sec
                and self.distance_travelled <= self.max_dead_reckoning_m)

    def _stop(self, reason, hold=False):
        self.stop_requested = True
        self.holding = self.holding or hold
        return Decision("HOLD" if self.holding else "SAFE_STOP", reason,
                        0.0, 1.0, 0.0, self.distance)

    def update(self, now, ros_now, speed_mps):
        if not finite(now) or not finite(ros_now):
            return self._stop("invalid_clock", hold=True)
        dt = 0.0 if self.previous_now is None else now - self.previous_now
        motion_dt = 0.0 if self.previous_ros is None else ros_now - self.previous_ros
        reset = dt < 0 or motion_dt < 0
        self.previous_now, self.previous_ros = now, ros_now
        if reset:
            self.line = self.signal = None
            self.distance = self.distance_at = None
            self.last_line_stamp = self.last_signal_stamp = 0.0
            self.consumed_line_stamp = None
            self.green_since = None
            self.green_samples = 0
            self.previous_speed = None
            return self._stop("clock_reset", hold=True)

        speed_ok = finite(speed_mps) and speed_mps >= 0
        if self.distance is not None and (dt > 0 or motion_dt > 0):
            if (not speed_ok or self.previous_speed is None
                    or max(dt, motion_dt) > self.max_update_gap_sec):
                # Motion during a data gap is unknown; do not freeze a distance
                # and later reuse it as if the car had stayed in place.
                self.distance = self.distance_at = None
            else:
                # Velocity belongs to the ROS/simulation clock. Wall time is
                # only a watchdog and must not scale motion in a slow replay.
                travel = 0.5 * (self.previous_speed + speed_mps) * motion_dt
                self.distance -= travel
                self.distance_travelled += travel
        self.previous_speed = speed_mps if speed_ok else None

        if (self.line is not None and self.line.stamp != self.consumed_line_stamp
                and self.line.fresh(ros_now, now, self.stopline_timeout_sec)):
            if speed_ok:
                age = max(0.0, ros_now - self.line.stamp)
                measured = self.line.value - self.front_reference_offset_m - speed_mps * age
                # Expired history is not an active target at the next junction.
                if not self._tracked(now):
                    self.distance = self.distance_at = None
                # A farther/new stop line cannot move the active target forward.
                if self.distance is None or not self.stop_requested or measured <= self.distance + 2.0:
                    self.distance = (min(self.distance, measured)
                                     if self.stop_requested and self.distance is not None else measured)
                    self.distance_at = now - age
                    self.distance_travelled = speed_mps * age
                self.consumed_line_stamp = self.line.stamp

        signal_fresh = self.signal is not None and self.signal.fresh(ros_now, now, self.signal_timeout_sec)
        green = signal_fresh and self.signal.value == "GREEN"
        green_confirmed = (green and self.green_since is not None and self.green_samples >= 2
                           and self.signal.stamp - self.green_since >= self.green_confirmation_sec)
        if not signal_fresh:
            self.green_since = None
            self.green_samples = 0
        if green_confirmed:
            self.stop_requested = self.holding = False

        tracked = self._tracked(now)
        if not self.stop_requested and green:
            if not tracked or self.distance < -self.hold_distance_m:
                self.distance = self.distance_at = None
            return Decision("NOMINAL", "green", distance_m=self.distance)
        if not self.stop_requested and tracked and self.distance >= -self.hold_distance_m:
            # A visible stop line with an unknown signal is not permission to cross.
            self.stop_requested = True
        if not self.stop_requested:
            return Decision("NOMINAL", "no_stop_request")
        if self.holding:
            return self._stop("awaiting_confirmed_green")
        if not speed_ok:
            return self._stop("odometry_unavailable")
        if not tracked:
            return self._stop("stopline_unavailable_or_prediction_expired")

        remaining = self.distance - self.hold_distance_m
        if remaining <= 0 or (speed_mps <= 0.08 and remaining <= 0.03):
            return self._stop("at_stop_target", hold=True)
        a = self.planning_decel_mps2
        tau = self.reaction_time_sec
        target = max(0.0, math.sqrt((a * tau)**2 + 2.0 * a * remaining) - a * tau)
        braking_distance = speed_mps * tau + speed_mps**2 / (2.0 * a)
        horizon = max(self.approach_distance_m,
                      braking_distance + self.hold_distance_m + self.trigger_margin_m)
        if self.distance > horizon:
            return Decision("NOMINAL", "stopline_far", distance_m=self.distance)

        brake = 0.0
        accel_limit = clamp((target - speed_mps) * 3.6 * 0.2, 0.0, 1.0)
        if remaining <= braking_distance + self.trigger_margin_m and speed_mps > 0.15:
            effective = max(remaining - speed_mps * tau, 0.05)
            required_decel = speed_mps**2 / (2.0 * effective)
            brake = clamp(required_decel / self.max_decel_mps2, 0.0, 1.0)
            accel_limit = 0.0
        return Decision("APPROACH", "stopline_speed_envelope", accel_limit, brake,
                        target * 3.6, self.distance)
