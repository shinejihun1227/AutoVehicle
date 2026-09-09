"""Bounded dead reckoning and explicit-green release for stop-line control.

Distances refer to the configured ego origin; front_reference_offset_m converts
them to the front-wheel/front-bumper reference. Pedal/deceleration mapping is a
calibration assumption, not a guarantee of vehicle acceleration.
"""

import math
from collections import deque
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


class AccelRiseLimiter:
    """Shared pedal slew limit applied after all stop overlays, never to brake."""
    def __init__(self, rate_per_sec=0.5, max_gap_sec=0.5):
        if not all(finite(v) and v > 0 for v in (rate_per_sec, max_gap_sec)):
            raise ValueError("Pedal rise rate and maximum gap must be positive and finite")
        self.rate, self.max_gap = rate_per_sec, max_gap_sec
        self.last_now = self.last_ros = None
        self.last_accel = 0.0

    def limit(self, requested, brake, now, ros_now):
        dt = 0.0
        if all(finite(v) for v in (now, ros_now, self.last_now, self.last_ros)):
            wall_dt, ros_dt = now - self.last_now, ros_now - self.last_ros
            if 0 <= wall_dt <= self.max_gap and 0 <= ros_dt <= self.max_gap:
                dt = ros_dt
            else:
                self.last_accel = 0.0
        else:
            self.last_accel = 0.0
        output = (0.0 if brake > 0 or not finite(requested) else
                  min(clamp(requested, 0.0, 1.0), self.last_accel + self.rate * dt))
        self.last_accel = output
        self.last_now, self.last_ros = now, ros_now
        return output


class StopLineControllerCore:
    def __init__(self, max_decel_mps2=1.5, planning_decel_mps2=1.0,
                 reaction_time_sec=0.3, hold_distance_m=0.5,
                 approach_distance_m=20.0, trigger_margin_m=1.0,
                 front_reference_offset_m=0.0, min_confidence=0.5,
                 signal_min_confidence=0.5, stopline_timeout_sec=0.8,
                 signal_timeout_sec=0.8, green_confirmation_sec=0.3,
                 max_dead_reckoning_sec=8.0, max_dead_reckoning_m=12.0,
                 max_update_gap_sec=0.5, stop_tolerance_m=0.03,
                 history_extrapolation_sec=0.05):
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
        self.line_target_id = self.active_target_id = None
        self.signal = None
        self.distance = None
        self.distance_at = None
        self.distance_stamp = None
        self.distance_travelled = 0.0
        self.tracking_fault = False
        self.motion_history = deque()
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

    def observe_line(self, distance_m, confidence, valid, stamp, received, ros_now, target_id=None):
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
        # Only an upstream map planner may provide this identity. Camera
        # StopLineDetection has no identity and must leave it unset.
        self.line_target_id = target_id if isinstance(target_id, str) and target_id else None
        signal_green = self._green_confirmed(received, ros_now)
        if not signal_green:
            # Preserve the stop intent even before odometry is available or
            # when another signal callback arrives before the next timer tick.
            self.stop_requested = True
        return True

    def observe_signal(self, state, confidence, valid, stamp, received, ros_now):
        sample = Sample(stamp, received, "UNKNOWN")
        if not sample.fresh(ros_now, received, self.signal_timeout_sec):
            return False
        known = valid and finite(confidence) and self.signal_min_confidence <= confidence <= 1
        state = str(state).upper() if known else "UNKNOWN"
        if state not in ("GREEN", "RED", "YELLOW", "AMBER", "RED_STOP", "YELLOW_STOP"):
            state = "UNKNOWN"
        if stamp <= self.last_signal_stamp:
            # Identical replays cannot confirm green or refresh its lifetime.
            # Conflicting observations at one source time cannot prove which
            # signal applies; invalidate permission until genuinely new frames.
            if stamp == self.last_signal_stamp and self.signal is not None and self.signal.value != state:
                self.revoke_permission()
            return False
        self.last_signal_stamp = stamp
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
            ahead = self._tracked(received, ros_now) and self.distance >= -self.hold_distance_m
            if state != "UNKNOWN" or pending_line or ahead:
                self.stop_requested = True
        return True

    def revoke_permission(self):
        """Invalidate release evidence without moving/forgetting a stop target.

        Keep source timestamp watermarks: replayed green is not new evidence.
        The caller uses this for lost observation order or a control data gap.
        """
        self.signal = None
        self.green_since = None
        self.green_samples = 0
        self.stop_requested = True

    def _green_confirmed(self, now, ros_now):
        return (self.signal is not None and self.signal.value == "GREEN"
                and self.signal.fresh(ros_now, now, self.signal_timeout_sec)
                and self.green_since is not None and self.green_samples >= 2
                and self.signal.stamp - self.green_since + 1e-9 >= self.green_confirmation_sec)

    def _tracked(self, now, ros_now=None):
        ros_now = self.previous_ros if ros_now is None else ros_now
        return (self.distance is not None and self.distance_at is not None
                and self.distance_stamp is not None and ros_now is not None
                and 0 <= now - self.distance_at <= self.max_dead_reckoning_sec
                and -0.05 <= ros_now - self.distance_stamp <= self.max_dead_reckoning_sec
                and self.distance_travelled <= self.max_dead_reckoning_m)

    def _drop_distance(self):
        # Without a line ID/map association, a new farther detection cannot
        # prove the old red-light target has been cleared. Require green after
        # losing an already acquired stop target, not just another line frame.
        # A fresh same-ID map observation may re-establish it in update().
        if self.distance is not None and self.stop_requested:
            self.tracking_fault = True
        self.distance = self.distance_at = self.distance_stamp = None

    def _record_motion(self, ros_now, speed, speed_ok, gap):
        if gap or not speed_ok:
            self.motion_history.clear()
        if not speed_ok:
            return
        if self.motion_history and self.motion_history[-1][0] == ros_now:
            self.motion_history.pop()
        self.motion_history.append((ros_now, speed))
        horizon = self.stopline_timeout_sec + self.max_update_gap_sec
        while len(self.motion_history) > 2 and self.motion_history[1][0] < ros_now - horizon:
            self.motion_history.popleft()

    def _travel_since(self, stamp, ros_now):
        """Integrate observed speed over the source-frame delay, not v_now*age.

        At most one small timing skew before the history start is approximated
        by its first speed. Larger missing intervals are unknown, even if the
        vehicle happens to be stationary now.
        """
        if stamp >= ros_now:
            return 0.0
        if not self.motion_history:
            return None
        points = list(self.motion_history)
        missing = max(0.0, points[0][0] - stamp)
        if missing > self.history_extrapolation_sec + 1e-9:
            return None
        travel = missing * points[0][1]
        for (start, v0), (end, v1) in zip(points, points[1:]):
            left, right = max(start, stamp), min(end, ros_now)
            if right <= left:
                continue
            v_left = v0 + (v1 - v0) * (left - start) / (end - start)
            v_right = v0 + (v1 - v0) * (right - start) / (end - start)
            travel += 0.5 * (v_left + v_right) * (right - left)
        return travel

    def _stop(self, reason, hold=False):
        self.stop_requested = True
        self.holding = self.holding or hold
        return Decision("HOLD" if self.holding else "SAFE_STOP", reason,
                        0.0, 1.0, 0.0, self.distance)

    def update(self, now, ros_now, speed_mps):
        if not finite(now) or not finite(ros_now):
            self.revoke_permission()
            return self._stop("invalid_clock", hold=True)
        dt = 0.0 if self.previous_now is None else now - self.previous_now
        motion_dt = 0.0 if self.previous_ros is None else ros_now - self.previous_ros
        reset = dt < 0 or motion_dt < 0
        self.previous_now, self.previous_ros = now, ros_now
        if reset:
            self.line = self.signal = None
            self.line_target_id = self.active_target_id = None
            self._drop_distance()
            self.motion_history.clear()
            self.last_line_stamp = self.last_signal_stamp = 0.0
            self.consumed_line_stamp = None
            self.green_since = None
            self.green_samples = 0
            self.previous_speed = None
            return self._stop("clock_reset", hold=True)

        speed_ok = finite(speed_mps) and speed_mps >= 0
        gap = max(dt, motion_dt) > self.max_update_gap_sec
        if gap or not speed_ok:
            self.revoke_permission()
        self._record_motion(ros_now, speed_mps, speed_ok, gap)
        if self.distance is not None and (dt > 0 or motion_dt > 0):
            if not speed_ok or self.previous_speed is None or gap:
                # Motion during a data gap is unknown; do not freeze a distance
                # and later reuse it as if the car had stayed in place.
                self._drop_distance()
            else:
                # Velocity belongs to the ROS/simulation clock. Wall time is
                # only a watchdog and must not scale motion in a slow replay.
                travel = 0.5 * (self.previous_speed + speed_mps) * motion_dt
                self.distance -= travel
                self.distance_travelled += travel
        self.previous_speed = speed_mps if speed_ok else None
        if self.distance is not None and not self._tracked(now, ros_now):
            self._drop_distance()

        mapped_reacquisition = (self.line is not None and self.line_target_id is not None
                                and self.line_target_id == self.active_target_id
                                and abs(ros_now - self.line.stamp) <= self.history_extrapolation_sec)
        if ((not self.tracking_fault or mapped_reacquisition) and self.line is not None
                and self.line.stamp != self.consumed_line_stamp
                and self.line.fresh(ros_now, now, self.stopline_timeout_sec)):
            if speed_ok:
                travel = self._travel_since(self.line.stamp, ros_now)
                if travel is not None:
                    measured = self.line.value - self.front_reference_offset_m - travel
                    # A farther/new stop line cannot move the active target forward.
                    if self.distance is None or not self.stop_requested or measured <= self.distance + 2.0:
                        self.distance = (min(self.distance, measured)
                                         if self.stop_requested and self.distance is not None else measured)
                        self.distance_at = self.line.received
                        self.distance_stamp = self.line.stamp
                        self.distance_travelled = travel
                        self.active_target_id = self.line_target_id
                        self.tracking_fault = False
                    self.consumed_line_stamp = self.line.stamp

        signal_fresh = self.signal is not None and self.signal.fresh(ros_now, now, self.signal_timeout_sec)
        green_confirmed = self._green_confirmed(now, ros_now)
        if not signal_fresh:
            self.green_since = None
            self.green_samples = 0
        if not speed_ok:
            return self._stop("odometry_unavailable")
        if green_confirmed and speed_ok:
            self.stop_requested = self.holding = False
            self.tracking_fault = False

        tracked = self._tracked(now)
        if not self.stop_requested and green_confirmed:
            if not tracked or self.distance < -self.hold_distance_m:
                self._drop_distance()
            return Decision("NOMINAL", "green", distance_m=self.distance)
        if not self.stop_requested and tracked and self.distance >= -self.hold_distance_m:
            # A visible stop line with an unknown signal is not permission to cross.
            self.stop_requested = True
        if not self.stop_requested:
            return Decision("NOMINAL", "no_stop_request")
        if self.holding:
            return self._stop("awaiting_confirmed_green")
        if self.tracking_fault:
            return self._stop("stopline_tracking_lost_awaiting_green")
        if not tracked:
            return self._stop("stopline_unavailable_or_prediction_expired")

        remaining = self.distance - self.hold_distance_m
        if remaining <= self.stop_tolerance_m:
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
