"""Small camera lane-centering feedback, independent of ROS.

Camera geometry is compared with the curve that an ideally aligned vehicle
would see on the route. This removes the normal 7/14 m chord deflection in a
bend before applying additional, bounded tracking feedback.
"""

import math
import threading
from bisect import bisect_right
from collections import deque
from statistics import median


_NEAR_M = 7.0
_FAR_M = 14.0
_ACQUIRE_SEC = 0.2


def _finite(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _polyline(points):
    if not isinstance(points, (list, tuple)) or len(points) < 2:
        return None
    if any(not isinstance(p, (list, tuple)) or len(p) < 2
           or not all(_finite(v) for v in p[:2]) for p in points):
        return None
    if any(a[0] >= b[0] for a, b in zip(points, points[1:])):
        return None
    return points


def _at_x(points, x):
    """Return (y, local slope) only inside the observed polyline."""
    for a, b in zip(points, points[1:]):
        if a[0] <= x <= b[0]:
            slope = (b[1] - a[1]) / (b[0] - a[0])
            return a[1] + slope * (x - a[0]), slope
    return None


def lane_measurement(info):
    """Return the LaneDetection convention: -y(7), atan2(y(14)-y(7), 7)."""
    if not isinstance(info, dict):
        return None
    points = _polyline(info.get('centerline_points'))
    if points is None:
        return None
    near, far = _at_x(points, _NEAR_M), _at_x(points, _FAR_M)
    if near is None or far is None:
        return None
    values = -near[0], math.atan2(far[0] - near[0], _FAR_M - _NEAR_M)
    return values if all(_finite(v) for v in values) else None


def _xy(point):
    try:
        return float(point.x), float(point.y)
    except AttributeError:
        return float(point[0]), float(point[1])


def ideal_lane_reference(points, s_values, progress_s):
    """Expected y at body x=7/14 m from the route-center tangent frame.

    Arc lengths are the planner's strictly increasing cumulative XY lengths.
    A short symmetric tangent avoids vertex-heading discontinuities. Only the
    first forward branch is used; short paths, hairpins and ambiguous geometry
    disable the aid instead of extrapolating across an intersection.
    """
    try:
        if (len(points) != len(s_values) or len(points) < 2
                or not _finite(progress_s)
                or any(not _finite(s) for s in s_values)
                or any(b <= a for a, b in zip(s_values, s_values[1:]))
                or not s_values[0] <= progress_s <= s_values[-1]):
            return None

        def at_s(s):
            i = min(len(points) - 2, max(0, bisect_right(s_values, s) - 1))
            ratio = (s - s_values[i]) / (s_values[i + 1] - s_values[i])
            ax, ay = _xy(points[i])
            bx, by = _xy(points[i + 1])
            return ax + ratio * (bx - ax), ay + ratio * (by - ay)

        origin = at_s(progress_s)
        before = at_s(max(s_values[0], progress_s - 0.75))
        after = at_s(min(s_values[-1], progress_s + 0.75))
        tx, ty = after[0] - before[0], after[1] - before[1]
        length = math.hypot(tx, ty)
        if not math.isfinite(length) or length < 1e-6:
            return None
        tx, ty = tx / length, ty / length
        finish = min(s_values[-1], progress_s + 40.0)
        first = bisect_right(s_values, progress_s)
        last = bisect_right(s_values, finish)
        samples = [_xy(point) for point in points[first:last]]
        samples.append(at_s(finish))
        previous = (0.0, 0.0)
        values = []
        for px, py in samples:
            dx, dy = px - origin[0], py - origin[1]
            current = (tx * dx + ty * dy, -ty * dx + tx * dy)
            if not all(math.isfinite(v) for v in current):
                return None
            if math.hypot(current[0] - previous[0], current[1] - previous[1]) < 1e-8:
                continue
            if current[0] <= previous[0] + 1e-8:
                return None
            for target in (_NEAR_M, _FAR_M)[len(values):]:
                if previous[0] <= target <= current[0]:
                    ratio = (target - previous[0]) / (current[0] - previous[0])
                    values.append(previous[1] + ratio * (current[1] - previous[1]))
            if len(values) == 2:
                return tuple(values) if all(math.isfinite(v) for v in values) else None
            previous = current
    except (AttributeError, TypeError, ValueError, IndexError, OverflowError):
        return None
    return None


def lane_quality(info, min_confidence):
    """Require two observed paint boundaries covering the same 7--14 m span.

    Width is measured approximately normal to the local lane tangent. Raw
    left/right y separation grows in a bend and is not the physical width.
    A guide substitution or a coast-only prediction never enables feedback.
    """
    if not _finite(min_confidence) or not 0.0 <= min_confidence <= 1.0:
        raise ValueError('min_confidence must be finite and within [0, 1]')
    minimum = 0.0

    def reject(reason):
        return False, 0.0, reason, minimum

    if not isinstance(info, dict) or info.get('lane_valid') is not True:
        return reject('lane_invalid')
    if (info.get('frame_id') != 'base_link'
            or info.get('coordinate_convention') != {'x': 'forward_m', 'y': 'left_m'}):
        return reject('lane_frame')
    if info.get('center_source') != 'both':
        return reject('two_observed_boundaries_required')
    straddling = info.get('straddling_lane')
    if straddling is not None and (not isinstance(straddling, dict)
                                   or straddling.get('detected') is not False):
        return reject('straddling_lane')
    boundaries = []
    confidences = []
    for side in ('left', 'right'):
        meta = info.get(side + '_lane')
        if (not isinstance(meta, dict) or meta.get('detected') is not True
                or meta.get('from_guide') is not False or meta.get('coasted') is not False):
            return reject(side + '_not_observed_paint')
        confidence = meta.get('confidence')
        if not _finite(confidence) or not 0.0 <= confidence <= 1.0:
            return reject(side + '_invalid_confidence')
        confidences.append(confidence)
        minimum = min(confidences)
        if confidence < min_confidence:
            return reject(side + '_low_confidence')
        count = meta.get('n_points')
        limits = meta.get('x_range_m')
        if (not _finite(count) or count < 2
                or not isinstance(limits, (list, tuple)) or len(limits) != 2
                or not all(_finite(v) for v in limits)
                or not limits[0] <= _NEAR_M < _FAR_M <= limits[1]):
            return reject(side + '_short_observation')
        points = _polyline(info.get(side + '_boundary_points'))
        if points is None or points[0][0] > _NEAR_M or points[-1][0] < _FAR_M:
            return reject(side + '_invalid_geometry')
        boundaries.append(points)
    center = _polyline(info.get('centerline_points'))
    width = info.get('lane_width_m')
    if center is None or not _finite(width) or width <= 0:
        return reject('invalid_center_geometry')
    # Include source vertices as well as half-metre samples to catch short
    # discontinuities and nonphysical boundary crossings between endpoints.
    xs = {_NEAR_M + 0.5 * i for i in range(15)}
    xs.update(p[0] for line in boundaries + [center] for p in line
              if _NEAR_M <= p[0] <= _FAR_M)
    widths = []
    for x in sorted(xs):
        left, right, middle = (_at_x(line, x) for line in boundaries + [center])
        if left is None or right is None or middle is None:
            return reject('incomplete_center_support')
        dy = left[0] - right[0]
        slope = 0.5 * (left[1] + right[1])
        normal_width = dy / math.hypot(1.0, slope)
        if not _finite(normal_width) or not 2.7 <= normal_width <= 4.2:
            return reject('implausible_lane_width')
        if abs(middle[0] - 0.5 * (left[0] + right[0])) > 0.12:
            return reject('center_boundary_mismatch')
        widths.append(normal_width)
    spread = max(widths) - min(widths)
    if spread > 0.7:
        return reject('lane_width_changes_too_fast')
    quality = _clamp(1.0 - spread / 1.0, 0.3, 1.0)
    return True, quality, 'both_boundaries_observed', minimum


class LaneCenteringAssist:
    """Acquire stable residuals, then produce a small continuous steering aid.

    Positive correction steers left in the usual vehicle convention. The
    caller applies its own steering sign convention. ``received``/``wall_now``
    must share one monotonic clock; source stamps and ``ros_now`` share ROS time.
    ``active=False`` can still have a decaying correction: apply correction_rad
    on every control tick so signal loss does not create a steering step.

    Confirmation uses 0.8 s (or timeout_sec if longer) relative to the newest
    frame, so a stable CPU stream need not exceed 7.5 FPS to acquire four frames.
    The newest frame itself must be fresh on both clocks at every control tick.
    Four frames over 0.2 s acquire the aid; once acquired, two consistent recent
    frames maintain it. Invalid/stale/jumping input clears this latch. Only the
    newest three samples contribute to the output median, limiting added lag.
    """

    def __init__(self, weight=0.25, max_correction_rad=0.045, min_frames=4,
                 timeout_sec=0.4, filter_tau_sec=0.35,
                 correction_rate_rad_s=0.06, release_rate_rad_s=0.10):
        values = (weight, max_correction_rad, timeout_sec, filter_tau_sec,
                  correction_rate_rad_s, release_rate_rad_s)
        if (not all(_finite(v) for v in values) or not 0 <= weight <= 1
                or not 0 <= max_correction_rad < math.pi / 4
                or timeout_sec <= _ACQUIRE_SEC or filter_tau_sec <= 0
                or correction_rate_rad_s <= 0 or release_rate_rad_s <= 0
                or isinstance(min_frames, bool) or not isinstance(min_frames, int)
                or min_frames < 2):
            raise ValueError('Invalid lane-centering parameters')
        self.weight = weight
        self.maximum = max_correction_rad
        self.min_frames = min_frames
        self.timeout = timeout_sec
        self.confirmation_window = max(0.8, timeout_sec)
        self.tau = filter_tau_sec
        self.rate = correction_rate_rad_s
        self.release_rate = release_rate_rad_s
        self._samples = deque(maxlen=256)
        self._last_stamp = None
        self._last_received = None
        self._filtered = 0.0
        self._output = 0.0
        self._reason = 'waiting_for_lane'
        self._acquired = False
        self._lock = threading.RLock()

    def _invalidate(self, reason):
        self._samples.clear()
        self._acquired = False
        self._filtered = 0.0
        self._reason = str(reason)

    def invalidate(self, reason):
        """Discard confirmation, retaining only output for controlled release."""
        with self._lock:
            self._invalidate(reason)

    def observe(self, lateral, heading, reference_tuple, confidence, quality,
                stamp, received):
        with self._lock:
            if (not all(_finite(v) for v in
                        (lateral, heading, confidence, quality, stamp, received))
                    or stamp <= 0 or received < 0
                    or not 0 <= confidence <= 1 or not 0 < quality <= 1
                    or not isinstance(reference_tuple, (tuple, list))
                    or len(reference_tuple) != 2
                    or not all(_finite(v) for v in reference_tuple)
                    or abs(heading) >= math.pi / 2 - 0.01):
                self._invalidate('invalid_lane_observation')
                return False
            if ((self._last_stamp is not None and stamp <= self._last_stamp)
                    or (self._last_received is not None and received <= self._last_received)):
                self._invalidate('non_monotonic_lane_clock')
                # A clock reset requires new increasing observations to reacquire.
                self._last_stamp, self._last_received = stamp, received
                return False
            if ((self._last_stamp is not None and stamp - self._last_stamp > self.timeout)
                    or (self._last_received is not None
                        and received - self._last_received > self.timeout)):
                self._invalidate('lane_observation_gap')
            self._last_stamp, self._last_received = stamp, received
            y7 = -lateral
            y14 = y7 + 7.0 * math.tan(heading)
            d7, d14 = y7 - reference_tuple[0], y14 - reference_tuple[1]
            offset = 2.0 * d7 - d14
            error = math.atan2(d14 - d7, 7.0)
            if (not all(_finite(v) for v in (offset, error))
                    or abs(offset) > 1.2 or abs(error) > 0.25):
                self._invalidate('lane_route_disagreement')
                return False
            while self._samples and (stamp - self._samples[0][0] > self.confirmation_window
                                     or received - self._samples[0][1] > self.confirmation_window):
                self._samples.popleft()
            sample = (stamp, received, offset, error, confidence, quality)
            candidate = list(self._samples) + [sample]
            offsets, errors = [p[2] for p in candidate], [p[3] for p in candidate]
            if max(offsets) - min(offsets) > 0.35 or max(errors) - min(errors) > 0.10:
                self._invalidate('lane_residual_unstable')
                self._samples.append(sample)
                return False
            self._samples.append(sample)
            self._reason = 'acquiring_lane'
            return True

    def update(self, speed_mps, ros_now, wall_now, dt, enabled=True, stopped=False):
        with self._lock:
            clock_valid = all(_finite(v) for v in (ros_now, wall_now, dt)) and dt > 0
            source_age = ros_now - self._last_stamp if (
                _finite(ros_now) and self._last_stamp is not None) else None
            received_age = wall_now - self._last_received if (
                _finite(wall_now) and self._last_received is not None) else None
            if not clock_valid:
                self._invalidate('invalid_control_clock')
            elif not enabled:
                self._invalidate('disabled')
            elif not _finite(speed_mps) or speed_mps < 0:
                self._invalidate('invalid_vehicle_speed')
            elif stopped or speed_mps < 0.3:
                self._invalidate('stopped')
                # The caller applies no lane correction while stopped. Do not
                # retain a hidden prior value that could reappear at departure.
                self._output = 0.0
            elif (source_age is None or received_age is None
                  or not -0.05 <= source_age <= self.timeout
                  or not 0 <= received_age <= self.timeout):
                # Preserve the explicit reason a newly received frame was
                # rejected; otherwise missing accepted samples hide every
                # quality/route failure behind a misleading stale label.
                if self._samples or self._reason in ('waiting_for_lane', 'stopped',
                                                      'centering_active', 'acquiring_lane'):
                    self._invalidate('lane_observation_stale')

            samples = list(self._samples)
            stable = len(samples)
            recent = samples[-3:]
            offset = median(p[2] for p in recent) if recent else 0.0
            error = median(p[3] for p in recent) if recent else 0.0
            confidence = min(p[4] for p in samples) if samples else 0.0
            quality = min(p[5] for p in samples) if samples else 0.0
            if (stable >= self.min_frames
                    and samples[-1][0] - samples[0][0] >= _ACQUIRE_SEC
                    and samples[-1][1] - samples[0][1] >= _ACQUIRE_SEC):
                self._acquired = True
            active = (clock_valid and bool(enabled) and not stopped
                      and _finite(speed_mps) and speed_mps >= 0.3
                      and self._acquired and stable >= 2)
            target = 0.0
            if active:
                # A slowly varying residual is useful; a jumping painted-line
                # selection is not. Range penalties reduce gain before rejection.
                offset_range = max(p[2] for p in samples) - min(p[2] for p in samples)
                error_range = max(p[3] for p in samples) - min(p[3] for p in samples)
                jitter_gain = _clamp(1.0 - max(offset_range / 0.35,
                                              error_range / 0.10), 0.2, 1.0)
                quality *= jitter_gain
                offset_control = math.copysign(max(0.0, abs(offset) - 0.05), offset)
                heading_control = math.copysign(max(0.0, abs(error) - 0.01), error)
                target = (math.atan2(0.8 * offset_control, speed_mps + 2.0)
                          + 0.5 * heading_control) * self.weight * confidence * quality
                target = _clamp(target, -self.maximum, self.maximum)
                self._reason = 'centering_active'
            elif samples:
                self._reason = 'acquiring_lane'
            # A long scheduling pause must not cause an abrupt steering change.
            step_dt = min(dt, 0.1) if clock_valid else 0.0
            if active:
                alpha = -math.expm1(-step_dt / self.tau)
                self._filtered += alpha * (target - self._filtered)
            else:
                self._filtered = 0.0
            desired = self._filtered
            if desired * self._output < 0:
                desired = 0.0
            releasing = abs(desired) < abs(self._output)
            limit = (self.release_rate if releasing else self.rate) * step_dt
            self._output += _clamp(desired - self._output, -limit, limit)
            self._output = _clamp(self._output, -self.maximum, self.maximum)
            return dict(enabled=bool(enabled), active=bool(active), reason=self._reason,
                        correction_rad=self._output, quality=quality, confidence=confidence,
                        stable_frames=stable, offset_m=offset, heading_error_rad=error,
                        source_age_sec=source_age)
