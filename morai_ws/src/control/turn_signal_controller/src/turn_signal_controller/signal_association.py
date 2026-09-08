"""Pure image-time signal projection, route association, and confirmation.

The caller owns route-head selection, calibration enablement, pose lookup at
the image timestamp, and source freshness against the current clock. No ROS
types, camera defaults, wall clock, or screen-center heuristics are used here.
"""

from collections.abc import Mapping
from dataclasses import dataclass, replace
import math
from typing import Hashable, Optional


__all__ = ["Selection", "project_signal", "associate", "SignalConfirmation"]

_STATES = frozenset(("RED", "YELLOW", "GREEN", "LEFT", "RIGHT", "GREEN_LEFT",
                     "GREEN_RIGHT", "RED_LEFT", "RED_RIGHT"))
_MIN_CONFIDENCE = 0.5
_GATE_PX = 25.0


@dataclass(frozen=True)
class Selection:
    """Associated state; consumers must require ``valid`` before permitting.

    A pending confirmation retains the observed state/ID with ``valid=False``.
    Unavailable, malformed, or ambiguous evidence is UNKNOWN with confidence
    zero and no selected ID. RED_LEFT/RED_RIGHT preserve directional meaning;
    the caller decides which confirmed states permit its route maneuver.
    """

    state: str = "UNKNOWN"
    confidence: float = 0.0
    valid: bool = False
    selected_id: Optional[Hashable] = None
    reason: str = "signal_unknown"


def _unknown(reason):
    return Selection(reason=reason)


def _number(value):
    if isinstance(value, bool):
        raise ValueError("boolean is not a measurement")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("measurement must be finite")
    return result


def _valid_id(value):
    if value is None or isinstance(value, bool):
        return False
    if isinstance(value, str):
        return bool(value.strip())
    try:
        hash(value)
        return value == value  # Reject NaN identifiers too.
    except TypeError:
        return False


def project_signal(point, pose, camera):
    """Return an image-pixel ``(u, v)`` or None when not safely projectable.

    ``point`` is an MGeo map-frame dict with x/y/z in meters. ``pose`` is the
    ego origin in that same frame at the IMAGE timestamp: x/y/z and yaw in
    radians, plus optional roll/pitch in radians (both default to zero).
    Body axes are x forward, y left, z up. Pose uses right-handed ROS RPY:
    R_world_body = Rz(yaw) Ry(pitch) Rx(roll); positive pose pitch noses DOWN.

    Every camera key is required: width/height (positive integral pixels),
    horizontal_fov_deg (strictly between 0 and 180), x/y/z (optical-center
    offset in body meters), pitch_deg and yaw_deg (mount angles in degrees).
    Camera yaw is positive LEFT; camera pitch is explicitly positive UP,
    so R_body_camera = Rz(yaw_deg) Ry(-pitch_deg). Camera roll is zero.

    The pinhole has square pixels, focal length width/(2*tan(hfov/2)), and
    principal point (width/2, height/2). u grows right and v grows down.
    Detections must use this same calibrated, unletterboxed pixel image.
    Missing/nonfinite inputs, points behind the optical plane, and points
    outside [0,width) x [0,height) return None. Calibration flags and lens
    distortion/rectification are caller responsibilities.
    """
    if not all(isinstance(item, Mapping) for item in (point, pose, camera)):
        return None
    try:
        px, py, pz = (_number(point[key]) for key in ("x", "y", "z"))
        ex, ey, ez, yaw = (_number(pose[key]) for key in ("x", "y", "z", "yaw"))
        roll = _number(pose.get("roll", 0.0))
        pitch = _number(pose.get("pitch", 0.0))
        width, height, fov, cx, cy, cz, cp, cyaw = (
            _number(camera[key]) for key in ("width", "height", "horizontal_fov_deg",
                                            "x", "y", "z", "pitch_deg", "yaw_deg"))
        if (width <= 0 or height <= 0 or not width.is_integer()
                or not height.is_integer() or not 0 < fov < 180):
            return None

        # Inverse ego RPY, followed by the body-frame camera translation.
        dx, dy, dz = px - ex, py - ey, pz - ez
        sy, co_y = math.sin(yaw), math.cos(yaw)
        bx, by = co_y * dx + sy * dy, -sy * dx + co_y * dy
        sp, co_p = math.sin(pitch), math.cos(pitch)
        bx, bz = co_p * bx - sp * dz, sp * bx + co_p * dz
        sr, co_r = math.sin(roll), math.cos(roll)
        by, bz = co_r * by + sr * bz, -sr * by + co_r * bz
        bx, by, bz = bx - cx, by - cy, bz - cz

        # Inverse mount yaw and positive-UP pitch. Optical depth is forward.
        cp, cyaw = math.radians(cp), math.radians(cyaw)
        forward = math.cos(cyaw) * bx + math.sin(cyaw) * by
        left = -math.sin(cyaw) * bx + math.cos(cyaw) * by
        depth = math.cos(cp) * forward + math.sin(cp) * bz
        up = -math.sin(cp) * forward + math.cos(cp) * bz
        if not all(math.isfinite(v) for v in (depth, left, up)) or depth <= 1e-9:
            return None
        focal = width / (2.0 * math.tan(math.radians(fov) / 2.0))
        u, v = width / 2.0 - focal * left / depth, height / 2.0 - focal * up / depth
        if math.isfinite(u) and math.isfinite(v) and 0 <= u < width and 0 <= v < height:
            return u, v
    except (KeyError, TypeError, ValueError, OverflowError, ZeroDivisionError):
        pass
    return None


def _projections(values):
    if not isinstance(values, Mapping):
        raise ValueError("projections must be an ID-to-pixel mapping")
    result = {}
    for head_id, pixel in values.items():
        if not _valid_id(head_id):
            raise ValueError("invalid head ID")
        if pixel is None:  # Direct use of project_signal results is supported.
            continue
        u, v = pixel
        u, v = _number(u), _number(v)
        if u < 0 or v < 0:
            raise ValueError("pixel coordinates must be nonnegative")
        result[head_id] = (u, v)
    return result


def _state(name):
    if not isinstance(name, str):
        return "UNKNOWN"
    name = name.strip().upper().replace(" ", "_").replace("-", "_")
    if {"YELLOW", "AMBER"}.intersection(name.split("_")):
        return "YELLOW"
    return name if name in _STATES else "UNKNOWN"


def _contains(box, pixel):
    x0, y0, x1, y1 = box
    u, v = pixel
    return x0 <= u <= x1 and y0 <= v <= y1


def associate(objects, projected_targets, projected_others=None):
    """Select exactly one geometrically unique route-head/detection pair.

    Objects expose ObjectInfo attributes x_center/y_center/width/height in
    pixels, conf in [0,1], and class_name. Boxes expand by a fixed, capped
    25 pixels per side; there is no distance-to-screen-center or confidence
    ranking. Scores below 0.5 are ignored, but malformed measurements fail
    closed (including NaN, confidence >1, and nonpositive box dimensions).

    ``projected_targets`` and optional ``projected_others`` map stable head
    IDs to (u,v). None values are unprojectable heads and are skipped. Other
    heads constrain ambiguity only; they can never be selected. A repeated
    ID in both maps is the same head only if its pixel coordinates agree.
    Multiple plausible pairs, even of the same color, or an other head in
    the selected box's gate return UNKNOWN. Unsupported matched classes also
    return UNKNOWN. An empty target set never falls back to another head.
    """
    try:
        targets = _projections(projected_targets)
        others = _projections({} if projected_others is None else projected_others)
        for head_id in targets.keys() & others.keys():
            if targets[head_id] != others[head_id]:
                return _unknown("invalid_projection")
        others = {head_id: pixel for head_id, pixel in others.items() if head_id not in targets}
    except (TypeError, ValueError, OverflowError):
        return _unknown("invalid_projection")
    if not targets:
        return _unknown("no_projected_targets")

    matches = []
    try:
        for obj in objects:
            x, y, width, height, confidence = (
                _number(getattr(obj, key)) for key in
                ("x_center", "y_center", "width", "height", "conf"))
            if x < 0 or y < 0 or width <= 0 or height <= 0 or not 0 <= confidence <= 1:
                return _unknown("invalid_object")
            box = (x - width / 2.0 - _GATE_PX, y - height / 2.0 - _GATE_PX,
                   x + width / 2.0 + _GATE_PX, y + height / 2.0 + _GATE_PX)
            if not all(math.isfinite(edge) for edge in box):
                return _unknown("invalid_object")
            name = obj.class_name
            if confidence < _MIN_CONFIDENCE:
                continue
            for head_id, pixel in targets.items():
                if _contains(box, pixel):
                    matches.append((head_id, box, _state(name), confidence))
    except (AttributeError, TypeError, ValueError, OverflowError):
        return _unknown("invalid_object")

    if not matches:
        return _unknown("no_match")
    if len(matches) != 1:
        return _unknown("ambiguous_match")
    head_id, box, state, confidence = matches[0]
    if any(_contains(box, pixel) for pixel in others.values()):
        return _unknown("ambiguous_other_head")
    if state == "UNKNOWN":
        return _unknown("unsupported_class")
    return Selection(state, confidence, True, head_id, "matched")


class SignalConfirmation:
    """Confirm permissions from distinct, ordered source-image timestamps.

    update(selection, stamp) returns Selection. Pass the raw association and
    the IMAGE timestamp as finite, nonnegative seconds, never a control-tick
    time. All permissive states (including LEFT/RIGHT and RED_LEFT/RED_RIGHT)
    need the same ID AND state for min_frames distinct frames spanning
    min_duration_sec. Pending output has valid=False/reason=signal_unconfirmed.
    RED/YELLOW are valid immediately and discard permission history. UNKNOWN
    or invalid evidence resets confirmation; no previous red state is held.

    Identical calls at one timestamp return the cached result without adding
    evidence. Changed permission evidence at a duplicate timestamp or reordered
    timestamps revoke confirmation. Reordering retains the timestamp watermark;
    reset() is required for a new clock epoch. Gaps greater than max_gap_sec
    start a fresh sequence. With only source timestamps this class cannot know
    wall-clock age or detect silence: the caller must reject stale/future images
    and reset on camera timeout, route change, or calibration invalidation.
    """

    def __init__(self, min_frames=3, min_duration_sec=0.3, max_gap_sec=0.5):
        if isinstance(min_frames, bool) or not isinstance(min_frames, int) or min_frames < 3:
            raise ValueError("min_frames must be an integer >= 3")
        try:
            min_duration_sec = _number(min_duration_sec)
            max_gap_sec = _number(max_gap_sec)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("confirmation times must be finite numbers") from exc
        if min_duration_sec < 0.3 or max_gap_sec <= 0:
            raise ValueError("min_duration_sec must be >= 0.3 and max_gap_sec must be > 0")
        self.min_frames = min_frames
        self.min_duration_sec = min_duration_sec
        self.max_gap_sec = max_gap_sec
        self.reset()

    def _clear_candidate(self):
        self._candidate = None
        self._started_stamp = None
        self._frames = 0

    def reset(self):
        """Discard all evidence and the source timestamp watermark."""
        self._clear_candidate()
        self._last_stamp = None
        self._last_selection = None
        self._output = _unknown("signal_reset")

    def _revoke(self, reason):
        self._clear_candidate()
        self._last_selection = None
        self._output = _unknown(reason)
        return self._output

    def update(self, selection, stamp):
        """Consume one source observation and return its confirmation result."""
        try:
            stamp = _number(stamp)
            if stamp < 0:
                raise ValueError("negative timestamp")
        except (TypeError, ValueError, OverflowError):
            return self._revoke("invalid_stamp")
        if self._last_stamp is not None and stamp < self._last_stamp:
            return self._revoke("signal_out_of_order")
        duplicate = stamp == self._last_stamp
        if self._last_stamp is not None and stamp > self._last_stamp + self.max_gap_sec:
            self._clear_candidate()
        previous = self._last_selection
        self._last_stamp = stamp
        self._last_selection = selection

        try:
            if (not isinstance(selection, Selection) or type(selection.valid) is not bool
                    or not isinstance(selection.state, str)):
                raise ValueError("invalid selection structure")
            confidence = _number(selection.confidence)
            if (not 0 <= confidence <= 1 or selection.state not in _STATES | {"UNKNOWN"}
                    or (selection.valid and (confidence < _MIN_CONFIDENCE
                                            or not _valid_id(selection.selected_id)
                                            or selection.state == "UNKNOWN"))):
                raise ValueError("invalid selection evidence")
        except (TypeError, ValueError, OverflowError):
            return self._revoke("invalid_selection")
        if not selection.valid:
            return self._revoke(selection.reason or "signal_unknown")
        if selection.state in ("RED", "YELLOW"):
            self._clear_candidate()
            self._output = replace(selection, confidence=confidence, reason="signal_immediate")
            return self._output
        if duplicate:
            if selection == previous:
                return self._output
            return self._revoke("signal_duplicate_stamp")

        candidate = (selection.selected_id, selection.state)
        if candidate != self._candidate:
            self._clear_candidate()
            self._candidate = candidate
            self._started_stamp = stamp
        self._frames += 1
        confirmed = (self._frames >= self.min_frames
                     and stamp >= self._started_stamp + self.min_duration_sec)
        self._output = replace(selection, confidence=confidence, valid=confirmed,
                               reason="signal_confirmed" if confirmed else "signal_unconfirmed")
        return self._output
