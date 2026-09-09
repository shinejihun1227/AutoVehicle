"""YOLO 신호등 클래스명을 주행/정지 조건으로 변환한다."""

import math
from types import SimpleNamespace


def traffic_bbox_plausible(x, y, width, height, image_height):
    """Only reject invalid boxes; oblique/vertical heads need not be wide.

    Shape or screen centre is not evidence that a light belongs to our lane.
    The final controller associates boxes with projected route-linked heads.
    """
    return (all(isinstance(v, (int, float)) and math.isfinite(v)
                for v in (x, y, width, height, image_height))
            and image_height > 0 and width >= 2 and height >= 2
            and x >= 0 and 0 <= y <= image_height)


def directional_observation(objects, min_confidence=0.5):
    """Preserve one directional class; conflicting signal heads are UNKNOWN.

    Combined classes (e.g. Red_Left) must describe one detected signal head.
    Never merge a red head with an unrelated green/arrow head into permission.
    Bare Left/Right mean the model's illuminated arrow classes; generic Arrow
    has no direction and cannot authorize a maneuver.
    """
    supported = {"RED", "YELLOW", "GREEN", "LEFT", "RIGHT", "GREEN_LEFT",
                 "GREEN_RIGHT", "RED_LEFT", "RED_RIGHT"}
    evidence = {}
    for item in objects:
        try:
            score = float(item.conf)
        except (AttributeError, ValueError, TypeError, OverflowError):
            continue
        if not math.isfinite(score) or not min_confidence <= score <= 1.0:
            continue
        name = str(getattr(item, "class_name", "UNKNOWN")).strip().upper().replace(" ", "_")
        if "YELLOW" in name or "AMBER" in name:
            name = "YELLOW"
        if name not in supported:
            name = "UNKNOWN"
        evidence[name] = max(evidence.get(name, 0.0), score)
    if len(evidence) != 1 or "UNKNOWN" in evidence:
        return "UNKNOWN", 0.0
    return next(iter(evidence.items()))


def straight_observation(objects, min_confidence=0.5):
    """Conservative straight-ahead state for consumers without route intent.

    An illuminated turn arrow does not authorize straight travel. Preserve
    directional classes separately for the route-associated maneuver node.
    Conflicting heads never become green, even if one has higher confidence.
    """
    state, confidence = directional_observation(objects, min_confidence)
    if state in ("GREEN", "GREEN_LEFT", "GREEN_RIGHT"):
        return "GREEN", confidence
    if state in ("RED", "RED_LEFT", "RED_RIGHT", "LEFT", "RIGHT"):
        return "RED", confidence
    if state == "YELLOW":
        return state, confidence
    return "UNKNOWN", 0.0


def _class_observation(class_names):
    return straight_observation([SimpleNamespace(class_name=name, conf=1.0)
                                 for name in class_names])[0]


def traffic_signal_has_green(class_names):
    """Only compatible, explicitly recognized green evidence permits straight travel."""
    return _class_observation(class_names) == "GREEN"


def traffic_signal_requires_stop(class_names):
    """Visible conflicting/unsupported heads request a stop; emptiness is unknown."""
    names = list(class_names)
    return bool(names) and not traffic_signal_has_green(names)


class TrafficSignalStopLatch:
    """Latch stops until distinct, continuous green source frames confirm release.

    Empty frames cannot release a stop. Callers enforce source age; repeated
    source stamps never refresh evidence, and callback backlog cannot satisfy
    the confirmation interval unless receipt time also spans that interval.
    """

    def __init__(self, clear_confirmation_s=0.5, max_gap_s=0.8):
        self.clear_confirmation_s = float(clear_confirmation_s)
        self.max_gap_s = float(max_gap_s)
        if (not math.isfinite(self.clear_confirmation_s) or self.clear_confirmation_s < 0
                or not math.isfinite(self.max_gap_s) or self.max_gap_s <= 0):
            raise ValueError("Confirmation must be finite/nonnegative and gap finite/positive")
        self.stop_required = False
        self.last_stamp = self.last_received = self.last_state = None
        self.clear_since = None
        self.clear_received = None
        self.green_samples = 0

    def revoke(self, reset_clock=False):
        self.stop_required = True
        self.clear_since = self.clear_received = None
        self.green_samples = 0
        if reset_clock:
            self.last_stamp = self.last_received = self.last_state = None
        return self.stop_required

    def update(self, class_names, timestamp_sec, received_sec=None):
        names = list(class_names)
        return self.observe(_class_observation(names), timestamp_sec, received_sec,
                            detected=bool(names))

    def observe(self, state, timestamp_sec, received_sec=None, detected=True):
        stamp = float(timestamp_sec)
        received = stamp if received_sec is None else float(received_sec)
        if not math.isfinite(stamp) or stamp <= 0 or not math.isfinite(received):
            return self.revoke()
        evidence = (state, bool(detected))
        if self.last_stamp is not None and stamp <= self.last_stamp:
            if stamp < self.last_stamp or evidence != self.last_state:
                self.revoke()
            return self.stop_required
        if (self.last_stamp is not None and
                (stamp - self.last_stamp > self.max_gap_s
                 or not 0 <= received - self.last_received <= self.max_gap_s)):
            self.revoke()
        self.last_stamp, self.last_received, self.last_state = stamp, received, evidence
        if state != "GREEN":
            self.clear_since = self.clear_received = None
            self.green_samples = 0
            if state in ("RED", "YELLOW") or detected:
                self.stop_required = True
            return self.stop_required
        if self.clear_since is None:
            self.clear_since, self.clear_received = stamp, received
            self.green_samples = 0
            self.stop_required = True
        self.green_samples += 1
        if (self.green_samples >= 2
                and stamp - self.clear_since + 1e-9 >= self.clear_confirmation_s
                and received - self.clear_received + 1e-9 >= self.clear_confirmation_s):
            self.stop_required = False
        return self.stop_required
