"""YOLO 신호등 클래스명을 주행/정지 조건으로 변환한다."""

import math
from collections import Counter, deque
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


class TrackedSignalVotes:
    """Smooth one tracked lamp without delaying a newly observed stop colour.

    Track IDs belong to the custom detector, not to the road or route. A green
    indication needs three matching samples from the last five frames; red and
    yellow take effect on the current frame. Missing IDs use the raw class so
    a tracker that has not assigned IDs does not hide an otherwise valid lamp.
    """

    def __init__(self, window=5, green_votes=3):
        self.window = int(window)
        self.green_votes = int(green_votes)
        self.history = {}

    def observe(self, track_id, class_name):
        if track_id is None:
            return class_name
        key = int(track_id)
        samples = self.history.setdefault(key, deque(maxlen=self.window))
        samples.append(class_name)
        name = str(class_name).lower()
        if "red" in name or "yellow" in name or "amber" in name:
            return class_name
        if any(word in name for word in ("green", "left", "right", "arrow")):
            return class_name if Counter(samples)[class_name] >= self.green_votes else "Unknown"
        return class_name

    def retain(self, visible_ids):
        visible = set(visible_ids)
        for key in tuple(self.history):
            if key not in visible:
                del self.history[key]


def register_cbam_model_layers(torch_module=None):
    """Register custom layers referenced by the team's YOLO checkpoints.

    The signal checkpoint was serialized with C2f_EMA and EMA under
    ``ultralytics.nn.modules.block``.  Registering only on ``tasks`` is enough
    for YAML parsing but not enough for torch's pickle loader, which resolves
    the original module path stored in the checkpoint.
    """
    if torch_module is None:
        import torch as torch_module
    # The camera loop's dependency-free tests replace torch with a small stub.
    if not hasattr(torch_module, "nn"):
        return
    import ultralytics.nn.tasks as tasks
    import ultralytics.nn.modules as modules
    import ultralytics.nn.modules.block as block
    from ultralytics.nn.modules.block import C2f

    nn = torch_module.nn

    class ChannelAttention(nn.Module):
        def __init__(self, channels, reduction=16):
            super().__init__()
            hidden = max(1, channels // reduction)
            self.fc = nn.Sequential(
                nn.Linear(channels, hidden, bias=False), nn.ReLU(inplace=True),
                nn.Linear(hidden, channels, bias=False),
            )
            self.sigmoid = nn.Sigmoid()

        def forward(self, x):
            batch, channels, _, _ = x.size()
            average = self.fc(x.mean((2, 3)).view(batch, channels)).view(batch, channels, 1, 1)
            maximum = self.fc(x.amax((2, 3)).view(batch, channels)).view(batch, channels, 1, 1)
            return x * self.sigmoid(average + maximum)

    class SpatialAttention(nn.Module):
        def __init__(self, kernel_size=7):
            super().__init__()
            self.conv = nn.Conv2d(2, 1, kernel_size, padding=kernel_size // 2, bias=False)
            self.sigmoid = nn.Sigmoid()

        def forward(self, x):
            average = torch_module.mean(x, dim=1, keepdim=True)
            maximum, _ = torch_module.max(x, dim=1, keepdim=True)
            return x * self.sigmoid(self.conv(torch_module.cat([average, maximum], dim=1)))

    class CBAM(nn.Module):
        def __init__(self, c1, kernel_size=7):
            super().__init__()
            self.ca = ChannelAttention(c1)
            self.sa = SpatialAttention(kernel_size)

        def forward(self, x):
            return self.sa(self.ca(x))

    class EMA(nn.Module):
        """Efficient Multi-scale Attention used during signal-model training."""

        def __init__(self, channels, factor=8):
            super().__init__()
            self.groups = factor
            assert channels // self.groups > 0
            self.softmax = nn.Softmax(-1)
            self.agp = nn.AdaptiveAvgPool2d((1, 1))
            self.pool_h = nn.AdaptiveAvgPool2d((None, 1))
            self.pool_w = nn.AdaptiveAvgPool2d((1, None))
            group_channels = channels // self.groups
            self.gn = nn.GroupNorm(group_channels, group_channels)
            self.conv1x1 = nn.Conv2d(
                group_channels, group_channels, kernel_size=1, stride=1, padding=0
            )
            self.conv3x3 = nn.Conv2d(
                group_channels, group_channels, kernel_size=3, stride=1, padding=1
            )

        def forward(self, x):
            batch, channels, height, width = x.size()
            group_x = x.reshape(batch * self.groups, -1, height, width)
            x_h = self.pool_h(group_x)
            x_w = self.pool_w(group_x).permute(0, 1, 3, 2)
            hw = self.conv1x1(torch_module.cat([x_h, x_w], dim=2))
            x_h, x_w = torch_module.split(hw, [height, width], dim=2)

            x1 = self.gn(
                group_x * x_h.sigmoid() * x_w.permute(0, 1, 3, 2).sigmoid()
            )
            x2 = self.conv3x3(group_x)
            x11 = self.softmax(
                self.agp(x1).reshape(batch * self.groups, -1, 1).permute(0, 2, 1)
            )
            x12 = x2.reshape(batch * self.groups, channels // self.groups, -1)
            x21 = self.softmax(
                self.agp(x2).reshape(batch * self.groups, -1, 1).permute(0, 2, 1)
            )
            x22 = x1.reshape(batch * self.groups, channels // self.groups, -1)
            weights = (
                torch_module.matmul(x11, x12) + torch_module.matmul(x21, x22)
            ).reshape(batch * self.groups, 1, height, width)
            return (group_x * weights.sigmoid()).reshape(batch, channels, height, width)

    class C2f_EMA(C2f):
        """C2f block whose inner bottlenecks use the training EMA module."""

        def __init__(self, c1, c2, n=1, shortcut=False, g=1, e=0.5):
            super().__init__(c1, c2, n, shortcut, g, e)
            self.m = nn.ModuleList(EMA(self.c) for _ in range(n))

    tasks.CBAM = CBAM
    tasks.EMA = EMA
    tasks.C2f_EMA = C2f_EMA
    modules.EMA = EMA
    modules.C2f_EMA = C2f_EMA
    block.EMA = EMA
    block.C2f_EMA = C2f_EMA


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
