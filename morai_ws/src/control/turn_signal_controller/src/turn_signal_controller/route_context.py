"""Offline, conservative binding of an ordered route to static MGeo signals.

Only the standard library is needed. Coordinates must share the map's metre
frame; ``s_values`` are cumulative XY distances (an initial offset is allowed).
Consecutive duplicate route points with equal progress are harmless. Invalid
geometry, progress, identifiers or JSON raise ValueError; unreadable files raise
OSError. Callers must treat either exception as unavailable map context.

Matching requires the *whole* directed link, <= 0.8 m XYZ residual, <= 30 degree
XY heading error, at least three route vertices, and >= 2 m continuous support.
Both link-to-route and route-to-link geometry are checked. Spatial grids bound
the work; no ROS, NumPy, network, signal state or scenario input is used.

An ambiguous binding stays UNKNOWN, even when competing links have the same
semantic. Repeated traversals are emitted as UNKNOWN at each supported occurrence
and never assigned a lap implicitly. A route covering only part of a controlled
link produces no context for that link. An empty result means no verified full
link binding, **not** permission to proceed.
"""

import bisect
from collections import defaultdict
from dataclasses import dataclass
import json
import math
from pathlib import Path


__all__ = ["load_route_contexts"]

_RESIDUAL = 0.8
_MIN_COS = math.cos(math.radians(30.0))
_STEP = 0.5
_MIN_SPAN = 2.0
_EPS = 1e-7
_CELL = 4.0
_SEMANTICS = {"straight": "STRAIGHT", "left": "LEFT", "right": "RIGHT",
              "right_unprotected": "RIGHT"}


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("Coordinates and progress must be finite numbers")
    try:
        value = float(value)
    except OverflowError as exc:
        raise ValueError("Numeric value is out of range") from exc
    if not math.isfinite(value):
        raise ValueError("Coordinates and progress must be finite numbers")
    return value


def _xyz(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("Expected an XYZ point")
    return tuple(_number(v) for v in value)


def _id(value):
    if isinstance(value, bool) or not isinstance(value, (str, int)):
        raise ValueError("Expected a nonempty MGeo identifier")
    value = str(value)
    if not value.strip():
        raise ValueError("Expected a nonempty MGeo identifier")
    return value


def _read_set(directory, name):
    with (Path(directory) / (name + ".json")).open(encoding="utf-8-sig") as stream:
        records = json.load(stream)
    if not isinstance(records, list):
        raise ValueError(name + " must be a JSON array")
    result = {}
    for record in records:
        if not isinstance(record, dict):
            raise ValueError(name + " contains a non-object record")
        identifier = _id(record.get("idx"))
        if identifier in result:
            raise ValueError(name + " has duplicate identifier " + identifier)
        result[identifier] = record
    return result


def _distance(a, b):
    return math.hypot(*(x - y for x, y in zip(a, b)))


def _mix(a, b, t):
    return tuple(x + t * (y - x) for x, y in zip(a, b))


@dataclass(frozen=True)
class _Projection:
    s: float
    index: int
    residual: float
    cosine: float


class _Polyline:
    """Directed segments indexed by their XY bounding boxes."""

    def __init__(self, points, s_values=None):
        self.points = []
        self.s = []
        for index, point in enumerate(points):
            s = s_values[index] if s_values is not None else 0.0
            if self.points:
                previous = self.points[-1]
                length = math.hypot(point[0] - previous[0], point[1] - previous[1])
                if not math.isfinite(length):
                    raise ValueError("Segment length is not finite")
                if length == 0.0:
                    if point != previous or (s_values is not None and s != self.s[-1]):
                        raise ValueError("Zero XY length segment has inconsistent Z/progress")
                    continue
                if s_values is None:
                    s = self.s[-1] + length
                elif s <= self.s[-1] or not math.isclose(
                        s - self.s[-1], length, rel_tol=1e-6, abs_tol=1e-5):
                    raise ValueError("s_values must be ordered cumulative XY distances")
            if not math.isfinite(s):
                raise ValueError("Polyline length is not finite")
            self.points.append(point)
            self.s.append(s)
        if len(self.points) < 2:
            raise ValueError("A polyline needs at least two distinct points")
        self.grid = defaultdict(list)
        self.long_segments = []
        self.headings = []
        for i, (a, b) in enumerate(zip(self.points, self.points[1:])):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            self.headings.append(((b[0] - a[0]) / length, (b[1] - a[1]) / length))
            x0, x1 = sorted((math.floor(a[0] / _CELL), math.floor(b[0] / _CELL)))
            y0, y1 = sorted((math.floor(a[1] / _CELL), math.floor(b[1] / _CELL)))
            # Unusually long/sparse segments must not allocate an enormous grid.
            if (x1 - x0 + 1) * (y1 - y0 + 1) > 4096:
                self.long_segments.append(i)
            else:
                for x in range(x0, x1 + 1):
                    for y in range(y0, y1 + 1):
                        self.grid[x, y].append(i)

    def project(self, point, heading):
        """All local occurrences, never the globally nearest route occurrence."""
        indices = set(self.long_segments)
        for x in range(math.floor((point[0] - _RESIDUAL) / _CELL),
                       math.floor((point[0] + _RESIDUAL) / _CELL) + 1):
            for y in range(math.floor((point[1] - _RESIDUAL) / _CELL),
                           math.floor((point[1] + _RESIDUAL) / _CELL) + 1):
                indices.update(self.grid.get((x, y), ()))
        candidates = []
        for i in sorted(indices):
            direction = self.headings[i]
            cosine = direction[0] * heading[0] + direction[1] * heading[1]
            if cosine < _MIN_COS:
                continue
            a, b = self.points[i:i + 2]
            length = self.s[i + 1] - self.s[i]
            along = (point[0] - a[0]) * direction[0] + (point[1] - a[1]) * direction[1]
            # Do not clamp points beyond link/route endpoints into fake entries.
            if along < -_EPS or along > length + _EPS:
                continue
            t = min(1.0, max(0.0, along / length))
            residual = math.hypot(*(v - w for v, w in zip(point, _mix(a, b, t))))
            if residual <= _RESIDUAL:
                candidates.append(_Projection(self.s[i] + t * length, i, residual, cosine))
        # Adjacent segments describe one local projection at a vertex. Distant
        # indices remain distinct, including identical geometry on another lap.
        groups = []
        for candidate in candidates:
            if not groups or candidate.index > groups[-1][-1].index + 1:
                groups.append([])
            groups[-1].append(candidate)
        return [min(group, key=lambda p: (p.residual, -p.cosine, p.s)) for group in groups]

    def samples(self, start=None, end=None):
        start = self.s[0] if start is None else start
        end = self.s[-1] if end is None else end
        if (end - start) / _STEP > 200000:
            raise ValueError("Polyline exceeds the offline sampling budget")
        first = max(0, bisect.bisect_right(self.s, start) - 1)
        last = min(len(self.headings) - 1, bisect.bisect_left(self.s, end) - 1)
        for i in range(first, last + 1):
            lo, hi = max(start, self.s[i]), min(end, self.s[i + 1])
            count = max(1, math.ceil((hi - lo) / _STEP - _EPS))
            for j in range(count):
                s = lo + (hi - lo) * j / count
                t = (s - self.s[i]) / (self.s[i + 1] - self.s[i])
                yield s, _mix(self.points[i], self.points[i + 1], t), self.headings[i]
        if last >= first:
            t = (end - self.s[last]) / (self.s[last + 1] - self.s[last])
            yield end, _mix(self.points[last], self.points[last + 1], t), self.headings[last]


@dataclass
class _Match:
    link_id: str
    entry: float
    exit: float
    sample_count: int
    route_point_count: int
    max_residual: float
    min_cosine: float
    complete: bool


def _match_link(identifier, geometry, route):
    """Extend contiguous ordered runs, retaining all possible occurrences."""
    active = []
    runs = []
    previous_link_s = None
    for link_s, point, heading in geometry.samples():
        projections = route.project(point, heading)
        following = []
        used = set()
        for first_s, chain in active:
            extensions = []
            for index, projection in enumerate(projections):
                delta = projection.s - chain[-1].s
                if _EPS < delta <= link_s - previous_link_s + 2 * _RESIDUAL:
                    extensions.append((first_s, chain + [projection]))
                    used.add(index)
            if extensions:
                following.extend(extensions)
            else:
                runs.append((first_s, previous_link_s, chain))
        for index, projection in enumerate(projections):
            if index not in used:
                following.append((link_s, [projection]))
        if len(following) > 64:
            raise ValueError("Route has too many ambiguous overlapping traversals")
        active = following
        previous_link_s = link_s
    runs.extend((first_s, previous_link_s, chain) for first_s, chain in active)
    matches = []
    for first_s, last_s, chain in runs:
        entry, exit_s = chain[0].s, chain[-1].s
        count = bisect.bisect_right(route.s, exit_s + _EPS) - bisect.bisect_left(route.s, entry - _EPS)
        if len(chain) < 3 or count < 3 or exit_s - entry < _MIN_SPAN - _EPS:
            continue
        if abs((exit_s - entry) - (last_s - first_s)) > 2 * _RESIDUAL:
            continue
        # Reverse coverage rules out shortcuts, out-and-back excursions, and
        # matching only vertices while the intervening route leaves the link.
        residual = max(p.residual for p in chain)
        cosine = min(p.cosine for p in chain)
        previous_s = None
        for _, point, heading in route.samples(entry, exit_s):
            projections = [p for p in geometry.project(point, heading)
                           if first_s - _EPS <= p.s <= last_s + _EPS]
            if len(projections) != 1 or (previous_s is not None and projections[0].s < previous_s - _EPS):
                break
            previous_s = projections[0].s
            residual = max(residual, projections[0].residual)
            cosine = min(cosine, projections[0].cosine)
        else:
            matches.append(_Match(identifier, entry, exit_s, len(chain), count, residual, cosine,
                                  first_s <= _EPS and last_s >= geometry.s[-1] - _EPS))
    return matches


def _competes(a, b):
    if a.link_id == b.link_id:
        return False
    overlap = min(a.exit, b.exit) - max(a.entry, b.entry)
    if overlap < _MIN_SPAN - _EPS:
        return False
    # A partial alternative must cover the full candidate: a short shared fork
    # prefix is resolved by the rest of the complete directed geometry.
    return b.complete or (b.entry <= a.entry + _EPS and b.exit >= a.exit - _EPS)


def _stopline(match, links, nodes, incoming, matches, bindings):
    current = match
    visited = set()
    predecessors = []
    signal_ids = {signal["id"] for signal in bindings[match.link_id]}
    while current.link_id not in visited:
        visited.add(current.link_id)
        link = links[current.link_id]
        node_id = _id(link["from_node_idx"])
        node = nodes[node_id]
        geometry = link["geometry"]
        # Topological IDs alone cannot turn a displaced node into a stopline.
        if _distance(node["xyz"], geometry.points[0]) > _RESIDUAL:
            return None, None, predecessors, "node_geometry_mismatch"
        if node.get("on_stop_line", False):
            if node.get("traffic_light_id") is not None and _id(node["traffic_light_id"]) not in signal_ids:
                return None, None, predecessors, "stopline_signal_mismatch"
            projections = link["route"].project(node["xyz"], geometry.headings[0])
            # Reject repeated node occurrences, even if one is the closest.
            if len(projections) != 1 or abs(projections[0].s - current.entry) > _RESIDUAL:
                return None, None, predecessors, "stopline_projection_ambiguous"
            stop_s = projections[0].s
            if stop_s > match.entry + _EPS:
                return None, None, predecessors, "stopline_after_entry"
            return stop_s, node_id, predecessors, "verified_map_node"
        candidates = []
        for predecessor_id in incoming.get(node_id, ()):
            for candidate in matches.get(predecessor_id, ()):
                if candidate.complete and abs(candidate.exit - current.entry) <= _EPS:
                    candidates.append(candidate)
        if len(candidates) != 1:
            return None, None, predecessors, "predecessor_missing_or_ambiguous"
        candidate = candidates[0]
        if len([m for m in matches[candidate.link_id] if m.complete]) != 1:
            return None, None, predecessors, "repeated_predecessor"
        if any(_competes(candidate, other) for group in matches.values() for other in group):
            return None, None, predecessors, "predecessor_geometry_ambiguous"
        if candidate.link_id in bindings:
            return None, None, predecessors, "previous_signal_context"
        predecessor = links[candidate.link_id]
        if _distance(predecessor["geometry"].points[-1], node["xyz"]) > _RESIDUAL:
            return None, None, predecessors, "predecessor_node_mismatch"
        predecessors.append(candidate.link_id)
        current = candidate
    return None, None, predecessors, "predecessor_cycle"


def _merge_overlaps(contexts):
    """One guard per overlapping component, including upstream stopline guards."""
    groups = []
    for context in sorted(contexts, key=lambda c: (c["start"], c["end"], c["id"])):
        if not groups or context["start"] >= groups[-1][0] - _EPS:
            groups.append([context["end"], [context]])
        else:
            groups[-1][0] = max(groups[-1][0], context["end"])
            groups[-1][1].append(context)
    result = []
    for end, members in groups:
        if len(members) == 1:
            result.append(members[0])
            continue
        merged = dict(members[0])
        signals = {p["id"]: p for c in members for p in c["signal_points"]}
        link_ids = sorted({identifier for c in members for identifier in c["candidate_link_ids"]})
        member_ids = sorted(c["id"] for c in members)
        reasons = sorted({reason for c in members for reason in c["ambiguity_reasons"]} | {"overlapping_contexts"})
        merged.update(id="mgeo:ambiguous:" + "|".join(member_ids), direction="UNKNOWN",
                      start=min(c["start"] for c in members), end=end,
                      entry_s=min(c["entry_s"] for c in members), exit_s=end,
                      stop_s=None, signal_ids=sorted(signals),
                      signal_points=[signals[i] for i in sorted(signals)],
                      link_id=None, link_ids=link_ids, candidate_link_ids=link_ids,
                      related_signal=None, binding_status="ambiguous",
                      ambiguity_reasons=reasons, stopline_status="ambiguous_binding",
                      stop_node_id=None, predecessor_link_ids=[], member_context_ids=member_ids,
                      match_quality={
                          "sample_count": min(c["match_quality"]["sample_count"] for c in members),
                          "route_point_count": min(c["match_quality"]["route_point_count"] for c in members),
                          "max_residual_m": max(c["match_quality"]["max_residual_m"] for c in members),
                          "min_heading_cosine": min(c["match_quality"]["min_heading_cosine"] for c in members)})
        result.append(merged)
    return result


def load_route_contexts(mgeo_dir, points, s_values):
    """Return sorted dictionaries describing static signal-controlled route spans.

    Required fields: ``id``, ``direction`` (STRAIGHT/LEFT/RIGHT/UNKNOWN),
    ``start`` (verified ``stop_s`` or conservative ``entry_s``), ``end`` (actual
    link exit), ``stop_s`` (float or None), ``signal_ids``, ``signal_points``
    (dictionaries with id/x/y/z), and ``source='mgeo'``. All progress is in metres.

    Also exposes entry_s/exit_s, link_id/link_ids/candidate_link_ids,
    related_signal, binding_status, ambiguity_reasons, stopline_status,
    stop_node_id, predecessor_link_ids and match_quality. Signal lists on an
    ambiguous context are candidate heads, not a selected signal. Such a context
    has link_id=None, direction=UNKNOWN and stop_s=None. Unsupported semantics
    (including uturn and left_unprotected) remain UNKNOWN even on a unique link.
    Overlapping contexts are merged into one UNKNOWN context with the earliest
    original guard and latest actual exit. For that merged context, start may
    precede entry_s even though stop_s is None: the guard is retained, but no
    single stopline or signal head is claimed. member_context_ids names its parts.

    No fallback turn geometry or default stopping distance is applied. Only
    traffic_light_set records explicitly typed 'car' bind links. Lamp 'value',
    'dynamic', 'sub_type' and other runtime/phase fields are deliberately ignored.
    """
    points, s_values = list(points), list(s_values)
    if len(points) != len(s_values):
        raise ValueError("points and s_values must have equal lengths")
    try:
        xyz = [_xyz((p.x, p.y, p.z)) for p in points]
    except AttributeError as exc:
        raise ValueError("Route points must expose x, y, z") from exc
    progress = [_number(s) for s in s_values]
    route = _Polyline(xyz, progress)
    links = _read_set(mgeo_dir, "link_set")
    nodes = _read_set(mgeo_dir, "node_set")
    lights = _read_set(mgeo_dir, "traffic_light_set")
    incoming = defaultdict(list)
    for node in nodes.values():
        node["xyz"] = _xyz(node.get("point"))
        if not isinstance(node.get("on_stop_line", False), bool):
            raise ValueError("on_stop_line must be a boolean")
        if node.get("traffic_light_id") is not None:
            _id(node["traffic_light_id"])
    for identifier, link in links.items():
        start, end = _id(link.get("from_node_idx")), _id(link.get("to_node_idx"))
        if start not in nodes or end not in nodes:
            raise ValueError("Link references an unknown node: " + identifier)
        if not isinstance(link.get("points"), list):
            raise ValueError("Link points must be an XYZ array: " + identifier)
        link["geometry"] = _Polyline([_xyz(p) for p in link["points"]])
        link["route"] = route
        incoming[end].append(identifier)
    bindings = defaultdict(list)
    for identifier, signal in lights.items():
        point = _xyz(signal.get("point"))
        if signal.get("type") != "car":
            continue
        if not isinstance(signal.get("link_id_list"), list):
            raise ValueError("Car signal link_id_list must be an array")
        for link_id in sorted({_id(i) for i in signal["link_id_list"]}):
            if link_id not in links:
                raise ValueError("Car signal references an unknown link: " + link_id)
            bindings[link_id].append(dict(id=identifier, x=point[0], y=point[1], z=point[2]))
    matches = {identifier: _match_link(identifier, link["geometry"], route)
               for identifier, link in links.items()}
    all_matches = [match for group in matches.values() for match in group]
    contexts = []
    for identifier in sorted(bindings):
        complete = [m for m in matches[identifier] if m.complete]
        for occurrence, match in enumerate(complete):
            alternatives = [other for other in all_matches if _competes(match, other)]
            candidate_ids = sorted({identifier} | {m.link_id for m in alternatives})
            reasons = []
            if len(complete) > 1:
                reasons.append("repeated_route_link")
            if alternatives:
                reasons.append("competing_links")
            signals = {signal["id"]: signal for link_id in candidate_ids for signal in bindings.get(link_id, ())}
            related = links[identifier].get("related_signal")
            direction = _SEMANTICS.get(related, "UNKNOWN") if isinstance(related, str) else "UNKNOWN"
            if reasons:
                direction = "UNKNOWN"
                stop_s, stop_node, predecessors, stop_status = None, None, [], "ambiguous_binding"
            else:
                stop_s, stop_node, predecessors, stop_status = _stopline(
                    match, links, nodes, incoming, matches, bindings)
            contexts.append({
                "id": "mgeo:{}:{}".format(identifier, occurrence),
                "direction": direction,
                "start": stop_s if stop_s is not None else match.entry,
                "end": match.exit,
                "entry_s": match.entry,
                "exit_s": match.exit,
                "stop_s": stop_s,
                "signal_ids": sorted(signals),
                "signal_points": [signals[i] for i in sorted(signals)],
                "source": "mgeo",
                "link_id": None if reasons else identifier,
                "link_ids": candidate_ids,
                "candidate_link_ids": candidate_ids,
                "related_signal": related if isinstance(related, str) else None,
                "binding_status": "ambiguous" if reasons else "matched",
                "ambiguity_reasons": reasons,
                "stopline_status": stop_status,
                "stop_node_id": stop_node,
                "predecessor_link_ids": predecessors,
                "match_quality": {"sample_count": match.sample_count,
                                  "route_point_count": match.route_point_count,
                                  "max_residual_m": match.max_residual,
                                  "min_heading_cosine": match.min_cosine},
            })
    return _merge_overlaps(contexts)
