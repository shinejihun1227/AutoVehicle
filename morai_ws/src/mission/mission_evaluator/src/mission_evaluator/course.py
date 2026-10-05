"""ROS-free route/checkpoint geometry. No simulator ground-truth is queried."""

import bisect
from collections import defaultdict, namedtuple
import json
import math
from pathlib import Path

Point = namedtuple("Point", "x y z")


def finite_number(value):
    if isinstance(value, bool):
        raise ValueError("Boolean is not a coordinate/time")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("Expected finite number")
    return value


def project_segment(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    norm = dx * dx + dy * dy
    t = 0.0 if norm == 0 else max(0.0, min(1.0, ((point[0]-a[0])*dx + (point[1]-a[1])*dy) / norm))
    return t, math.hypot(point[0]-a[0]-t*dx, point[1]-a[1]-t*dy)


def circle_entry(a, b, centre, radius):
    """Fraction of a continuous segment entering the XY checkpoint disc."""
    px, py = a[0]-centre[0], a[1]-centre[1]
    c = px*px + py*py - radius*radius
    if c <= 1e-10:
        return 0.0
    dx, dy = b[0]-a[0], b[1]-a[1]
    aa, bb = dx*dx + dy*dy, 2*(px*dx + py*dy)
    discriminant = bb*bb - 4*aa*c
    if aa <= 1e-12 or discriminant < -1e-9:
        return None
    t = (-bb-math.sqrt(max(0.0, discriminant))) / (2*aa)
    return max(0.0, min(1.0, t)) if -1e-9 <= t <= 1+1e-9 else None


class Route:
    def __init__(self, points):
        self.points, self.s = [], []
        for raw in points:
            p = Point(*(finite_number(v) for v in raw))
            if self.points and math.hypot(p.x-self.points[-1].x, p.y-self.points[-1].y) < 1e-8:
                continue
            distance = 0.0 if not self.points else math.hypot(p.x-self.points[-1].x, p.y-self.points[-1].y)
            self.s.append((self.s[-1] if self.s else 0.0) + distance)
            self.points.append(p)
        if len(self.points) < 2 or self.s[-1] <= 0:
            raise ValueError("Route requires at least two distinct XY points")
        self.length = self.s[-1]

    @classmethod
    def load(cls, filename):
        points = []
        for line in Path(filename).read_text(encoding="utf-8").splitlines():
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            values = line.split()
            if len(values) not in (2, 3):
                raise ValueError("Expected x y [z] route line")
            points.append([float(values[0]), float(values[1]), float(values[2]) if len(values) == 3 else 0.0])
        return cls(points)

    def project(self, point, lower=0.0, upper=None):
        upper = self.length if upper is None else min(self.length, upper)
        lower = max(0.0, lower)
        if upper < lower:
            raise ValueError("Invalid progress search interval")
        start = min(len(self.s)-2, max(0, bisect.bisect_right(self.s, lower)-1))
        end = min(len(self.s)-1, bisect.bisect_right(self.s, upper))
        best = None
        for i in range(start, end):
            fraction, _ = project_segment(point, self.points[i], self.points[i+1])
            ds = self.s[i+1]-self.s[i]
            progress = max(lower, min(upper, self.s[i]+fraction*ds))
            fraction = (progress-self.s[i])/ds
            a, b = self.points[i], self.points[i+1]
            distance = math.hypot(point[0]-a.x-fraction*(b.x-a.x), point[1]-a.y-fraction*(b.y-a.y))
            candidate = (distance, progress, i)
            if best is None or candidate < best:
                best = candidate
        return {"distance_m": best[0], "s_m": best[1], "segment": best[2]}

    def point_at(self, progress):
        s = max(0.0, min(self.length, progress))
        i = min(len(self.points)-2, max(0, bisect.bisect_right(self.s, s)-1))
        t = (s-self.s[i])/(self.s[i+1]-self.s[i])
        a, b = self.points[i], self.points[i+1]
        return Point(*(a[j]+t*(b[j]-a[j]) for j in range(3)))


class Course:
    def __init__(self, route, rules, mgeo_dir):
        self.route, self.rules = route, rules
        self.radius = finite_number(rules["checkpoint_radius_m"])
        if self.radius <= 0:
            raise ValueError("Checkpoint radius must be positive")
        self.start = rules["start_finish"]["xyz"]
        if math.dist(self.start[:2], route.points[0][:2]) > 0.8 or math.dist(self.start[:2], route.points[-1][:2]) > 0.8:
            raise ValueError("Configured route must begin/end at the official START_END")
        self.checkpoints = []
        previous_s, seen = 0.0, set()
        for raw in rules["checkpoints"]:
            cp = dict(raw)
            if cp["id"] in seen:
                raise ValueError("Duplicate checkpoint ID")
            seen.add(cp["id"])
            match = route.project(cp["xyz"])
            if match["distance_m"] > self.radius or match["s_m"] <= previous_s:
                raise ValueError("Checkpoint not on route/in order: " + cp["id"])
            cp.update(s_m=match["s_m"], route_residual_m=match["distance_m"])
            self.checkpoints.append(cp)
            previous_s = cp["s_m"]
        if not self.checkpoints:
            raise ValueError("Ordered checkpoints are required")
        links = {str(item["idx"]): item for item in json.loads((Path(mgeo_dir)/"link_set.json").read_text(encoding="utf-8"))}
        highway = rules["highway"]
        start = route.project(links[highway["start_link"]]["points"][0])
        end = route.project(links[highway["end_link"]]["points"][-1])
        if max(start["distance_m"], end["distance_m"]) > 0.8 or start["s_m"] >= end["s_m"]:
            raise ValueError("Official highway endpoints do not match this route")
        # Validate the complete two boundary links, not merely nearby endpoints.
        for key in ("start_link", "end_link"):
            matched = [route.project(p) for p in links[highway[key]]["points"]]
            if any(p["distance_m"] > 0.8 for p in matched) or any(b["s_m"] < a["s_m"]-1e-6 for a,b in zip(matched,matched[1:])):
                raise ValueError("Highway link geometry/direction mismatch")
        self.highway = (start["s_m"], end["s_m"])

    def highway_active(self, progress):
        return self.highway[0] <= progress < self.highway[1]

    def blackout(self, progress):
        entries = self.rules.get("blackout_regions", [])
        if not entries:
            return None
        return any(item["start_s_m"] <= progress <= item["end_s_m"] for item in entries)

    def report(self):
        return {"route_length_m": self.route.length, "start_target_m": self.route.length*self.rules["start_progress_ratio"],
                "highway_start_s_m": self.highway[0], "highway_end_s_m": self.highway[1],
                "checkpoint_count": len(self.checkpoints), "checkpoints": self.checkpoints,
                "blackout_geometry_configured": bool(self.rules.get("blackout_regions"))}


class LaneMap:
    """Approximate wheel/solid-line contact, not Unity wheel-collider truth.

    Stop lines/crosswalks are explicitly excluded. Mixed solid/broken records
    are skipped because the permitted side cannot be inferred from their name.
    """
    def __init__(self, filename, config):
        self.config, self.segments, self.grid = config, [], defaultdict(set)
        self.skipped_mixed = 0
        for line in json.loads(Path(filename).read_text(encoding="utf-8")):
            shapes = [str(s).lower() for s in line.get("lane_shape", [])]
            types = line.get("lane_type", [])
            if len(shapes) != 1 or len(types) != 1 or " " in shapes[0]:
                self.skipped_mixed += 1
                continue
            if types[0] not in config["line_types"] or (shapes[0] != "solid" and types[0] != 501):
                continue
            width = finite_number(line.get("lane_width", 0.15))
            if not 0 < width <= 1.0:
                continue
            for a,b in zip(line["points"], line["points"][1:]):
                index = len(self.segments)
                self.segments.append((a,b,width/2,str(line["idx"])))
                for x in range(math.floor((min(a[0],b[0])-1)/5),math.floor((max(a[0],b[0])+1)/5)+1):
                    for y in range(math.floor((min(a[1],b[1])-1)/5),math.floor((max(a[1],b[1])+1)/5)+1):
                        self.grid[(x,y)].add(index)
        if not self.segments:
            raise ValueError("No supported solid/centre-line segments")

    def contact(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        touches = set()
        for longitudinal in (0.0, self.config["wheelbase_m"]):
            for lateral in (-self.config["wheel_track_m"]/2, self.config["wheel_track_m"]/2):
                p = (x+c*longitudinal-s*lateral, y+s*longitudinal+c*lateral)
                for index in self.grid.get((math.floor(p[0]/5),math.floor(p[1]/5)), ()):
                    a,b,half_width,identifier = self.segments[index]
                    if project_segment(p,a,b)[1] <= half_width+self.config["tire_half_width_m"]:
                        touches.add(identifier)
        return sorted(touches)
