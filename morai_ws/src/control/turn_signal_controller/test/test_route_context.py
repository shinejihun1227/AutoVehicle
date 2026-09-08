"""Standard-library tests; run directly with Python -B, without ROS or pytest.

Pass --smoke to additionally print repository route/map coverage and quality.
The smoke oracle independently compares source map vertices with route vertices;
it does not assume that all route distance is controlled by traffic signals.
"""

from collections import Counter, defaultdict, namedtuple
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[2]
sys.path.insert(0, str(PACKAGE / "src"))
from turn_signal_controller.route_context import load_route_contexts


PathPoint = namedtuple("PathPoint", "x y z", defaults=(0.0,))


def progress(points, offset=0.0):
    values = [offset]
    for a, b in zip(points, points[1:]):
        values.append(values[-1] + math.hypot(b.x - a.x, b.y - a.y))
    return values


def straight(start=0, end=100, y=0.0, z=0.0):
    return [PathPoint(float(x), y, z) for x in range(start, end + 1)]


class RouteContextTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.links, self.nodes, self.lights = [], {}, []
        self.route = straight()

    def link(self, identifier="controlled", start=10, end=40, semantic="straight",
             y=0.0, z=0.0, from_node=None, to_node=None, points=None, stop=False):
        points = points or [(start, y, z), (end, y, z)]
        from_node, to_node = from_node or identifier + "_in", to_node or identifier + "_out"
        for node_id, point in ((from_node, points[0]), (to_node, points[-1])):
            self.nodes.setdefault(node_id, dict(idx=node_id, point=list(point), on_stop_line=False))
        if stop:
            self.nodes[from_node]["on_stop_line"] = True
        record = dict(idx=identifier, from_node_idx=from_node, to_node_idx=to_node,
                      points=[list(p) for p in points], related_signal=semantic)
        self.links.append(record)
        return record

    def signal(self, identifier="head", link_ids=None, kind="car", point=(15, 3, 5), **extra):
        record = dict(idx=identifier, link_id_list=link_ids or ["controlled"],
                      type=kind, point=list(point))
        record.update(extra)
        self.lights.append(record)
        return record

    def load(self, points=None, s_values=None):
        for name, records in (("link_set", self.links), ("node_set", list(self.nodes.values())),
                              ("traffic_light_set", self.lights)):
            (Path(self.directory.name) / (name + ".json")).write_text(json.dumps(records), encoding="utf-8")
        points = self.route if points is None else points
        return load_route_contexts(self.directory.name, points,
                                   progress(points) if s_values is None else s_values)

    def test_interface_and_all_static_heads_without_head_selection(self):
        self.link(stop=True)
        self.signal("far", point=(1000, 2000, 5))
        self.signal("near", point=(10, 0, 5))
        self.signal("pedestrian", kind="pedestrian")
        self.signal("bus", kind="bus")
        context, = self.load()
        self.assertEqual(context["direction"], "STRAIGHT")
        self.assertEqual((context["start"], context["entry_s"], context["end"], context["stop_s"]),
                         (10.0, 10.0, 40.0, 10.0))
        self.assertEqual(context["signal_ids"], ["far", "near"])
        self.assertEqual(context["signal_points"], [dict(id="far", x=1000.0, y=2000.0, z=5.0),
                                                    dict(id="near", x=10.0, y=0.0, z=5.0)])
        self.assertEqual(context["source"], "mgeo")
        self.assertEqual(context["binding_status"], "matched")
        self.assertTrue(context["id"])
        self.assertLessEqual(context["match_quality"]["max_residual_m"], 0.8)

    def test_fractional_entry_exit_and_progress_offset_are_not_fixed_preview(self):
        self.link(start=10.25, end=68.75)
        self.signal()
        context, = self.load(s_values=progress(self.route, 100))
        self.assertAlmostEqual(context["entry_s"], 110.25)
        self.assertAlmostEqual(context["end"], 168.75)
        self.assertIsNone(context["stop_s"])
        self.assertEqual(context["start"], context["entry_s"])

    def test_semantics_are_explicit_and_never_inferred_from_geometry(self):
        link = self.link(stop=True)
        self.signal()
        for semantic, expected in (("straight", "STRAIGHT"), ("left", "LEFT"), ("right", "RIGHT"),
                                   ("right_unprotected", "RIGHT"), ("left_unprotected", "UNKNOWN"),
                                   ("uturn", "UNKNOWN"), ("uturn_normal", "UNKNOWN"),
                                   ("straight_left", "UNKNOWN"), (None, "UNKNOWN"), (["left"], "UNKNOWN")):
            with self.subTest(semantic=semantic):
                link["related_signal"] = semantic
                context, = self.load()
                self.assertEqual(context["direction"], expected)

    def test_curved_route_keeps_map_straight_semantic(self):
        self.route = straight(0, 10) + [PathPoint(10, float(y), 0) for y in range(1, 21)]
        self.link(points=[(5, 0, 0), (10, 0, 0), (10, 15, 0)])
        self.signal()
        context, = self.load()
        self.assertEqual(context["direction"], "STRAIGHT")
        self.assertAlmostEqual(context["end"], 25.0)

    def test_strict_residual_boundary_and_height(self):
        link = self.link(y=0.8)
        self.signal()
        self.assertEqual(len(self.load()), 1)
        link["points"] = [[10, 0.800001, 0], [40, 0.800001, 0]]
        self.assertEqual(self.load(), [])
        link["points"] = [[10, 0, 3], [40, 0, 3]]
        self.assertEqual(self.load(), [])

    def test_wrong_way_and_crossing_do_not_bind(self):
        self.link(start=40, end=10)
        self.signal()
        self.assertEqual(self.load(), [])
        self.links[0]["points"] = [[20, -10, 0], [20, 10, 0]]
        self.assertEqual(self.load(), [])

    def test_heading_disagreement_rejects_close_zigzags(self):
        self.link(points=[(10, 0, 0), (10.4, 0.6, 0), (10.8, 0, 0),
                          (11.2, 0.6, 0), (11.6, 0, 0), (12, 0.6, 0), (12.4, 0, 0)])
        self.signal()
        self.assertEqual(self.load(), [])

    def test_close_parallel_and_unsignalled_competitor_preserve_unknown(self):
        self.link(stop=True)
        self.link("parallel", y=0.6, semantic="left")
        self.signal()
        context, = self.load()
        self.assertEqual(context["direction"], "UNKNOWN")
        self.assertEqual(context["candidate_link_ids"], ["controlled", "parallel"])
        self.assertIsNone(context["link_id"])
        self.assertIsNone(context["stop_s"])
        self.assertEqual(context["start"], context["entry_s"])
        self.assertIn("competing_links", context["ambiguity_reasons"])

    def test_parallel_outside_threshold_and_opposite_lane_are_not_alternatives(self):
        self.link(stop=True)
        self.link("parallel", y=0.81)
        self.link("opposite", start=40, end=10, y=0.1)
        self.signal()
        context, = self.load()
        self.assertEqual(context["direction"], "STRAIGHT")
        self.assertEqual(context["candidate_link_ids"], ["controlled"])

    def test_equal_semantic_duplicates_are_still_ambiguous(self):
        self.link(stop=True)
        self.link("duplicate")
        self.signal()
        self.signal("other_head", ["duplicate"])
        context, = self.load()
        self.assertEqual(context["direction"], "UNKNOWN")
        self.assertEqual(context["signal_ids"], ["head", "other_head"])
        self.assertEqual((context["start"], context["end"]), (10.0, 40.0))
        self.assertEqual(len(context["member_context_ids"]), 2)

    def test_overlapping_different_directions_merge_transitively(self):
        self.link(start=10, end=30, stop=True)
        self.link("left", start=20, end=40, semantic="left", stop=True)
        self.link("right", start=35, end=60, semantic="right", stop=True)
        self.signal()
        self.signal("left_head", ["left"])
        self.signal("right_head", ["right"])
        context, = self.load()
        self.assertEqual(context["direction"], "UNKNOWN")
        self.assertEqual(context["binding_status"], "ambiguous")
        self.assertIsNone(context["stop_s"])
        self.assertEqual((context["start"], context["end"]), (10.0, 60.0))
        self.assertEqual(context["signal_ids"], ["head", "left_head", "right_head"])
        self.assertEqual(len(context["member_context_ids"]), 3)

    def test_adjacent_contexts_with_shared_boundary_are_separate(self):
        self.link(start=10, end=30, stop=True)
        self.link("next", start=30, end=60, semantic="left", stop=True)
        self.signal()
        self.signal("next_head", ["next"])
        first, second = self.load()
        self.assertEqual([first["direction"], second["direction"]], ["STRAIGHT", "LEFT"])
        self.assertEqual(first["end"], second["start"])

    def test_overlapping_stopline_guards_keep_earliest_guard(self):
        self.link(start=20, end=40, from_node="entry")
        self.link("approach", start=5, end=20, to_node="entry", stop=True)
        # The short head context is inside the first context's verified guard
        # interval, although the controlled link geometries do not overlap.
        self.link("short", start=19, end=21, semantic="left", stop=True)
        self.signal()
        self.signal("short_head", ["short"])
        context, = self.load()
        self.assertEqual(context["direction"], "UNKNOWN")
        self.assertIsNone(context["stop_s"])
        self.assertEqual((context["start"], context["entry_s"], context["end"]), (5.0, 19.0, 40.0))

    def test_merged_output_is_independent_of_map_record_order(self):
        self.link(start=10, end=30)
        self.link("left", start=20, end=40, semantic="left")
        self.signal()
        self.signal("left_head", ["left"])
        reference = self.load()
        self.links.reverse()
        self.lights.reverse()
        self.assertEqual(self.load(), reference)

    def test_partial_long_parallel_alternative_cannot_be_silently_ignored(self):
        self.route = straight(0, 50)
        self.link(stop=True)
        self.link("long_parallel", start=-10, end=60, y=0.5)
        self.signal()
        context, = self.load()
        self.assertEqual(context["direction"], "UNKNOWN")

    def test_fork_shared_prefix_is_resolved_by_complete_geometry(self):
        self.link()
        self.link("fork", points=[(10, 0, 0), (15, 0, 0), (40, 10, 0)])
        self.signal()
        context, = self.load()
        self.assertEqual(context["direction"], "STRAIGHT")

    def test_partial_route_does_not_invent_link_entry_or_exit(self):
        self.link()
        self.signal()
        self.assertEqual(self.load(points=straight(0, 30)), [])
        self.assertEqual(self.load(points=straight(20, 50)), [])
        self.assertEqual(self.load(points=straight(11, 39)), [])

    def test_sample_count_and_minimum_support(self):
        self.link()
        self.signal()
        self.assertEqual(self.load(points=[PathPoint(0, 0), PathPoint(100, 0)]), [])
        self.links[0]["points"] = [[10, 0, 0], [11, 0, 0]]
        self.assertEqual(self.load(), [])

    def test_route_excursion_breaks_contiguous_match(self):
        self.link()
        self.signal()
        route = straight(0, 20) + [PathPoint(20, 10), PathPoint(21, 10)] + straight(21, 100)
        self.assertEqual(self.load(points=route), [])

    def test_reverse_coverage_catches_excursion_between_map_samples(self):
        self.link()
        self.signal()
        route = straight(0, 15) + [PathPoint(15.125, 0.81), PathPoint(15.25, 0)] + straight(16, 100)
        self.assertEqual(self.load(points=route), [])

    def test_two_laps_have_distinct_unknown_contexts(self):
        self.link(stop=True)
        self.signal()
        route = straight(0, 50) + [PathPoint(50, 10), PathPoint(0, 10)] + straight(0, 50)
        contexts = self.load(points=route)
        self.assertEqual(len(contexts), 2)
        self.assertNotEqual(contexts[0]["id"], contexts[1]["id"])
        self.assertLess(contexts[0]["end"], contexts[1]["start"])
        for context in contexts:
            self.assertEqual(context["direction"], "UNKNOWN")
            self.assertIsNone(context["stop_s"])
            self.assertIn("repeated_route_link", context["ambiguity_reasons"])

    def test_duplicate_route_and_map_vertices_are_safe(self):
        link = self.link(stop=True)
        link["points"].insert(1, link["points"][0][:])
        self.signal()
        route = self.route[:11] + [self.route[10]] + self.route[11:]
        self.assertEqual(self.load(points=route), self.load())

    def predecessor_chain(self):
        self.link(start=20, end=60, from_node="entry")
        self.link("connector", start=12, end=20, from_node="connector_in", to_node="entry")
        self.link("approach", start=5, end=12, to_node="connector_in", stop=True)
        self.signal()

    def test_stopline_walks_unique_predecessors_without_fixed_distance(self):
        self.predecessor_chain()
        context, = self.load()
        self.assertEqual((context["start"], context["stop_s"], context["entry_s"], context["end"]),
                         (5.0, 5.0, 20.0, 60.0))
        self.assertEqual(context["predecessor_link_ids"], ["connector", "approach"])
        self.assertEqual(context["stop_node_id"], "approach_in")

    def test_wrong_way_predecessor_does_not_create_topology_ambiguity(self):
        self.predecessor_chain()
        self.link("wrong_way", start=30, end=20, to_node="entry", stop=True)
        context, = self.load()
        self.assertEqual(context["stop_s"], 5.0)

    def test_ambiguous_predecessors_leave_guard_at_entry(self):
        self.predecessor_chain()
        self.link("another_connector", start=12, end=20, to_node="entry", stop=True)
        context, = self.load()
        self.assertEqual(context["direction"], "STRAIGHT")
        self.assertIsNone(context["stop_s"])
        self.assertEqual(context["start"], 20.0)

    def test_stopline_not_guessed_from_unconnected_nearby_node(self):
        self.link()
        self.signal()
        self.nodes["nearby"] = dict(idx="nearby", point=[9, 0, 0], on_stop_line=True)
        context, = self.load()
        self.assertIsNone(context["stop_s"])
        self.assertEqual(context["start"], 10.0)

    def test_topological_connection_with_route_gap_is_not_stopline_evidence(self):
        self.link(start=20, end=60, from_node="entry")
        self.link("bad_connector", start=5, end=19, to_node="entry", stop=True)
        self.signal()
        context, = self.load()
        self.assertIsNone(context["stop_s"])

    def test_displaced_or_other_signal_stop_node_is_unverified(self):
        self.link(stop=True)
        self.signal()
        self.nodes["controlled_in"]["point"][1] = 2
        context, = self.load()
        self.assertIsNone(context["stop_s"])
        self.nodes["controlled_in"]["point"][1] = 0
        self.nodes["controlled_in"]["traffic_light_id"] = "other_head"
        context, = self.load()
        self.assertIsNone(context["stop_s"])

    def test_stopline_search_does_not_cross_another_controlled_link(self):
        self.predecessor_chain()
        self.signal("earlier_head", ["approach"])
        contexts = {c["link_id"]: c for c in self.load()}
        self.assertEqual(contexts["approach"]["stop_s"], 5.0)
        self.assertIsNone(contexts["controlled"]["stop_s"])

    def test_dynamic_truth_and_signal_phase_fields_are_ignored(self):
        self.link(stop=True)
        light = self.signal()
        reference = self.load()
        light.update(value=float("nan"), dynamic=True, sub_type=["left"],
                     scenario_state="GREEN", heading=float("inf"))
        self.assertEqual(self.load(), reference)
        light.update(value=16, dynamic=False, sub_type=["red"])
        self.assertEqual(self.load(), reference)

    def test_nonfinite_route_coordinates_and_progress_fail_closed(self):
        self.link()
        self.signal()
        for value in (float("nan"), float("inf"), -float("inf"), True, "1"):
            with self.subTest(value=value):
                for axis in range(3):
                    route = self.route[:]
                    point = list(route[20])
                    point[axis] = value
                    route[20] = PathPoint(*point)
                    with self.assertRaises(ValueError):
                        self.load(points=route, s_values=progress(self.route))
                s = progress(self.route)
                s[20] = value
                with self.assertRaises(ValueError):
                    self.load(s_values=s)

    def test_invalid_lengths_progress_and_degenerate_route_fail_closed(self):
        self.link()
        self.signal()
        for points, s in (([], []), ([PathPoint(1, 0)], [0]), (self.route, [0]),
                          (self.route, list(reversed(progress(self.route)))),
                          (self.route, [2 * s for s in progress(self.route)]),
                          ([PathPoint(0, 0), PathPoint(0, 0)], [0, 1]),
                          ([PathPoint(0, 0), PathPoint(0, 0, 1)], [0, 0])):
            with self.subTest(size=len(points), progress_size=len(s)):
                with self.assertRaises(ValueError):
                    self.load(points=points, s_values=s)

    def test_nonfinite_static_map_geometry_fails_closed(self):
        link = self.link()
        light = self.signal()
        targets = [link["points"][0], self.nodes["controlled_in"]["point"], light["point"]]
        for point in targets:
            original = point[:]
            for axis in range(3):
                for value in (float("nan"), float("inf"), "bad", True):
                    with self.subTest(axis=axis, value=value):
                        point[axis] = value
                        with self.assertRaises(ValueError):
                            self.load()
                        point[:] = original

    def test_invalid_map_identifiers_and_stop_flag_fail_closed(self):
        link = self.link()
        light = self.signal()
        self.links.append(copy.deepcopy(link))
        with self.assertRaises(ValueError):
            self.load()
        self.links.pop()
        light["link_id_list"] = ["missing"]
        with self.assertRaises(ValueError):
            self.load()
        light["link_id_list"] = ["controlled"]
        link["to_node_idx"] = "missing_node"
        with self.assertRaises(ValueError):
            self.load()
        link["to_node_idx"] = "controlled_out"
        self.nodes["controlled_in"]["on_stop_line"] = "false"
        with self.assertRaises(ValueError):
            self.load()

    def test_missing_and_malformed_map_do_not_fall_back_to_geometry(self):
        with self.assertRaises(OSError):
            load_route_contexts(self.directory.name, self.route, progress(self.route))
        self.link()
        self.signal()
        self.load()
        (Path(self.directory.name) / "link_set.json").write_text("{}", encoding="utf-8")
        with self.assertRaises(ValueError):
            load_route_contexts(self.directory.name, self.route, progress(self.route))

    def test_only_car_binding_creates_context(self):
        self.link(stop=True)
        self.signal(kind="pedestrian")
        self.assertEqual(self.load(), [])


def repository_smoke():
    map_dir = WORKSPACE / "src/detection/camera_perception/lane/mgeo/R_KR_PR_K-city_2025"
    route_file = WORKSPACE / "data/routes/2026_molit_comp_global_path.txt"
    # Use the actual repository PathPoint loader and duplicate-cleaning policy.
    sys.path.insert(0, str(WORKSPACE / "src/experimental/curvature_speed_purepursuit/src"))
    from curvature_speed_purepursuit.planner import (
        clean_consecutive_duplicates, cumulative_arc_lengths, load_path_file)
    raw = load_path_file(str(route_file))
    route = clean_consecutive_duplicates(raw)
    s = cumulative_arc_lengths(route)
    started = time.perf_counter()
    contexts = load_route_contexts(map_dir, route, s)
    elapsed = time.perf_counter() - started
    links = json.loads((map_dir / "link_set.json").read_text(encoding="utf-8"))
    lights = json.loads((map_dir / "traffic_light_set.json").read_text(encoding="utf-8"))
    linked = {i for signal in lights if signal["type"] == "car" for i in signal["link_id_list"]}
    vertices = defaultdict(list)
    for index, point in enumerate(route):
        vertices[tuple(round(v, 5) for v in (point.x, point.y, point.z))].append(index)
    exact_links = {}
    for link in links:
        if link["idx"] not in linked:
            continue
        hits = [vertices.get(tuple(round(v, 5) for v in point), []) for point in link["points"]]
        if all(len(hit) == 1 for hit in hits):
            indices = [hit[0] for hit in hits]
            if all(b == a + 1 for a, b in zip(indices, indices[1:])):
                exact_links[link["idx"]] = (s[indices[0]], s[indices[-1]])
    by_id = {c["link_id"]: c for c in contexts if c["binding_status"] == "matched"}
    if not exact_links or not exact_links.keys() <= by_id.keys():
        raise AssertionError("Exact ordered map links missing from contexts: " + str(exact_links.keys() - by_id.keys()))
    for identifier, (entry, exit_s) in exact_links.items():
        if abs(by_id[identifier]["entry_s"] - entry) > 1e-6 or abs(by_id[identifier]["end"] - exit_s) > 1e-6:
            raise AssertionError("Context bounds differ from independent route vertex oracle")
    for context in contexts:
        if context["match_quality"]["max_residual_m"] > 0.8:
            raise AssertionError("Residual exceeds the strict limit")
        if context["start"] != (context["stop_s"] if context["stop_s"] is not None else context["entry_s"]):
            raise AssertionError("Guard contract violated")
    intervals = sorted((c["entry_s"], c["end"]) for c in contexts)
    covered, previous_end = 0.0, -math.inf
    for entry, exit_s in intervals:
        covered += max(0.0, exit_s - max(entry, previous_end))
        previous_end = max(previous_end, exit_s)
    report = dict(map_dir=str(map_dir), route_file=str(route_file), map_links=len(links),
                  car_signal_links=len(linked), raw_route_points=len(raw), route_points=len(route),
                  route_length_m=round(s[-1] - s[0], 3), contexts=len(contexts),
                  exact_ordered_signal_links=len(exact_links), exact_links_recovered=len(exact_links.keys() & by_id.keys()),
                  directions=dict(Counter(c["direction"] for c in contexts)),
                  binding_status=dict(Counter(c["binding_status"] for c in contexts)),
                  stopline_status=dict(Counter(c["stopline_status"] for c in contexts)),
                  controlled_route_m=round(covered, 3),
                  controlled_route_percent=round(100 * covered / (s[-1] - s[0]), 3),
                  max_residual_m=max((c["match_quality"]["max_residual_m"] for c in contexts), default=None),
                  load_seconds=round(elapsed, 3))
    print("REPOSITORY_SMOKE " + json.dumps(report, sort_keys=True))
    for context in contexts:
        print("CONTEXT " + json.dumps({key: context[key] for key in
              ("id", "direction", "start", "entry_s", "end", "stop_s", "signal_ids", "binding_status")}, sort_keys=True))
    return report


class RepositoryRouteSmokeTest(unittest.TestCase):
    def test_actual_repository_route_and_map(self):
        map_file = WORKSPACE / "src/detection/camera_perception/lane/mgeo/R_KR_PR_K-city_2025/traffic_light_set.json"
        route_file = WORKSPACE / "data/routes/2026_molit_comp_global_path.txt"
        if not map_file.is_file() or not route_file.is_file():
            self.skipTest("Repository static map/route are not installed")
        self.assertGreater(repository_smoke()["contexts"], 0)


if __name__ == "__main__":
    if "--smoke" in sys.argv:
        repository_smoke()
    else:
        unittest.main()
