"""Competition route wins over signal arrows and conflicting configuration."""
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

import test_maneuver_fusion as fixtures
from curvature_speed_purepursuit.planner import PathPoint
from turn_signal_controller.route_contract import (
    reference_path_reason, route_checked_maneuvers, route_event_key,
)
from turn_signal_controller.scheduler import Maneuver


def reference_message(points):
    return fixtures.Message(poses=[fixtures.Message(pose=NS(position=NS(x=p.x, y=p.y, z=p.z)))
                                    for p in points])


class RouteContractTest(unittest.TestCase):
    def setUp(self):
        self.points = [PathPoint(0., 0.), PathPoint(1., 0.), PathPoint(1., 1., .5)]
        self.context = dict(id="mapped", start=10., end=30., direction="LEFT", source="mgeo",
                            binding_status="matched", link_id="route_link", kind="turn",
                            signal_points=[dict(id="own")], stop_s=10.)
        self.turn = Maneuver("annotation", 10., "LEFT", 30., kind="turn")

    def test_static_reference_accepts_identical_ordered_xyz_without_fresh_timestamp(self):
        message = reference_message(self.points)
        self.assertEqual(reference_path_reason(self.points, message), "matched")

    def test_path_subset_reversal_height_and_frame_cannot_match(self):
        for change, reason in (
            (lambda m: m.poses.pop(), "length_mismatch"),
            (lambda m: m.poses.reverse(), "geometry_mismatch"),
            (lambda m: setattr(m.poses[-1].pose.position, "z", 0.), "geometry_mismatch"),
            (lambda m: setattr(m.poses[1].pose.position, "y", 2.), "geometry_mismatch"),
            (lambda m: setattr(m.header, "frame_id", "odom"), "frame_mismatch"),
            (lambda m: setattr(m.poses[0].header, "frame_id", "odom"), "frame_mismatch"),
            (lambda m: setattr(m.poses[0].pose.position, "x", float("nan")), "invalid"),
            (lambda m: setattr(m.poses[0].pose.position, "z", float("inf")), "invalid"),
        ):
            with self.subTest(reason=reason):
                message = reference_message(self.points)
                change(message)
                self.assertEqual(reference_path_reason(self.points, message), "reference_path_" + reason)
        self.assertEqual(reference_path_reason(self.points, NS()), "reference_path_invalid")

    def test_conflicting_manual_turn_never_overrides_straight_or_right_route(self):
        for direction in ("STRAIGHT", "RIGHT", "UNKNOWN"):
            with self.subTest(direction=direction), self.assertRaisesRegex(ValueError, "conflicts"):
                route_checked_maneuvers([self.turn], [dict(self.context, direction=direction)])

    def test_manual_turn_requires_one_verified_mapped_movement(self):
        for contexts in ([], [self.context, dict(self.context, id="other")],
                         [dict(self.context, binding_status="ambiguous")],
                         [dict(self.context, signal_points=[])], [dict(self.context, link_id=None)]):
            with self.subTest(contexts=contexts), self.assertRaises(ValueError):
                route_checked_maneuvers([self.turn], contexts)

    def test_matching_manual_turn_does_not_duplicate_mapped_event(self):
        lane = Maneuver("change", 40., "RIGHT", 50., kind="lane_change")
        self.assertEqual(route_checked_maneuvers([self.turn, lane], [self.context]), [lane])

    def test_lane_change_cannot_override_signal_controlled_movement(self):
        with self.assertRaisesRegex(ValueError, "Lane-change"):
            route_checked_maneuvers([Maneuver("change", 15., "RIGHT", 25.)], [self.context])

    def test_permission_identity_ignores_camera_distance_but_binds_route_movement(self):
        key = route_event_key(self.context)
        self.assertEqual(key, route_event_key(dict(self.context, start=9., camera_line_confirmed=True)))
        for field, value in (("direction", "RIGHT"), ("link_id", "other"), ("end", 31.),
                             ("stop_s", 11.), ("signal_points", [dict(id="other")])):
            with self.subTest(field=field):
                self.assertNotEqual(key, route_event_key(dict(self.context, **{field: value})))


class RouteAuthorityIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.case = fixtures.StrictFusionTest()
        self.case.setUp()
        self.node = self.case.node
        self.node.require_reference_path = True

    def match_path(self):
        self.node.reference_path_callback(reference_message(self.node.points))

    def test_strict_startup_requires_reference_from_steering(self):
        self.case.fixture.params["~require_route_signal_context"] = True
        node = self.case.fixture.module.ManeuverFusionNode()
        self.assertTrue(node.require_reference_path)
        output, status = self.case.frame()
        self.assertEqual((output.accel, output.brake), (0., 1.))
        self.assertIn("reference_path_not_received", status["reason"])
        self.assertFalse(status["permission"])
        self.match_path()
        for _ in range(120):
            output, status = self.case.frame()
        self.assertTrue(status["reference_path_match"])
        self.assertTrue(status["permission"])

    def test_route_direction_signal_matrix_preserves_nominal_steering(self):
        permissions = {"GREEN": {"STRAIGHT", "RIGHT"},
                       "GREEN_LEFT": {"STRAIGHT", "LEFT", "RIGHT"},
                       "GREEN_RIGHT": {"STRAIGHT", "RIGHT"},
                       "LEFT": {"LEFT"}, "RED_LEFT": {"LEFT"},
                       "RIGHT": {"RIGHT"}, "RED_RIGHT": {"RIGHT"},
                       "RED": set(), "YELLOW": set(), "UNKNOWN": set()}
        for direction in ("STRAIGHT", "LEFT", "RIGHT"):
            for signal, allowed in permissions.items():
                with self.subTest(route=direction, signal=signal):
                    self.setUp()
                    self.match_path()
                    self.case.context["direction"] = direction
                    for _ in range(120):
                        output, status = self.case.frame(signal)
                    self.assertEqual(status["route_direction"], direction)
                    self.assertEqual(status["permission"], direction in allowed)
                    self.assertEqual(status["route_signal_compatible"], direction in allowed)
                    self.assertEqual(status["signal_allowed_directions"], sorted(allowed))
                    self.assertEqual(status["lamp_requested"], "OFF" if direction == "STRAIGHT" else direction)
                    self.assertEqual(output.steering, fixtures.Command().steering)
                    self.assertEqual(output.brake, 0. if direction in allowed else 1.)

    def test_path_mismatch_cancels_permission_and_transient_recovery_needs_new_evidence(self):
        self.match_path()
        self.case.authorize()
        self.assertTrue(self.node.enter_permission)
        wrong = reference_message(self.node.points)
        wrong.poses[-1].pose.position.y += 10.
        self.node.reference_path_callback(wrong)
        self.assertFalse(self.node.enter_permission)
        self.assertIsNone(self.node.entry_ticket)
        output, status = self.case.frame()
        self.assertIn("reference_path_geometry_mismatch", status["reason"])
        self.assertEqual(output.brake, 1.)
        self.match_path()
        for _ in range(120):
            output, status = self.case.frame()
        self.assertTrue(status["permission"])
        # Both callbacks arrive between control ticks; stale permission still dies.
        self.node.reference_path_callback(wrong)
        self.match_path()
        output, status = self.case.frame()
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)

    def test_matching_reference_republication_does_not_reset_confirmation(self):
        self.match_path()
        self.case.authorize()
        self.match_path()
        _, status = self.case.frame()
        self.assertTrue(status["permission"])

    def test_blackout_cannot_bypass_divergent_steering_route(self):
        self.match_path()
        self.case.corridor_frame()
        output, status = self.case.corridor_frame("GPS_BLACKOUT", "gps_blackout_camera_fallback", x=10.1)
        self.assertTrue(status["blackout_lane_corridor"])
        wrong = reference_message(self.node.points)
        wrong.poses.reverse()
        self.node.reference_path_callback(wrong)
        output, status = self.case.corridor_frame("GPS_BLACKOUT", "gps_blackout_camera_fallback", x=10.2)
        self.assertFalse(status["blackout_lane_corridor"])
        self.assertEqual((output.accel, output.brake), (0., 1.))
        self.match_path()
        _, status = self.case.corridor_frame("GPS_BLACKOUT", "gps_blackout_camera_fallback", x=10.3)
        self.assertFalse(status["blackout_lane_corridor"])  # needs a new normal anchor

    def test_same_event_id_changed_direction_cannot_reuse_green_left_permission(self):
        self.match_path()
        self.case.context["direction"] = "STRAIGHT"
        for _ in range(20):
            _, status = self.case.frame("GREEN_LEFT")
        self.assertTrue(status["permission"])
        self.node.event["direction"] = "LEFT"
        output, status = self.case.frame("GREEN_LEFT")
        self.assertFalse(status["permission"])
        self.assertFalse(status["indicator_ready"])
        self.assertEqual(output.brake, 1.)
        self.assertIsNone(self.node.entry_ticket)
        for _ in range(120):
            _, status = self.case.frame("GREEN_LEFT")
        self.assertTrue(status["permission"])

    def test_approach_ticket_cannot_transfer_to_another_link_with_same_id(self):
        self.match_path()
        self.case.authorize()
        self.node.event["link_id"] = "another_link"
        self.node.progress = 87.1
        now = self.case.fixture.now
        self.assertFalse(self.node.entry_candidate(self.node.event, True, True, True, now, now))

    def test_committed_straight_does_not_switch_to_later_left_arrow(self):
        self.match_path()
        self.case.context["direction"] = "STRAIGHT"
        for _ in range(20):
            self.case.frame("GREEN")
        self.case.frame("GREEN", x=87.1)
        output, status = self.case.frame("LEFT", x=87.2)
        self.assertTrue(status["event"]["committed"])
        self.assertEqual(status["route_direction"], "STRAIGHT")
        self.assertEqual(status["lamp_requested"], "OFF")
        self.assertEqual(output.steering, fixtures.Command().steering)

    def test_constructor_rejects_conflict_and_keeps_only_mapped_turn(self):
        fixture = self.case.fixture
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, "traffic_light_set.json").write_text("[]", encoding="utf-8")
            context = dict(self.case.context, direction="STRAIGHT", binding_status="matched", link_id="link")
            fixture.module.load_route_contexts = lambda *_: [context]
            fixture.params.update({"~require_route_signal_context": True, "~signal_mgeo_path": directory,
                                   "~maneuvers": [dict(id="manual", kind="turn", direction="LEFT",
                                                       start_s_m=90., end_s_m=125.)]})
            with self.assertRaisesRegex(ValueError, "conflicts with the competition route"):
                fixture.module.ManeuverFusionNode()
            context["direction"] = "LEFT"
            node = fixture.module.ManeuverFusionNode()
            self.assertEqual(node.manual, [])
            self.assertEqual(node.contexts, [context])


if __name__ == "__main__":
    unittest.main()
