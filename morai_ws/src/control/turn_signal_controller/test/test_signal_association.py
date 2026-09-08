"""Signal association and source-frame confirmation; no ROS installation needed."""

from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace as ObjectInfo
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from turn_signal_controller.signal_association import (
    Selection, SignalConfirmation, associate, project_signal,
)


def detection(name="Green", x=320.0, y=240.0, width=10.0, height=20.0, conf=0.9):
    return ObjectInfo(class_name=name, x_center=x, y_center=y,
                      width=width, height=height, conf=conf)


def observation(state="GREEN", head_id="own", confidence=0.9):
    return Selection(state, confidence, True, head_id, "matched")


class ProjectionTest(unittest.TestCase):
    def setUp(self):
        self.point = dict(x=10.0, y=0.0, z=0.0)
        self.pose = dict(x=0.0, y=0.0, z=0.0, yaw=0.0)
        self.camera = dict(width=640, height=480, horizontal_fov_deg=90.0,
                           x=0.0, y=0.0, z=0.0, pitch_deg=0.0, yaw_deg=0.0)

    def assertPixel(self, actual, expected):
        self.assertIsNotNone(actual)
        for value, reference in zip(actual, expected):
            self.assertAlmostEqual(value, reference, places=7)

    def test_center_and_left_up_axes(self):
        self.assertPixel(project_signal(self.point, self.pose, self.camera), (320, 240))
        self.assertPixel(project_signal(dict(x=10, y=1, z=2), self.pose, self.camera), (288, 176))

    def test_world_translation_and_ego_yaw(self):
        pose = dict(x=100, y=200, z=3, yaw=math.pi / 2)
        self.assertPixel(project_signal(dict(x=100, y=210, z=3), pose, self.camera), (320, 240))
        # Same point moves right in the image when the vehicle yaws left.
        pixel = project_signal(self.point, dict(self.pose, yaw=0.1), self.camera)
        self.assertGreater(pixel[0], 320)

    def test_mount_translation_is_in_rotated_body_frame(self):
        pose = dict(x=100, y=200, z=3, yaw=math.pi / 2)
        camera = dict(self.camera, x=2, y=1, z=1)
        # In body coordinates the head is (12, 2, 3), camera-to-head (10, 1, 2).
        self.assertPixel(project_signal(dict(x=98, y=212, z=6), pose, camera), (288, 176))

    def test_camera_positive_pitch_points_up(self):
        camera = dict(self.camera, pitch_deg=30)
        self.assertPixel(project_signal(dict(x=10, y=0, z=10 / math.sqrt(3)),
                                        self.pose, camera), (320, 240))
        self.assertGreater(project_signal(self.point, self.pose, camera)[1], 240)

    def test_camera_positive_yaw_points_left(self):
        camera = dict(self.camera, yaw_deg=30)
        self.assertPixel(project_signal(dict(x=10, y=10 / math.sqrt(3), z=0),
                                        self.pose, camera), (320, 240))

    def test_pose_pitch_uses_ros_positive_nose_down(self):
        pose = dict(self.pose, pitch=math.pi / 6)
        self.assertPixel(project_signal(dict(x=10, y=0, z=-10 / math.sqrt(3)),
                                        pose, self.camera), (320, 240))

    def test_pose_roll_rotates_left_and_up(self):
        pose = dict(self.pose, roll=math.pi / 2)
        self.assertPixel(project_signal(dict(x=10, y=0, z=1), pose, self.camera), (288, 240))

    def test_combined_rpy_and_camera_mount(self):
        # Independent forward rotation: construct a map head from a known
        # camera-space point, then check the inverse projection against pixels.
        def rx(v, a):
            x, y, z = v
            return x, y * math.cos(a) - z * math.sin(a), y * math.sin(a) + z * math.cos(a)

        def ry(v, a):
            x, y, z = v
            return x * math.cos(a) + z * math.sin(a), y, -x * math.sin(a) + z * math.cos(a)

        def rz(v, a):
            x, y, z = v
            return x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a), z

        pose = dict(x=103, y=-71, z=6, roll=0.13, pitch=-0.21, yaw=0.8)
        camera = dict(self.camera, x=2.1, y=-0.2, z=1.7, yaw_deg=-12, pitch_deg=8)
        body = rz(ry((20, 2, 1), math.radians(-8)), math.radians(-12))
        body = tuple(body[i] + camera[k] for i, k in enumerate(("x", "y", "z")))
        world = rz(ry(rx(body, pose["roll"]), pose["pitch"]), pose["yaw"])
        point = {k: world[i] + pose[k] for i, k in enumerate(("x", "y", "z"))}
        self.assertPixel(project_signal(point, pose, camera), (288, 224))

    def test_calibration_dimensions_and_fov_change_projection(self):
        camera = dict(self.camera, width=1280, height=720, horizontal_fov_deg=60)
        expected = (640 - 64 * math.sqrt(3), 360 - 128 * math.sqrt(3))
        self.assertPixel(project_signal(dict(x=10, y=1, z=2), self.pose, camera), expected)

    def test_missing_camera_or_any_required_calibration_fails_closed(self):
        for camera in (None, {}, False):
            with self.subTest(camera=camera):
                self.assertIsNone(project_signal(self.point, self.pose, camera))
        for key in self.camera:
            camera = self.camera.copy()
            del camera[key]
            with self.subTest(missing=key):
                self.assertIsNone(project_signal(self.point, self.pose, camera))
        for original, index in ((self.point, 0), (self.pose, 1)):
            for key in original:
                args = [self.point, self.pose, self.camera]
                args[index] = {k: v for k, v in original.items() if k != key}
                with self.subTest(missing=key, argument=index):
                    self.assertIsNone(project_signal(*args))

    def test_invalid_numeric_projection_inputs(self):
        for index, original in enumerate((self.point, dict(self.pose, roll=0, pitch=0), self.camera)):
            for key in original:
                for bad in (float("nan"), float("inf"), -float("inf"), None, True, "invalid"):
                    args = [self.point, self.pose, self.camera]
                    args[index] = dict(original, **{key: bad})
                    with self.subTest(argument=index, key=key, bad=bad):
                        self.assertIsNone(project_signal(*args))
        for key, value in (("width", 0), ("width", 1.5), ("height", -1),
                           ("horizontal_fov_deg", 0), ("horizontal_fov_deg", 180)):
            self.assertIsNone(project_signal(self.point, self.pose, dict(self.camera, **{key: value})))

    def test_behind_optical_plane_and_outside_image(self):
        for point in (dict(x=-10, y=0, z=0), dict(x=0, y=0, z=0),
                      dict(x=10, y=-20, z=0), dict(x=10, y=0, z=20)):
            with self.subTest(point=point):
                self.assertIsNone(project_signal(point, self.pose, self.camera))
        self.assertIsNone(project_signal(self.point, self.pose, dict(self.camera, yaw_deg=180)))


class AssociationTest(unittest.TestCase):
    def assertUnknown(self, selection, reason=None):
        self.assertEqual((selection.state, selection.confidence, selection.valid, selection.selected_id),
                         ("UNKNOWN", 0.0, False, None))
        if reason:
            self.assertEqual(selection.reason, reason)

    def test_wrong_lane_green_and_screen_center_cannot_override_own_red(self):
        own = detection("Red", x=120, conf=0.6)
        other = detection("Green", x=320, conf=0.99)
        for objects in ([other, own], [own, other]):
            selected = associate(objects, {"own": (120, 240)}, {"other": (320, 240)})
            self.assertEqual(selected, Selection("RED", 0.6, True, "own", "matched"))
        self.assertUnknown(associate([other], {"own": (120, 240)}), "no_match")

    def test_oblique_narrow_bbox_accepts_projection_near_its_edge(self):
        # The map anchor is displaced from a narrow visible signal face.
        result = associate([detection(x=90, width=4)], {"own": (114, 240)})
        self.assertTrue(result.valid)
        self.assertEqual(result.selected_id, "own")

    def test_gate_is_capped_at_25_pixels_per_bbox_side(self):
        box = detection(width=4, height=6)
        self.assertTrue(associate([box], {"own": (347, 268)}).valid)
        for pixel in ((347.01, 240), (320, 268.01)):
            self.assertUnknown(associate([box], {"own": pixel}), "no_match")

    def test_conflicting_detections_at_target_are_ambiguous_regardless_of_score(self):
        for name in ("Red", "Green", "Left", "unsupported"):
            with self.subTest(name=name):
                self.assertUnknown(associate([detection(conf=0.99), detection(name, conf=0.5)],
                                             {"own": (320, 240)}), "ambiguous_match")

    def test_one_box_matching_multiple_route_heads_is_ambiguous(self):
        self.assertUnknown(associate([detection()], {"one": (320, 240), "two": (330, 240)}),
                           "ambiguous_match")

    def test_separate_matches_to_two_route_heads_are_also_ambiguous(self):
        self.assertUnknown(associate([detection(x=100), detection(x=400)],
                                     {"one": (100, 240), "two": (400, 240)}), "ambiguous_match")

    def test_plausible_other_head_cannot_be_disambiguated_by_nearest_center(self):
        for name in ("Green", "Red"):
            self.assertUnknown(associate([detection(name)], {"own": (320, 240)},
                                         projected_others={"other": (344, 240)}), "ambiguous_other_head")
        self.assertTrue(associate([detection()], {"own": (320, 240)}, {"far": (450, 240)}).valid)

    def test_same_id_in_all_heads_is_not_a_competitor_but_must_agree(self):
        targets = {"own": (320, 240)}
        self.assertTrue(associate([detection()], targets, dict(targets, far=(500, 240))).valid)
        self.assertUnknown(associate([detection()], targets, {"own": (321, 240)}), "invalid_projection")

    def test_missing_targets_and_unprojectable_heads_never_fall_back(self):
        for targets in ({}, {"own": None}):
            self.assertUnknown(associate([detection()], targets, {"other": (320, 240)}),
                               "no_projected_targets")
        self.assertTrue(associate([detection()], {"own": (320, 240), "behind": None}).valid)
        self.assertUnknown(associate([], {"own": (320, 240)}), "no_match")

    def test_invalid_projection_in_either_map_fails_closed(self):
        for bad in (None, [], {"own": (float("nan"), 240)}, {"own": (320, float("inf"))},
                    {"own": (320,)}, {"own": (-1, 240)}, {None: (320, 240)}):
            with self.subTest(bad=bad):
                self.assertUnknown(associate([detection()], bad), "invalid_projection")
                if bad is not None:
                    self.assertUnknown(associate([detection()], {"valid": (320, 240)}, bad),
                                       "invalid_projection")

    def test_invalid_boxes_and_confidences_cannot_produce_permission(self):
        for key in ("x_center", "y_center", "width", "height", "conf"):
            for value in (float("nan"), float("inf"), -float("inf"), None, True):
                invalid = detection()
                setattr(invalid, key, value)
                with self.subTest(key=key, value=value):
                    self.assertUnknown(associate([invalid, detection()], {"own": (320, 240)}),
                                       "invalid_object")
        for box in (detection(conf=1.01), detection(conf=-0.1), detection(width=0),
                    detection(height=-1), ObjectInfo(conf=0.9)):
            self.assertUnknown(associate([box], {"own": (320, 240)}), "invalid_object")
        self.assertUnknown(associate(None, {"own": (320, 240)}), "invalid_object")

    def test_confidence_threshold_and_supported_directional_states(self):
        self.assertUnknown(associate([detection(conf=0.49)], {"own": (320, 240)}), "no_match")
        for name, expected in (("Red_Left", "RED_LEFT"), ("green left", "GREEN_LEFT"),
                               ("Green_Right", "GREEN_RIGHT"), ("Red_Right", "RED_RIGHT"),
                               ("Left", "LEFT"), ("Right", "RIGHT"), ("Amber", "YELLOW"),
                               ("Yellow_Left", "YELLOW")):
            with self.subTest(name=name):
                selection = associate([detection(name, conf=0.5)], {"own": (320, 240)})
                self.assertTrue(selection.valid)
                self.assertEqual(selection.state, expected)
        for name in ("Green_Arrow", "Arrow", "traffic_light", None):
            self.assertUnknown(associate([detection(name)], {"own": (320, 240)}), "unsupported_class")


class ConfirmationTest(unittest.TestCase):
    def setUp(self):
        self.confirmation = SignalConfirmation()
        self.green = observation()

    def confirm(self, selection=None, start=100.0):
        selection = self.green if selection is None else selection
        self.assertFalse(self.confirmation.update(selection, start).valid)
        self.assertFalse(self.confirmation.update(selection, start + 0.15).valid)
        result = self.confirmation.update(selection, start + 0.3)
        self.assertTrue(result.valid)
        self.assertEqual(result.reason, "signal_confirmed")
        return result

    def test_three_distinct_frames_and_duration_are_both_required(self):
        first = self.confirmation.update(self.green, 10.0)
        self.assertEqual(first, replace(self.green, valid=False, reason="signal_unconfirmed"))
        self.assertFalse(self.confirmation.update(self.green, 10.1).valid)
        self.assertFalse(self.confirmation.update(self.green, 10.2).valid)
        self.assertTrue(self.confirmation.update(self.green, 10.3).valid)
        self.confirmation.reset()
        self.assertFalse(self.confirmation.update(self.green, 10.0).valid)
        self.assertFalse(self.confirmation.update(self.green, 10.4).valid)  # Only two frames.
        self.assertTrue(self.confirmation.update(self.green, 10.45).valid)

    def test_repeated_control_ticks_never_count_as_new_frames(self):
        for stamp in (100.0, 100.15):
            for _ in range(50):
                pending = self.confirmation.update(self.green, stamp)
                self.assertFalse(pending.valid)
                self.assertEqual(pending.reason, "signal_unconfirmed")
        self.assertTrue(self.confirmation.update(self.green, 100.3).valid)
        for _ in range(50):
            self.assertTrue(self.confirmation.update(self.green, 100.3).valid)

    def test_state_or_target_change_starts_new_confirmation(self):
        for changed in (observation("GREEN_LEFT"), observation(head_id="next")):
            with self.subTest(changed=changed):
                self.confirmation.reset()
                self.confirm()
                self.confirm(changed, start=100.4)

    def test_confidence_can_change_between_distinct_source_frames(self):
        self.assertFalse(self.confirmation.update(observation(confidence=0.9), 100.0).valid)
        self.assertFalse(self.confirmation.update(observation(confidence=0.7), 100.15).valid)
        result = self.confirmation.update(observation(confidence=0.6), 100.3)
        self.assertTrue(result.valid)
        self.assertEqual(result.confidence, 0.6)

    def test_directional_permissions_also_require_confirmation(self):
        for state in ("GREEN_LEFT", "GREEN_RIGHT", "LEFT", "RIGHT", "RED_LEFT", "RED_RIGHT"):
            with self.subTest(state=state):
                self.confirmation.reset()
                self.assertEqual(self.confirm(observation(state)).state, state)

    def test_red_and_yellow_revoke_immediately_then_green_must_reconfirm(self):
        for state in ("RED", "YELLOW"):
            for stamp in (100.3, 100.31):  # Also revoke on contradictory duplicate evidence.
                with self.subTest(state=state, stamp=stamp):
                    self.confirmation.reset()
                    self.confirm()
                    stop = self.confirmation.update(observation(state), stamp)
                    self.assertTrue(stop.valid)
                    self.assertEqual((stop.state, stop.reason), (state, "signal_immediate"))
                    self.confirm(start=100.4)

    def test_unknown_resets_without_holding_previous_red_or_green(self):
        for before in ("GREEN", "RED"):
            self.confirmation.reset()
            if before == "GREEN":
                self.confirm()
            else:
                self.confirmation.update(observation("RED"), 100.3)
            unknown = self.confirmation.update(Selection(reason="no_match"), 100.4)
            self.assertEqual(unknown, Selection(reason="no_match"))
            self.confirm(start=100.5)

    def test_long_time_gap_resets_confirmed_and_pending_evidence(self):
        for was_confirmed in (False, True):
            self.confirmation.reset()
            if was_confirmed:
                self.confirm()
            else:
                self.confirmation.update(self.green, 100.3)
            self.confirm(start=100.801)
        self.confirmation.reset()
        self.confirmation.update(self.green, 0.0)
        self.confirmation.update(self.green, 0.5)
        self.assertTrue(self.confirmation.update(self.green, 1.0).valid)  # Exactly .5 is allowed.

    def test_reordered_stale_frames_revoke_and_do_not_rewind_watermark(self):
        self.confirm()
        for stamp in (100.2, 99.0, 99.2, 99.4):
            result = self.confirmation.update(self.green, stamp)
            self.assertFalse(result.valid)
            self.assertEqual(result.reason, "signal_out_of_order")
        self.assertFalse(self.confirmation.update(self.green, 100.3).valid)
        self.confirm(start=100.4)

    def test_changed_permission_on_duplicate_stamp_resets_evidence(self):
        self.confirm()
        result = self.confirmation.update(observation("LEFT"), 100.3)
        self.assertEqual(result, Selection(reason="signal_duplicate_stamp"))
        self.assertFalse(self.confirmation.update(self.green, 100.3).valid)
        self.confirm(start=100.4)

    def test_invalid_stamp_revokes_without_forgetting_previous_watermark(self):
        for stamp in (float("nan"), float("inf"), -1, None, True):
            with self.subTest(stamp=stamp):
                self.confirmation.reset()
                self.confirm()
                self.assertEqual(self.confirmation.update(self.green, stamp), Selection(reason="invalid_stamp"))
                self.assertFalse(self.confirmation.update(self.green, 99.0).valid)
                self.confirm(start=100.4)

    def test_invalid_selection_confidence_and_structure_reset(self):
        for invalid in (replace(self.green, confidence=float("nan")),
                        replace(self.green, confidence=1.01), replace(self.green, confidence=-0.1),
                        replace(self.green, confidence=0.1), replace(self.green, selected_id=None),
                        replace(self.green, valid="true"), replace(self.green, state="GREEN_ARROW"),
                        replace(self.green, state="UNKNOWN"), None):
            with self.subTest(invalid=invalid):
                self.confirmation.reset()
                self.confirm()
                self.assertEqual(self.confirmation.update(invalid, 100.4), Selection(reason="invalid_selection"))
                self.confirm(start=100.5)

    def test_reset_accepts_a_new_clock_epoch(self):
        self.confirm()
        self.confirmation.reset()
        self.confirm(start=0.0)

    def test_duration_comparison_handles_epoch_timestamps(self):
        self.confirm(start=1_789_000_000.0)

    def test_stricter_configuration_and_invalid_configuration(self):
        self.confirmation = SignalConfirmation(min_frames=4, min_duration_sec=0.4, max_gap_sec=0.2)
        for stamp in (0.0, 0.15, 0.3):
            self.assertFalse(self.confirmation.update(self.green, stamp).valid)
        self.assertTrue(self.confirmation.update(self.green, 0.45).valid)
        for config in (dict(min_frames=2), dict(min_frames=3.5), dict(min_frames=True),
                       dict(min_duration_sec=0.2), dict(min_duration_sec=float("nan")),
                       dict(max_gap_sec=0), dict(max_gap_sec=float("inf"))):
            with self.subTest(config=config), self.assertRaises(ValueError):
                SignalConfirmation(**config)


if __name__ == "__main__":
    unittest.main()
