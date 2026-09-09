"""Integration scenarios with real callbacks and a mocked ROS/UDP boundary."""
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE.parents[1]
for relative in ("control/turn_signal_controller/src", "control/stopline_control/src",
                 "experimental/curvature_speed_purepursuit/src", "detection/camera_perception/src"):
    sys.path.insert(0, str(SOURCE / relative))
from turn_signal_controller.fusion import IndicatorLead, build_lamp_packet, route_intent, signal_permits
from camera_perception.traffic_signal import directional_observation
from curvature_speed_purepursuit.planner import PathPoint, cumulative_arc_lengths
from stopline_control.core import StopLineControllerCore


class Stamp:
    def __init__(self, seconds=0.0):
        self.seconds = seconds
    def to_sec(self):
        return self.seconds


class Message:
    def __init__(self, **kwargs):
        self.header = NS(stamp=Stamp(), frame_id="map")
        self.__dict__.update(kwargs)


class Command:
    def __init__(self, **kwargs):
        self.longlCmdType = 1
        self.accel, self.brake, self.steering = 0.6, 0.0, 0.12
        self.velocity = self.acceleration = 0.0
        self.__dict__.update(kwargs)


def turn_path(sign=1):
    points = [PathPoint(float(x), 0) for x in range(101)]
    for i in range(1, 91):
        a = math.radians(i)
        points.append(PathPoint(100 + 15 * math.sin(a), sign * 15 * (1 - math.cos(a))))
    points.extend(PathPoint(115, sign * y) for y in range(16, 101))
    return points


class DirectionContractTest(unittest.TestCase):
    def test_arrows_and_straight_green_are_not_interchangeable(self):
        self.assertTrue(signal_permits("RED_LEFT", "LEFT"))
        self.assertFalse(signal_permits("RED_LEFT", "STRAIGHT"))
        self.assertFalse(signal_permits("GREEN", "LEFT"))
        self.assertTrue(signal_permits("GREEN", "RIGHT"))
        self.assertFalse(signal_permits("GREEN", "RIGHT", right_on_green=False))
        for state in ("YELLOW", "RED", "UNKNOWN", "GREEN_ARROW"):
            for direction in ("LEFT", "RIGHT", "STRAIGHT"):
                self.assertFalse(signal_permits(state, direction))

    def test_conflicting_signal_heads_do_not_create_green_permission(self):
        def detect(*names):
            return directional_observation([NS(class_name=name, conf=0.9) for name in names])
        self.assertEqual(detect("Green_Left"), ("GREEN_LEFT", 0.9))
        self.assertEqual(detect("Red_Left"), ("RED_LEFT", 0.9))
        self.assertEqual(detect("Red", "Left"), ("UNKNOWN", 0.0))
        self.assertEqual(detect("Red", "Green"), ("UNKNOWN", 0.0))
        self.assertEqual(detect("Yellow_Left"), ("YELLOW", 0.9))
        self.assertEqual(directional_observation([NS(class_name="Green", conf=float("nan"))]), ("UNKNOWN", 0.0))

    def test_route_intent_left_right_straight_and_insufficient_path(self):
        for sign, expected in ((1, "LEFT"), (-1, "RIGHT")):
            points = turn_path(sign)
            s = cumulative_arc_lengths(points)
            self.assertEqual(route_intent(points, s, 90).direction, expected)
            self.assertEqual(route_intent(points, s, 10).direction, "STRAIGHT")
            self.assertEqual(route_intent(points, s, s[-1] - 1).direction, "UNKNOWN")

    def test_lamp_packet_matches_bundled_ctypes_definition(self):
        # Independent protocol oracle shipped with the repo.
        import ctypes
        with patch.object(sys, "path", [str(SOURCE / "common/morai_network")] + sys.path):
            from lib.define.TurnSignalLampControl import TurnSignalLampControl
        for direction, code in (("OFF", 0), ("LEFT", 1), ("RIGHT", 2)):
            reference = TurnSignalLampControl()
            reference.turnsignal = code
            packet = build_lamp_packet(direction)
            self.assertEqual(len(packet), 33)
            self.assertEqual(packet, ctypes.string_at(ctypes.addressof(reference), ctypes.sizeof(reference)))

    def test_five_seconds_requires_continuous_successful_sends_and_both_clocks(self):
        lead = IndicatorLead()
        for i in range(101):
            t = i * 0.05
            lead.sent("LEFT", t, t)
            self.assertEqual(lead.ready("LEFT", t, t), i == 100)
        lead.sent("RIGHT", 5.05, 5.05)
        self.assertFalse(lead.ready("RIGHT", 5.05, 5.05))
        lead.sent("RIGHT", 5.10, 5.10, success=False)
        self.assertFalse(lead.ready("RIGHT", 5.10, 5.10))
        for i in range(120):
            lead.sent("LEFT", 6 + i * 0.05, 5.1)  # simulation paused
        self.assertFalse(lead.ready("LEFT", 11.95, 5.1))

    def test_indicator_clock_jump_or_invalid_direction_revokes_lead(self):
        for clock in ("wall", "ros"):
            lead = IndicatorLead()
            for i in range(101):
                lead.sent("LEFT", 100 + i * .05, 100 + i * .05)
            wall, ros = (106., 105.05) if clock == "wall" else (105.05, 106.)
            self.assertFalse(lead.ready("LEFT", wall, ros))
            lead.sent("LEFT", wall, ros)
            self.assertFalse(lead.ready("LEFT", wall, ros))
        lead.sent("HAZARD", 106.05, 106.05)
        self.assertFalse(lead.ready("HAZARD", 112., 112.))
        self.assertEqual(lead.direction, "OFF")

    def test_indicator_rejects_nonfinite_clocks_and_bad_gap_configuration(self):
        for gap in (0., -1., float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                IndicatorLead(max_gap_sec=gap)
        lead = IndicatorLead()
        lead.sent("LEFT", 100., 100.)
        lead.sent("LEFT", 100.05, float("nan"))
        self.assertEqual(lead.direction, "OFF")


class FusionNodeTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.params = {"~path_file": "fixture", "~lamp_output_enabled": True,
                       "~require_route_signal_context": False,
                       "~require_fresh_camera_stream": False}
        self.ros = Mock()
        self.ros.get_param.side_effect = lambda key, default=None: self.params.get(key, default)
        self.ros.get_time.side_effect = lambda: self.now
        self.ros.Publisher.side_effect = lambda *_a, **_k: Mock()
        modules = {"rospy": self.ros, "morai_msgs.msg": NS(CtrlCmd=Command),
                   "common.msg": NS(ObjectInfoArray=Message),
                   "morai_perception_msgs.msg": NS(TrafficLight=Message, StopLineDetection=Message,
                                                    SafetyStop=Message, SensorQuality=Message),
                   "nav_msgs.msg": NS(Odometry=Message, Path=Message), "std_msgs.msg": NS(String=Message)}
        spec = importlib.util.spec_from_file_location("_fusion_test_node", PACKAGE / "scripts/maneuver_fusion_node.py")
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(self.module)
        self.module.time = NS(monotonic=lambda: self.now)
        self.socket = Mock()
        self.socket.sendto.return_value = 33
        self.module.socket = NS(socket=lambda *_: self.socket, AF_INET=2, SOCK_DGRAM=2)
        self.module.load_path_file = lambda _: turn_path()
        self.node = self.module.ManeuverFusionNode()

    def sample(self, key, frame="map", stamp=None, **kwargs):
        message = Message(header=NS(stamp=Stamp(self.now if stamp is None else stamp), frame_id=frame), **kwargs)
        self.node.observe(message, key)

    def tick(self, signal="LEFT", line=4.345, obstacle=False, quality="NORMAL",
             x=85.655, speed=0.0, refresh=True):
        if refresh:
            self.sample("odom", pose=NS(pose=NS(position=NS(x=x, y=0.),
                        orientation=NS(x=0., y=0., z=0., w=1.))),
                        twist=NS(twist=NS(linear=NS(x=speed, y=0.))))
            self.sample("quality", state=quality)
            self.sample("safety", stop_required=obstacle, reason="pedestrian" if obstacle else "clear")
            if signal is not None:
                self.sample("signal", state=signal, confidence=0.9, valid=signal != "UNKNOWN")
            if line is not None:
                self.sample("line", "base_link", distance_m=line, confidence=1.0, valid=True)
        self.node.command_callback(Command())
        self.node.tick(None)
        output = self.node.output_pub.publish.call_args.args[0]
        status = json.loads(self.node.state_pub.publish.call_args.args[0].data)
        return output, status

    def run_ticks(self, count, **kwargs):
        result = None
        for _ in range(count):
            result = self.tick(**kwargs)
            self.now = round(self.now + 0.05, 5)
        return result

    def test_matching_arrow_waits_five_seconds_and_stops_at_bumper_target(self):
        output, status = self.run_ticks(100)
        self.assertEqual(status["direction"], "LEFT")
        self.assertFalse(status["indicator_ready"])
        self.assertAlmostEqual(status["front_bumper_distance_m"], 0.5)
        self.assertEqual(output.brake, 1.0)
        output, status = self.run_ticks(10)
        self.assertTrue(status["indicator_ready"])
        self.assertTrue(status["permission"])
        self.assertEqual(output.brake, 0.0)

    def test_straight_green_does_not_authorize_left_turn(self):
        output, status = self.run_ticks(130, signal="GREEN")
        self.assertTrue(status["indicator_ready"])
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.0)

    def test_right_turn_and_direction_mismatch(self):
        self.module.load_path_file = lambda _: turn_path(-1)
        self.node = self.module.ManeuverFusionNode()
        output, status = self.run_ticks(110, signal="GREEN")
        self.assertEqual(status["direction"], "RIGHT")
        self.assertTrue(status["permission"])
        self.assertEqual(output.brake, 0.0)
        output, status = self.tick(signal="LEFT")
        self.assertEqual(output.brake, 1.0)
        self.assertFalse(status["permission"])

    def test_lane_change_needs_its_own_side_gap_and_five_seconds(self):
        self.params["~maneuvers"] = [{"id": "left_change", "kind": "lane_change",
                                     "start_s_m": 100., "end_s_m": 120., "direction": "LEFT"}]
        self.node = self.module.ManeuverFusionNode()
        output, status = self.run_ticks(110, signal=None, line=None, x=95.)
        self.assertEqual(output.brake, 1.0)
        self.assertIn("adjacent_lane_gap", status["reason"])
        for _ in range(12):
            self.node.merge_gap_callback(Message(data=json.dumps({"valid": True, "left": {"confirmed_available": True}})))
            output, status = self.tick(signal=None, line=None, x=95.)
            self.now += 0.05
        self.assertEqual(output.brake, 0.0)
        self.assertTrue(status["permission"])
        self.assertFalse(self.node.lane_change_clear("RIGHT", self.now))
        self.now += 0.6
        output, status = self.tick(signal=None, line=None, x=95.)
        self.assertEqual(output.brake, 1.0)

    def test_preview_mode_sends_no_lamp_packets(self):
        self.params["~lamp_output_enabled"] = False
        self.node = self.module.ManeuverFusionNode()
        self.socket.reset_mock()
        self.run_ticks(2)
        self.socket.sendto.assert_not_called()

    def test_camera_death_stops_even_without_active_maneuver(self):
        self.node.require_camera_stream = True
        output, status = self.run_ticks(2, line=None, signal=None, x=10.)
        self.assertEqual(output.brake, 1.0)
        self.assertIn("camera_observation_stream_stale", status["reason"])

    def test_unknown_images_are_fresh_but_cannot_release_red_stop(self):
        self.node.require_camera_stream = True
        for _ in range(3):
            self.sample("line", "base_link", distance_m=0., confidence=0., valid=False)
            output, status = self.tick(signal="UNKNOWN", line=None, x=10.)
            self.now += 0.05
        self.assertNotIn("camera_observation_stream_stale", status["reason"])
        self.assertEqual(output.brake, 0.0)

    def test_clock_reset_and_forward_teleport_do_not_reuse_permission(self):
        self.run_ticks(110)
        output, status = self.tick(x=100., line=None)
        self.assertEqual(output.brake, 1.0)
        self.assertFalse(status["permission"])
        self.now = 1.0
        output, status = self.tick()
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.0)

    def test_lamp_failure_and_obstacle_never_release_stop(self):
        self.socket.sendto.side_effect = OSError("test disconnect")
        output, status = self.run_ticks(130)
        self.assertFalse(status["indicator_ready"])
        self.assertEqual(output.brake, 1.0)
        self.socket.sendto.side_effect = None
        output, status = self.run_ticks(130, obstacle=True)
        self.assertEqual(output.brake, 1.0)
        self.assertIn("pedestrian", status["reason"])

    def test_stale_green_and_gps_blackout_block_departure(self):
        self.run_ticks(110)
        output, status = self.run_ticks(20, signal=None)
        self.assertEqual(output.brake, 1.0)
        output, status = self.run_ticks(2, quality="GPS_BLACKOUT")
        self.assertEqual(output.brake, 1.0)
        self.assertFalse(status["permission"])

    def test_committed_turn_survives_arrow_leaving_camera_but_not_obstacle(self):
        self.run_ticks(110)
        # Rear axle + 3m wheelbase must cross 90m; the bumper is not enough.
        self.tick(x=87.1, line=None)
        self.now += 0.05
        output, status = self.run_ticks(25, x=87.2, line=None, signal=None)
        self.assertTrue(status["event"]["committed"])
        self.assertEqual(output.brake, 0.0)
        output, _ = self.tick(x=87.2, line=None, signal=None, obstacle=True)
        self.assertEqual(output.brake, 1.0)

    def test_invalid_source_stamp_and_controller_death_fail_closed(self):
        self.run_ticks(110)
        self.now += 1.0
        output, status = self.tick(refresh=False)
        self.assertEqual(output.brake, 1.0)
        self.assertIn("unavailable", status["reason"])
        self.sample("signal", state="LEFT", confidence=1., valid=True, stamp=self.now - 10)
        self.assertLess(self.node.samples["signal"].stamp, self.now)

    def test_fusion_restart_after_safety_stop_limits_pedal_without_delaying_brake(self):
        self.run_ticks(120)
        output, _ = self.tick(obstacle=True)
        self.assertEqual((output.accel, output.brake), (0., 1.))
        self.now += .05
        output, status = self.tick(obstacle=False)
        self.assertEqual(output.brake, 0.)
        self.assertGreater(output.accel, 0.)
        self.assertLessEqual(output.accel, .025 + 1e-9)
        self.assertTrue(status["accel_rise_limited"])

    def test_delayed_line_route_position_uses_source_pose_during_braking(self):
        self.sample("odom", pose=NS(pose=NS(position=NS(x=80., y=0.))),
                    twist=NS(twist=NS(linear=NS(x=4., y=0.))))
        self.sample("line", "base_link", distance_m=10., valid=True, confidence=1.)
        line = self.node.samples["line"]
        self.now += .5
        self.sample("odom", pose=NS(pose=NS(position=NS(x=81.25, y=0.))),
                    twist=NS(twist=NS(linear=NS(x=1., y=0.))))
        self.node.update_route(self.node.samples["odom"], self.now, self.now)
        self.assertAlmostEqual(self.node.line_route_s(line), 90.)
        self.node.pose_history.popleft()
        self.assertIsNone(self.node.line_route_s(line))

    def test_final_launch_owns_one_lamp_writer_and_post_fallback_stopline(self):
        root = ET.parse(SOURCE / "bringup/morai_bringup/launch/perception_control_bringup.launch").getroot()
        fusion = root.find("node[@type='maneuver_fusion_node.py']")
        self.assertEqual(fusion.get("if"), "$(arg enable_maneuver_fusion)")
        self.assertIn("camera_fallback_cmd", fusion.find("param[@name='nominal_command_topic']").get("value"))
        self.assertEqual(fusion.find("param[@name='front_axle_offset_m']").get("value"),
                         "$(arg turn_entry_front_axle_offset_m)")
        final = ET.parse(SOURCE / "bringup/morai_bringup/launch/final_ws_bringup.launch").getroot()
        self.assertEqual(final.find("arg[@name='turn_entry_front_axle_offset_m']").get("default"), "3.0")
        self.assertEqual(final.find("include/arg[@name='turn_entry_front_axle_offset_m']").get("value"),
                         "$(arg turn_entry_front_axle_offset_m)")
        old = root.find("node[@type='stopline_controller.py']")
        self.assertIn("not arg('enable_maneuver_fusion')", old.find("param[@name='enabled']").get("value"))
        lamp = root.find("node[@type='turn_signal_node.py']")
        self.assertIn("not arg('enable_maneuver_fusion')", lamp.get("if"))
        for mux in root.findall(".//include[@file='$(find control_mux)/launch/control_mux.launch']"):
            self.assertIn("/control/maneuver_cmd", mux.find("arg[@name='nominal_command_topic']").get("value"))


class StrictFusionTest(unittest.TestCase):
    """Actual fusion callbacks with only ROS/UDP boundaries replaced."""
    def setUp(self):
        self.fixture = FusionNodeTest()
        self.fixture.setUp()
        self.node = self.fixture.node
        self.node.require_context = self.node.require_camera_stream = True
        self.node.camera = dict(calibrated=True, width=1280, height=720,
                                horizontal_fov_deg=90., x=0., y=0., z=1.,
                                yaw_deg=0., pitch_deg=0.)
        self.head = dict(id="own", x=110., y=0., z=5.)
        self.context = dict(id="junction", start=90., end=125., stop_s=90.,
                            direction="LEFT", source="mgeo", signal_points=[self.head])
        self.node.contexts = [self.context]
        self.node.map_heads = [self.head]

    def frame(self, state="LEFT", line=False, objects_stamp=None, extra=(), x=85.655,
              quality="NORMAL", no_objects=False, obstacle=False):
        from turn_signal_controller.signal_association import project_signal
        c = self.fixture
        c.sample("odom", pose=NS(pose=NS(position=NS(x=x, y=0., z=0.),
                     orientation=NS(x=0., y=0., z=0., w=1.))),
                 twist=NS(twist=NS(linear=NS(x=0., y=0.))))
        c.sample("quality", state=quality, imu_stale=False, gps_valid=quality == "NORMAL",
                 gps_blackout=quality == "GPS_BLACKOUT", gps_recovering=False)
        c.sample("safety", stop_required=obstacle, reason="pedestrian" if obstacle else "clear")
        c.sample("signal", state=state, confidence=.9, valid=state != "UNKNOWN")
        c.sample("line", "base_link", distance_m=4.345 if line else 0.,
                 confidence=1. if line else 0., valid=line)
        pixel = project_signal(self.head, dict(x=x, y=0., z=0., yaw=0.), self.node.camera)
        own = NS(class_name=state, conf=.9, x_center=pixel[0], y_center=pixel[1], width=30., height=12.)
        c.sample("objects", "front_camera", objects=[] if no_objects else [own] + list(extra), stamp=objects_stamp)
        result = c.tick(refresh=False)
        c.now = round(c.now + .05, 5)
        return result

    def test_missed_stopline_does_not_turn_left_route_into_straight(self):
        for _ in range(120):
            output, status = self.frame("GREEN")
        self.assertEqual(status["direction"], "LEFT")
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)
        self.assertAlmostEqual(status["front_bumper_distance_m"], .5)

    def test_matched_arrow_map_stopline_and_five_seconds_allow_turn(self):
        for _ in range(120):
            output, status = self.frame()
        self.assertTrue(status["permission"])
        self.assertEqual(status["selected_signal_id"], "own")
        self.assertEqual(output.brake, 0.)

    def test_adjacent_green_never_overrules_own_red(self):
        other = NS(class_name="GREEN", conf=.99, x_center=940., y_center=240., width=30., height=12.)
        for _ in range(120):
            output, status = self.frame("RED", extra=[other])
        self.assertEqual(status["selected_signal_state"], "RED")
        self.assertEqual(output.brake, 1.)

    def test_semantic_straight_is_not_reclassified_by_curved_path(self):
        self.context["direction"] = "STRAIGHT"
        for _ in range(20):
            output, status = self.frame("GREEN")
        self.assertEqual(status["direction"], "STRAIGHT")
        self.assertTrue(status["permission"])
        self.assertEqual(output.brake, 0.)

    def test_uncalibrated_camera_blocks_green(self):
        self.node.camera["calibrated"] = False
        for _ in range(120):
            output, status = self.frame()
        self.assertEqual(status["signal_selection_reason"], "signal_camera_uncalibrated")
        self.assertEqual(output.brake, 1.)

    def test_unverified_map_entry_requires_camera_line(self):
        self.context["stop_s"] = None
        for _ in range(120):
            output, status = self.frame()
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)
        for _ in range(12):
            output, status = self.frame(line=True)
        self.assertTrue(status["permission"])

    def test_unmapped_green_cannot_be_assumed_straight(self):
        self.node.contexts = []
        output, status = self.frame("GREEN")
        self.assertEqual(status["direction"], "UNKNOWN")
        self.assertIn("unmapped_signal_or_stopline", status["reason"])
        self.assertEqual(output.brake, 1.)

    def test_duplicate_image_does_not_count_as_stable_green(self):
        for _ in range(12):
            output, status = self.frame(objects_stamp=100.)
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)

    def test_red_revokes_previously_confirmed_arrow_immediately(self):
        for _ in range(120):
            self.frame()
        output, status = self.frame("RED")
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)

    def test_next_junction_is_discovered_without_one_tick_old_green(self):
        self.node.event = dict(self.context, end=85., committed=True, kind="turn")
        self.node.contexts = [dict(self.context, id="next", start=100., end=135.)]
        output, status = self.frame("RED")
        self.assertEqual(status["event"]["id"], "next")
        self.assertFalse(status["permission"])

    def test_localization_loss_stops_even_before_event_discovery(self):
        output, status = self.frame("UNKNOWN", quality="GPS_BLACKOUT", no_objects=True)
        self.assertIsNone(status["event"])
        self.assertIn("signal_localization_unreliable", status["reason"])
        self.assertEqual(output.brake, 1.)

    def test_earlier_lane_change_is_not_skipped_for_later_junction(self):
        self.node.manual = self.fixture.module.parse_maneuvers([
            dict(id="earlier", start_s_m=75., end_s_m=85., direction="LEFT", kind="lane_change")])
        output, status = self.frame("RED", x=65.)
        self.assertEqual(status["event"]["id"], "earlier")
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)

    def test_following_line_stops_bumper_before_current_rear_axle_exit(self):
        self.context.update(start=80., end=90., stop_s=80.)
        self.node.event = dict(self.context, committed=True, kind="turn")
        self.node.contexts.append(dict(self.context, id="next", start=91., stop_s=91., end=120.))
        for _ in range(120):
            output, _ = self.frame("LEFT")
        self.assertEqual(output.brake, 0.)
        output, status = self.frame("RED", x=86.655)
        self.assertEqual(status["next_junction_guard_id"], "next")
        self.assertEqual(output.brake, 1.)
        self.assertLess(status["progress_s_m"] + 3.845, 91.)

    def test_visible_known_junction_prearms_without_early_lamp(self):
        output, status = self.frame("LEFT", x=10.)
        self.assertEqual(status["event"]["id"], "junction")
        self.assertNotIn("unmapped_signal_or_stopline", status["reason"])
        self.assertFalse(status["indicator_ready"])
        self.assertEqual(self.fixture.node.lamp_pub.publish.call_args.args[0].data, "OFF")

    def test_wrong_frame_historical_pose_cannot_authorize_signal(self):
        for _ in range(120):
            self.frame()
        bad_stamp = self.fixture.now - .02
        value = NS(header=NS(frame_id="odom"), pose=self.node.pose_history[-1].value.pose)
        self.node.pose_history.append(NS(stamp=bad_stamp, value=value))
        output, status = self.frame(objects_stamp=bad_stamp)
        self.assertEqual(status["signal_selection_reason"], "signal_pose_unsynchronized")
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)

    def test_unsynchronized_stopline_pose_forces_safe_stop(self):
        self.frame("GREEN", line=True)
        self.node.pose_history.clear()
        output, status = self.fixture.tick(refresh=False)
        self.assertIn("stopline_pose_unsynchronized", status["reason"])
        self.assertEqual(output.brake, 1.)

    def test_distant_context_cannot_hide_nearer_stopline(self):
        self.context.update(start=200., end=240., stop_s=200.)
        output, status = self.frame("RED", line=True)
        self.assertIn("unmapped_stopline_before_context", status["reason"])
        self.assertEqual(output.brake, 1.)

    def test_unmatched_visible_light_cannot_be_overridden_by_distant_map(self):
        self.context.update(start=200., end=240., stop_s=200.,
                            signal_points=[dict(self.head, y=20.)])
        output, status = self.frame("RED")
        self.assertIn("unassociated_visible_signal", status["reason"])
        self.assertEqual(output.brake, 1.)

    def test_next_guard_exists_before_commit_and_includes_manual_change(self):
        self.node.manual = self.fixture.module.parse_maneuvers([
            dict(id="next_change", start_s_m=126., end_s_m=140., direction="LEFT", kind="lane_change")])
        output, status = self.frame("RED")
        self.assertFalse(status["event"]["committed"])
        self.assertEqual(status["next_junction_guard_id"], "next_change")

    def authorize(self):
        for _ in range(120):
            output, status = self.frame()
        self.assertTrue(status["permission"])

    def test_bumper_crossing_does_not_commit_before_front_axle(self):
        self.authorize()
        _, status = self.frame(x=86.3)
        self.assertFalse(status["event"]["committed"])
        output, status = self.frame("RED", x=86.4)
        self.assertFalse(status["event"]["committed"])
        self.assertEqual(output.brake, 1.)

    def test_red_at_front_axle_crossing_cannot_reuse_previous_arrow(self):
        self.authorize()
        output, status = self.frame("RED", x=87.1)
        self.assertFalse(status["event"]["committed"])
        self.assertIn("entry_without_current_permission", status["reason"])
        self.assertEqual(output.brake, 1.)
        for _ in range(120):
            output, status = self.frame(x=87.1)
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)

    def test_expired_entry_ticket_is_not_reused_after_control_gap(self):
        self.context["direction"] = "STRAIGHT"
        for _ in range(30):
            self.frame("GREEN")
        self.fixture.now += 1.
        output, status = self.frame("GREEN", x=87.1)
        self.assertFalse(status["event"]["committed"])
        self.assertEqual(output.brake, 1.)

    def test_mid_junction_start_cannot_create_permission_retroactively(self):
        for _ in range(120):
            output, status = self.frame(x=88.)
        self.assertFalse(status["event"]["committed"])
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)

    def test_lamp_failure_at_crossing_does_not_commit(self):
        self.authorize()
        self.fixture.socket.sendto.side_effect = OSError("disconnect")
        output, status = self.frame(x=87.1)
        self.assertFalse(status["event"]["committed"])
        self.assertEqual(output.brake, 1.)

    def test_valid_crossing_retains_lamp_during_localization_loss(self):
        self.authorize()
        _, status = self.frame(x=87.1)
        self.assertTrue(status["event"]["committed"])
        output, status = self.frame("UNKNOWN", x=87.1, quality="GPS_BLACKOUT", no_objects=True)
        self.assertEqual(output.brake, 1.)
        self.assertEqual(self.node.lamp_pub.publish.call_args.args[0].data, "LEFT")

    def test_stop_guard_adjustment_does_not_move_verified_axle_crossing(self):
        self.authorize()
        self.node.event["start"] = 89.
        _, status = self.frame(x=86.4)
        self.assertFalse(status["event"]["committed"])

    def test_front_axle_offset_is_validated(self):
        for offset in (-1., float("nan"), 4.):
            self.fixture.params["~front_axle_offset_m"] = offset
            with self.assertRaises(ValueError):
                self.fixture.module.ManeuverFusionNode()

    def test_obstacle_at_crossing_cannot_leave_latched_entry(self):
        self.authorize()
        output, status = self.frame(x=87.1, obstacle=True)
        self.assertFalse(status["event"]["committed"])
        self.assertEqual(output.brake, 1.)
        output, status = self.frame("RED", x=87.1)
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)

    def test_short_lamp_send_is_not_counted_as_success(self):
        self.fixture.socket.sendto.return_value = 10
        for _ in range(120):
            output, status = self.frame()
        self.assertEqual(status["lamp_requested"], "LEFT")
        self.assertFalse(status["lamp_transmit_ok"])
        self.assertFalse(status["indicator_ready"])
        self.assertEqual(output.brake, 1.)

    def test_right_turn_policy_false_requires_its_own_arrow(self):
        self.context["direction"] = "RIGHT"
        self.node.right_on_green = False
        for _ in range(120):
            output, status = self.frame("GREEN")
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)
        for _ in range(20):
            output, status = self.frame("RIGHT")
        self.assertTrue(status["permission"])
        self.assertEqual(self.node.lamp_pub.publish.call_args.args[0].data, "RIGHT")
        output, status = self.frame("RED", x=87.1)
        self.assertFalse(status["event"]["committed"])
        self.assertEqual(output.brake, 1.)

    def test_lamp_stays_on_until_verified_exit_then_turns_off(self):
        self.context["end"] = 90.
        self.authorize()
        for x in (87.1, 88., 89., 89.99):
            _, status = self.frame(x=x)
            self.assertTrue(status["event"]["committed"])
            self.assertEqual(self.node.lamp_pub.publish.call_args.args[0].data, "LEFT")
        _, status = self.frame("UNKNOWN", x=90., no_objects=True)
        self.assertIsNone(status["event"])
        self.assertEqual(self.node.lamp_pub.publish.call_args.args[0].data, "OFF")

    def test_adjacent_turn_restarts_lead_even_for_same_direction(self):
        for following_direction in ("LEFT", "RIGHT"):
            with self.subTest(direction=following_direction):
                self.setUp()
                self.context["end"] = 90.
                self.node.contexts.append(dict(self.context, id="next", start=100.,
                                               stop_s=100., end=125., direction=following_direction))
                self.authorize()
                for x in (87.1, 88., 89.):
                    self.frame(x=x)
                _, status = self.frame(following_direction, x=90.)
                self.assertEqual(status["event"]["id"], "next")
                self.assertFalse(status["indicator_ready"])
                self.assertFalse(status["permission"])
                self.assertEqual(self.node.lamp_pub.publish.call_args.args[0].data, following_direction)

    def corridor_frame(self, quality="NORMAL", mode="normal_nominal", x=10., **kwargs):
        self.node.allow_blackout_lane = True
        self.node.fallback_status_callback(Message(data=json.dumps(dict(
            stamp=self.fixture.now, mode=mode, camera_used=mode != "normal_nominal",
            lane_primary_usable=True, nominal_fresh=True, blackout_budget_exhausted=False,
            accel=0. if mode == "lane_loss_braking" else .5,
            brake=.2 if mode == "lane_loss_braking" else 0.))))
        return self.frame("UNKNOWN", no_objects=True, quality=quality, x=x, **kwargs)

    def test_bounded_blackout_corridor_reaches_final_command(self):
        self.corridor_frame()
        output, status = self.corridor_frame("GPS_BLACKOUT", "gps_blackout_camera_fallback", x=10.1)
        self.assertTrue(status["blackout_lane_corridor"])
        self.assertEqual(output.brake, 0.)
        self.assertGreater(output.accel, 0.)
        self.assertFalse(status["permission"])  # Not a junction entry permit.

    def test_blackout_corridor_needs_normal_anchor_and_fresh_status(self):
        output, status = self.corridor_frame("GPS_BLACKOUT", "gps_blackout_camera_fallback")
        self.assertFalse(status["blackout_lane_corridor"])
        self.assertEqual(output.brake, 1.)
        self.corridor_frame()
        self.node.samples.pop("fallback")
        output, status = self.frame("UNKNOWN", no_objects=True, quality="GPS_BLACKOUT", x=10.1)
        self.assertEqual(output.brake, 1.)
        self.assertIn("camera_fallback_status_stale", status["reason"])

    def test_blackout_corridor_cannot_cross_junction_or_visible_line(self):
        self.corridor_frame(x=49.)
        output, status = self.corridor_frame("GPS_BLACKOUT", "gps_blackout_camera_fallback", x=50.1)
        self.assertFalse(status["blackout_lane_corridor"])
        self.assertEqual(output.brake, 1.)
        self.setUp()
        self.corridor_frame()
        output, status = self.corridor_frame("GPS_BLACKOUT", "gps_blackout_camera_fallback", line=True)
        self.assertFalse(status["blackout_lane_corridor"])
        self.assertEqual(output.brake, 1.)

    def test_blackout_corridor_expiration_and_obstacle_override(self):
        self.corridor_frame()
        output, status = self.corridor_frame("GPS_BLACKOUT", "gps_blackout_camera_fallback", obstacle=True)
        self.assertEqual(output.brake, 1.)
        self.assertIn("pedestrian", status["reason"])
        self.node.blackout_max_duration = .1
        for _ in range(5):
            output, status = self.corridor_frame("GPS_BLACKOUT", "gps_blackout_camera_fallback")
        self.assertEqual(output.brake, 1.)
        self.assertIn("budget", status["blackout_corridor_reason"])

    def test_recovery_status_does_not_restore_route_permission_early(self):
        self.corridor_frame()
        self.corridor_frame("GPS_BLACKOUT", "gps_blackout_camera_fallback")
        output, status = self.corridor_frame("NORMAL", "recovery_camera")
        self.assertTrue(status["blackout_lane_corridor"])
        self.assertIsNone(status["event"])
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 0.)

    def test_corrupt_fallback_status_revokes_previous_valid_state(self):
        self.corridor_frame()
        self.node.fallback_status_callback(Message(data="not JSON"))
        output, status = self.frame("UNKNOWN", no_objects=True, x=10.)
        self.assertEqual(output.brake, 1.)
        self.assertIn("camera_fallback_status_stale", status["reason"])

    def test_lane_loss_braking_status_limits_older_nominal_accel(self):
        self.corridor_frame()
        output, status = self.corridor_frame("GPS_BLACKOUT", "lane_loss_braking")
        self.assertTrue(status["blackout_lane_corridor"])
        self.assertEqual(output.accel, 0.)
        self.assertGreaterEqual(output.brake, .2)

    def test_final_launch_wires_matching_blackout_budgets(self):
        root = ET.parse(SOURCE / "bringup/morai_bringup/launch/perception_control_bringup.launch").getroot()
        fusion = root.find("node[@type='maneuver_fusion_node.py']")
        fallback = root.find("node[@type='camera_localization_fallback_controller.py']")
        self.assertEqual(fusion.find("param[@name='allow_blackout_lane_corridor']").get("value"), "$(arg enable_lane_fallback)")
        for key in ("blackout_max_duration_sec", "blackout_max_distance_m"):
            self.assertEqual(fusion.find("param[@name='%s']" % key).get("value"),
                             fallback.find("param[@name='%s']" % key).get("value"))


class BumperStopTest(unittest.TestCase):
    def test_closed_loop_bumper_target_with_blind_camera_and_delay(self):
        # Ideal longitudinal plant, NOT a MORAI calibration claim.
        core = StopLineControllerCore(front_reference_offset_m=3.845, hold_distance_m=0.5)
        speed, bumper_distance, dt = 2.0, 14.0, 0.05
        for i in range(600):
            t = 100 + i * dt
            core.observe_signal("RED", 1., True, t, t, t)
            if bumper_distance > 2.5:
                core.observe_line(bumper_distance + 3.845 + speed * 0.1, 1., True, t - 0.1, t, t)
            decision = core.update(t, t, speed)
            acceleration = min(0.7 if speed < 2 else 0, decision.accel_limit)
            if decision.brake:
                acceleration = -1.5 * decision.brake
            following = max(0., speed + acceleration * dt)
            bumper_distance -= 0.5 * (speed + following) * dt
            speed = following
            if decision.mode == "HOLD" and speed <= 0.01:
                break
        self.assertEqual(decision.mode, "HOLD")
        self.assertAlmostEqual(bumper_distance, 0.5, delta=0.1)


if __name__ == "__main__":
    unittest.main()
