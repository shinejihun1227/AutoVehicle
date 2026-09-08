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
                   "nav_msgs.msg": NS(Odometry=Message), "std_msgs.msg": NS(String=Message)}
        spec = importlib.util.spec_from_file_location("_fusion_test_node", PACKAGE / "scripts/maneuver_fusion_node.py")
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(self.module)
        self.module.time = NS(monotonic=lambda: self.now)
        self.socket = Mock()
        self.module.socket = NS(socket=lambda *_: self.socket, AF_INET=2, SOCK_DGRAM=2)
        self.module.load_path_file = lambda _: turn_path()
        self.node = self.module.ManeuverFusionNode()

    def sample(self, key, frame="map", stamp=None, **kwargs):
        message = Message(header=NS(stamp=Stamp(self.now if stamp is None else stamp), frame_id=frame), **kwargs)
        self.node.observe(message, key)

    def tick(self, signal="LEFT", line=4.345, obstacle=False, quality="NORMAL",
             x=85.655, speed=0.0, refresh=True):
        if refresh:
            self.sample("odom", pose=NS(pose=NS(position=NS(x=x, y=0.))),
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
        self.tick(x=86.2, line=None)
        self.now += 0.05
        output, status = self.run_ticks(25, x=87.0, line=None, signal=None)
        self.assertTrue(status["event"]["committed"])
        self.assertEqual(output.brake, 0.0)
        output, _ = self.tick(x=87.0, line=None, signal=None, obstacle=True)
        self.assertEqual(output.brake, 1.0)

    def test_invalid_source_stamp_and_controller_death_fail_closed(self):
        self.run_ticks(110)
        self.now += 1.0
        output, status = self.tick(refresh=False)
        self.assertEqual(output.brake, 1.0)
        self.assertIn("unavailable", status["reason"])
        self.sample("signal", state="LEFT", confidence=1., valid=True, stamp=self.now - 10)
        self.assertLess(self.node.samples["signal"].stamp, self.now)

    def test_final_launch_owns_one_lamp_writer_and_post_fallback_stopline(self):
        root = ET.parse(SOURCE / "bringup/morai_bringup/launch/perception_control_bringup.launch").getroot()
        fusion = root.find("node[@type='maneuver_fusion_node.py']")
        self.assertEqual(fusion.get("if"), "$(arg enable_maneuver_fusion)")
        self.assertIn("camera_fallback_cmd", fusion.find("param[@name='nominal_command_topic']").get("value"))
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
              quality="NORMAL", no_objects=False):
        from turn_signal_controller.signal_association import project_signal
        c = self.fixture
        c.sample("odom", pose=NS(pose=NS(position=NS(x=x, y=0., z=0.),
                     orientation=NS(x=0., y=0., z=0., w=1.))),
                 twist=NS(twist=NS(linear=NS(x=0., y=0.))))
        c.sample("quality", state=quality)
        c.sample("safety", stop_required=False, reason="clear")
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
