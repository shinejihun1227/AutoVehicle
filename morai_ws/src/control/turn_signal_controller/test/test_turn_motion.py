"""Turn limits and actual Pure Pursuit + fusion callbacks in a bicycle plant.

ROS transport, camera inference and MORAI dynamics are replaced; route
projection, steering computation, signal association and command overlay run.
"""

import importlib.util
import math
import sys
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import Mock, patch

import test_maneuver_fusion as fixtures
from curvature_speed_purepursuit.planner import (
    cumulative_arc_lengths, interpolate_by_s, nearest_projection,
)
from turn_signal_controller.fusion import signal_permits
from turn_signal_controller.signal_association import project_signal
from turn_signal_controller.turn_motion import TurnMotionPlanner, quaternion_yaw
from stopline_control.core import Decision


class TurnMotionTest(unittest.TestCase):
    def planner(self, sign=1, **config):
        points = fixtures.turn_path(sign)
        self.s = cumulative_arc_lengths(points)
        return TurnMotionPlanner(points, self.s, **dict({"max_speed_kph": 40.}, **config))

    def event(self, direction="LEFT", **values):
        return dict(id="turn", kind="turn", direction=direction, start=90., end=125.,
                    committed=values.get("committed", False))

    def test_directional_signal_permission_matrix(self):
        left = {"LEFT", "RED_LEFT", "GREEN_LEFT"}
        right = {"RIGHT", "RED_RIGHT", "GREEN_RIGHT"}
        states = left | right | {"GREEN", "RED", "YELLOW", "UNKNOWN", "GREEN_ARROW"}
        for state in states:
            self.assertEqual(signal_permits(state, "LEFT"), state in left, state)
            self.assertEqual(signal_permits(state, "RIGHT", False), state in right, state)
            self.assertEqual(signal_permits(state, "RIGHT", True),
                             state in right | {"GREEN", "GREEN_LEFT"}, state)

    def test_mirrored_turns_have_same_curvature_speed_and_braking(self):
        results = []
        for sign, direction in ((1, "LEFT"), (-1, "RIGHT")):
            planner = self.planner(sign)
            p, _ = interpolate_by_s(planner.points, self.s, 110.)
            result = planner.evaluate(self.event(direction, committed=True), 110., 6.,
                                      p.x, p.y, planner.heading(110.), 87.)
            self.assertAlmostEqual(result.speed_limit_kph, 3.6 * math.sqrt(15.), delta=.15)
            self.assertGreater(result.brake, 0.)
            self.assertEqual(result.accel_limit, 0.)
            results.append(result)
        self.assertAlmostEqual(results[0].speed_limit_kph, results[1].speed_limit_kph)
        self.assertAlmostEqual(results[0].brake, results[1].brake)
        planner = self.planner(lateral_accel_limit_mps2=.4)
        result = planner.evaluate(self.event(), 87., 0., 87., 0., 0., 87.)
        self.assertLess(result.curve_speed_kph, 10.)

    def test_radius_controls_speed_without_directional_plateaus(self):
        limits = []
        for radius in (5., 15., 40.):
            points = [fixtures.PathPoint(float(x), 0.) for x in range(101)]
            # Dense circle samples keep the analytic-radius oracle distinct
            # from corners introduced by a coarse polygon approximation.
            points += [fixtures.PathPoint(100. + radius * math.sin(math.radians(i / 10.)),
                                          radius * (1. - math.cos(math.radians(i / 10.)))) for i in range(1, 901)]
            points += [fixtures.PathPoint(100. + radius, radius + i) for i in range(1, 21)]
            planner = TurnMotionPlanner(points, cumulative_arc_lengths(points), max_speed_kph=80.)
            event = dict(self.event(), end=100. + radius * math.pi / 2.)
            motion = planner.evaluate(event, 87., 0., 87., 0., 0., 87.)
            self.assertAlmostEqual(motion.curve_speed_kph, 3.6 * math.sqrt(radius), delta=.15)
            self.assertAlmostEqual(motion.speed_limit_kph, motion.curve_speed_kph)
            limits.append(motion.speed_limit_kph)
        self.assertTrue(limits[0] < limits[1] < limits[2])
        self.assertGreater(limits[2], 15.)

    def test_common_maximum_limits_both_turn_and_approach_but_not_raw_curvature_diagnostic(self):
        planner = self.planner(max_speed_kph=7.2)
        for progress in (50., 87.):
            motion = planner.evaluate(self.event(), progress, 3., progress, 0., 0., 87.)
            self.assertEqual(motion.speed_limit_kph, 7.2)
            self.assertGreater(motion.curve_speed_kph, 7.2)
            self.assertGreater(motion.brake, 0.)

    def test_straight_geometry_has_no_curvature_limit_but_obeys_overall_maximum(self):
        points = [fixtures.PathPoint(float(x), 0.) for x in range(141)]
        planner = TurnMotionPlanner(points, cumulative_arc_lengths(points), max_speed_kph=20.)
        for direction in ("LEFT", "RIGHT"):
            motion = planner.evaluate(self.event(direction), 87., 0., 87., 0., 0., 87.)
            self.assertIsNone(motion.curve_speed_kph)
            self.assertEqual(motion.curvature_abs_m_inv, 0.)
            self.assertEqual(motion.speed_limit_kph, 20.)

    def test_zero_overall_maximum_commands_deceleration(self):
        planner = self.planner(max_speed_kph=0.)
        motion = planner.evaluate(self.event(), 70., 1., 70., 0., 0., 87.)
        self.assertEqual(motion.speed_limit_kph, 0.)
        self.assertEqual(motion.accel_limit, 0.)
        self.assertGreater(motion.brake, 0.)

    def test_green_turn_is_prebraked_before_entry(self):
        planner = self.planner()
        result = planner.evaluate(self.event(), 65., 10., 65., 0., 0., 87.)
        self.assertGreater(result.brake, 0.)
        self.assertGreater(result.speed_limit_kph, result.curve_speed_kph)
        limited = result.constrain(Decision("NOMINAL", "green"))
        self.assertEqual(limited.mode, "NOMINAL")  # Keep signal permission semantics.
        self.assertEqual(limited.accel_limit, 0.)
        self.assertEqual(limited.reason, "turn_speed_envelope")

    def test_limit_does_not_relax_other_braking(self):
        planner = self.planner()
        motion = planner.evaluate(self.event(), 85., 0., 85., 0., 0., 87.)
        decision = motion.constrain(Decision("HOLD", "obstacle", 0., 1., 0.))
        self.assertEqual((decision.mode, decision.accel_limit, decision.brake,
                          decision.target_speed_kph), ("HOLD", 0., 1., 0.))

    def test_wrong_heading_and_lateral_drift_stop(self):
        planner = self.planner()
        for y, yaw, fault in ((0., math.pi, "turn_route_heading_mismatch"),
                              (2., 0., "turn_route_lateral_error")):
            motion = planner.evaluate(self.event(), 85., 2., 85., y, yaw, 87.)
            self.assertEqual(motion.fault, fault)
            self.assertEqual((motion.accel_limit, motion.brake), (0., 1.))

    def test_exit_requires_position_and_alignment_not_progress_alone(self):
        planner = self.planner()
        p, _ = interpolate_by_s(planner.points, self.s, 125.)
        end_heading = planner.heading(125.)
        motion = planner.evaluate(self.event(committed=True), 125., 2., p.x, p.y,
                                  end_heading - math.radians(25), 87.)
        self.assertFalse(motion.exit_ready)
        self.assertEqual(motion.phase, "EXIT_ALIGNMENT")
        aligned = planner.evaluate(self.event(committed=True), 125., 2., p.x, p.y, end_heading, 87.)
        self.assertTrue(aligned.exit_ready)
        off_lane = planner.evaluate(self.event(committed=True), 125., 2., p.x + 1., p.y, end_heading, 87.)
        self.assertFalse(off_lane.exit_ready)

    def test_unaligned_exit_has_bounded_overrun(self):
        planner = self.planner()
        p, _ = interpolate_by_s(planner.points, self.s, 129.)
        motion = planner.evaluate(self.event(committed=True), 129., 2., p.x, p.y,
                                  planner.heading(125.) - math.radians(25), 87.)
        self.assertEqual(motion.fault, "turn_exit_alignment_unconfirmed")
        self.assertEqual(motion.brake, 1.)

    def test_invalid_pose_and_configuration_are_rejected(self):
        for q in (None, NS(x=0., y=0., z=0., w=0.), NS(x=0., y=0., z=float("nan"), w=1.)):
            self.assertIsNone(quaternion_yaw(q))
        self.assertAlmostEqual(quaternion_yaw(NS(x=0., y=0., z=math.sin(.4), w=math.cos(.4))), .8)
        for config in ({"max_speed_kph": -1.}, {"max_speed_kph": float("nan")},
                       {"lateral_accel_limit_mps2": 0.}, {"lateral_accel_limit_mps2": float("nan")},
                       {"exit_heading_error_deg": 90.}, {"exit_lateral_error_m": 2.}):
            with self.assertRaises(ValueError):
                self.planner(**config)


class TurnNodeMotionTest(unittest.TestCase):
    def setUp(self):
        self.case = fixtures.StrictFusionTest()
        self.case.setUp()
        self.c, self.node = self.case.fixture, self.case.node

    def test_old_directional_parameters_cannot_override_shared_curvature_tuning(self):
        self.c.params.update({"~turn_left_speed_kph": 1., "~turn_right_speed_kph": 2.,
                              "~turn_lateral_accel_mps2": 9.,
                              "~max_speed_kph": 30., "~lateral_accel_limit_mps2": .8})
        node = self.c.module.ManeuverFusionNode()
        self.assertEqual(node.turn_motion.max_speed_kph, 30.)
        self.assertEqual(node.turn_motion.lateral_accel_limit_mps2, .8)
        self.assertEqual(self.c.ros.logwarn.call_count, 3)
        limits = [node.turn_motion.evaluate(dict(self.case.context, kind="turn", direction=direction),
                                            87., 0., 87., 0., 0., 87.).curve_speed_kph
                  for direction in ("LEFT", "RIGHT")]
        self.assertAlmostEqual(limits[0], limits[1])
        self.assertAlmostEqual(limits[0], 3.6 * math.sqrt(.8 * 15.), delta=.15)

    def test_launch_shares_speed_parameters_between_nominal_and_fusion(self):
        root = fixtures.SOURCE / "bringup/morai_bringup/launch"
        final = ET.parse(root / "final_ws_bringup.launch").getroot()
        perception = ET.parse(root / "perception_control_bringup.launch").getroot()
        nominal = ET.parse(root / "morai_udp_ekf_purepursuit.launch").getroot()
        fusion = perception.find("node[@name='final_ws_maneuver_fusion']")
        controller = nominal.find(".//node[@name='curvature_speed_purepursuit']")
        include = perception.find("include[@file='$(find morai_bringup)/launch/morai_udp_ekf_purepursuit.launch']")
        for key in ("max_speed_kph", "lateral_accel_limit_mps2"):
            expected = "$(arg " + key + ")"
            self.assertEqual(final.find("include/arg[@name='" + key + "']").get("value"), expected)
            self.assertEqual(include.find("arg[@name='" + key + "']").get("value"), expected)
            self.assertEqual(fusion.find("param[@name='" + key + "']").get("value"), expected)
            self.assertEqual(controller.find("param[@name='" + key + "']").get("value"), expected)

    def test_heading_fault_cannot_use_existing_arrow_permission(self):
        self.case.authorize()
        self.c.sample("odom", pose=NS(pose=NS(position=NS(x=85.655, y=0., z=0.),
            orientation=NS(x=0., y=0., z=1., w=0.))),
            twist=NS(twist=NS(linear=NS(x=0., y=0.))))
        output, status = self.c.tick(refresh=False)
        self.assertIn("turn_route_heading_mismatch", status["reason"])
        self.assertFalse(status["permission"])
        self.assertEqual((output.accel, output.brake), (0., 1.))
        self.c.now += .05
        output, status = self.case.frame()
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)
        for _ in range(25):
            output, status = self.case.frame()
        self.assertTrue(status["permission"])

    def test_unaligned_completed_distance_keeps_event_and_indicator(self):
        self.case.frame(no_objects=True, state="UNKNOWN")
        self.c.sample("odom", pose=NS(pose=NS(position=NS(x=85.8, y=0., z=0.),
            orientation=NS(x=0., y=0., z=math.sin(.2), w=math.cos(.2)))),
            twist=NS(twist=NS(linear=NS(x=0., y=0.))))
        # A crossing was already authorized; distance alone must not end it.
        self.node.event = dict(self.case.context, start=80., stop_s=80., end=85.,
                               kind="turn", committed=True)
        output, status = self.c.tick(refresh=False)
        self.assertIsNotNone(status["event"])
        self.assertEqual(status["turn_phase"], "EXIT_ALIGNMENT")
        self.assertEqual(status["lamp_requested"], "LEFT")
        self.c.now += .05
        self.c.sample("odom", pose=NS(pose=NS(position=NS(x=85.8, y=0., z=0.),
            orientation=NS(x=0., y=0., z=0., w=1.))),
            twist=NS(twist=NS(linear=NS(x=0., y=0.))))
        _, status = self.c.tick(refresh=False)
        self.assertIsNone(status["event"])
        self.assertEqual(status["completed_event_id"], "junction")
        self.assertEqual(status["lamp_requested"], "OFF")

    def load_actual_steering(self, points, s_values):
        source = fixtures.SOURCE
        sys.path.insert(0, str(source / "control/purepursuit_mgeo/src"))
        modules = {"rospy": Mock(), "geometry_msgs.msg": NS(PointStamped=fixtures.Message, PoseStamped=fixtures.Message),
                   "morai_msgs.msg": NS(CtrlCmd=fixtures.Command),
                   "nav_msgs.msg": NS(Odometry=fixtures.Message, Path=fixtures.Message),
                   "std_msgs.msg": NS(Bool=fixtures.Message, Float64=fixtures.Message)}
        path = source / "experimental/curvature_speed_purepursuit/scripts/curvature_speed_purepursuit_node.py"
        spec = importlib.util.spec_from_file_location("_turn_actual_pp", path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
        controller = module.CurvatureSpeedPurePursuitNode.__new__(module.CurvatureSpeedPurePursuitNode)
        controller.points, controller.s_values = points, s_values
        controller.total_length_m = s_values[-1]
        controller.lookahead_min_m, controller.lookahead_gain = 4., .35
        controller.wheelbase_m, controller.steering_sign = 3., 1.
        controller.max_steering_rad = math.radians(40.)
        return controller

    def drive(self, sign, direction):
        self.node.points = fixtures.turn_path(sign)
        self.node.s_values = cumulative_arc_lengths(self.node.points)
        self.node.turn_motion = TurnMotionPlanner(self.node.points, self.node.s_values, max_speed_kph=40.)
        self.case.context["direction"] = direction
        pp = self.load_actual_steering(self.node.points, self.node.s_values)
        x, y, yaw, speed, dt = 85.655, 0., 0., 0., .05
        committed = False
        worst_lateral, peak_turn_speed, apex_steering = 0., 0., []
        peak_apex_speed = 0.
        saw_hold = saw_permission = False
        for _ in range(1800):
            projection = nearest_projection(pp.points, pp.s_values, x, y)
            steering, _, _ = pp.compute_steering(x, y, yaw, speed, projection.progress_s)
            self.c.sample("odom", pose=NS(pose=NS(position=NS(x=x, y=y, z=0.),
                orientation=NS(x=0., y=0., z=math.sin(yaw / 2.), w=math.cos(yaw / 2.)))),
                twist=NS(twist=NS(linear=NS(x=speed, y=0.))))
            self.c.sample("quality", state="NORMAL", imu_stale=False, gps_valid=True,
                          gps_blackout=False, gps_recovering=False)
            self.c.sample("safety", stop_required=False, reason="clear")
            # The signal disappears after entry; the observation stream stays alive.
            self.c.sample("signal", state="UNKNOWN" if committed else direction,
                          confidence=.9, valid=not committed)
            self.c.sample("line", "base_link", distance_m=0., confidence=0., valid=False)
            pixel = project_signal(self.case.head, dict(x=x, y=y, z=0., yaw=yaw), self.node.camera)
            objects = [] if committed or pixel is None else [NS(class_name=direction, conf=.9,
                x_center=pixel[0], y_center=pixel[1], width=30., height=12.)]
            self.c.sample("objects", "front_camera", objects=objects)
            self.node.command_callback(fixtures.Command(steering=steering, accel=.6))
            self.node.tick(None)
            output = self.node.output_pub.publish.call_args.args[0]
            status = fixtures.json.loads(self.node.state_pub.publish.call_args.args[0].data)
            self.assertAlmostEqual(output.steering, steering)
            saw_hold |= status["mode"] == "HOLD"
            saw_permission |= status["permission"]
            committed |= bool(status["event"] and status["event"]["committed"])
            self.assertFalse(status["entry_fault"], status["reason"])
            if committed:
                peak_turn_speed = max(peak_turn_speed, speed)
                worst_lateral = max(worst_lateral, projection.distance_m)
                if 106. < projection.progress_s < 117.:
                    apex_steering.append(output.steering)
                    peak_apex_speed = max(peak_apex_speed, speed)
            if status["completed_event_id"]:
                break
            next_speed = max(0., speed + (output.accel - 1.5 * output.brake) * dt)
            mean_speed = .5 * (speed + next_speed)
            dyaw = mean_speed / 3. * math.tan(output.steering) * dt
            x += mean_speed * math.cos(yaw + dyaw / 2.) * dt
            y += mean_speed * math.sin(yaw + dyaw / 2.) * dt
            yaw += dyaw
            speed = next_speed
            self.c.now = round(self.c.now + dt, 6)
        self.assertTrue(saw_hold and saw_permission and committed)
        self.assertEqual(status["completed_event_id"], "junction", status)
        self.assertLess(worst_lateral, .75)
        self.assertLess(abs(math.degrees(yaw) - sign * 90.), 15.)
        self.assertTrue(apex_steering)
        self.assertTrue(all(value * sign > 0 for value in apex_steering))
        self.assertLessEqual(peak_apex_speed * 3.6, 3.6 * math.sqrt(15.) + .5)
        self.assertLessEqual(peak_turn_speed * 3.6, self.node.turn_motion.max_speed_kph)
        return dict(direction=direction, exit_yaw_deg=math.degrees(yaw),
                    max_lateral_error_m=worst_lateral, max_turn_speed_kph=peak_turn_speed * 3.6,
                    elapsed_sec=self.c.now - 100., signal_lost_after_entry=True)

    def test_actual_pure_pursuit_and_signal_fusion_complete_left_turn(self):
        self.drive(1, "LEFT")

    def test_actual_pure_pursuit_and_signal_fusion_complete_right_turn(self):
        self.drive(-1, "RIGHT")


if __name__ == "__main__":
    unittest.main()
