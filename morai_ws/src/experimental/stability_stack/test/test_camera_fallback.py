"""Real callbacks with ROS replaced; these are not vehicle dynamics tests."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


class Stamp:
    def __init__(self, seconds=0.):
        self.seconds = seconds
    def to_sec(self):
        return self.seconds


class Message:
    def __init__(self, **kwargs):
        self.header = NS(stamp=Stamp(), frame_id="base_link")
        self.__dict__.update(kwargs)


class Command:
    def __init__(self, **kwargs):
        self.longlCmdType = 1
        self.accel, self.brake, self.steering = .5, 0., .4
        self.velocity = self.acceleration = 0.
        self.__dict__.update(kwargs)


class FallbackTest(unittest.TestCase):
    def setUp(self):
        self.now = self.ros_now = 100.
        self.params = {}
        self.ros = Mock()
        self.ros.get_time.side_effect = lambda: self.ros_now
        self.ros.Time.now.side_effect = lambda: Stamp(self.ros_now)
        self.ros.get_param.side_effect = lambda k, default=None: self.params.get(k, default)
        self.ros.Publisher.side_effect = lambda *_a, **_k: Mock()
        self.modules = {"rospy": self.ros, "morai_msgs.msg": NS(CtrlCmd=Command),
                        "morai_perception_msgs.msg": NS(LaneDetection=Message, SensorQuality=Message, GpsHealth=Message),
                        "nav_msgs.msg": NS(Odometry=Message), "sensor_msgs.msg": NS(Imu=Message),
                        "std_msgs.msg": NS(Bool=Message, String=Message)}
        self.module = self.load("camera_localization_fallback_controller")
        self.node = self.module.CameraLocalizationFallbackController()

    def load(self, name):
        spec = importlib.util.spec_from_file_location("_test_" + name, SCRIPTS / (name + ".py"))
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, self.modules):
            spec.loader.exec_module(module)
        module.time = NS(monotonic=lambda: self.now)
        return module

    def message(self, frame="base_link", stamp=None, **kwargs):
        return Message(header=NS(stamp=Stamp(self.ros_now if stamp is None else stamp), frame_id=frame), **kwargs)

    def advance(self, dt=.05):
        self.now = round(self.now + dt, 6)
        self.ros_now = round(self.ros_now + dt, 6)

    def lane(self, confidence=.95, lateral=.2, heading=.05, valid=True, stamp=None):
        self.node.lane_callback(self.message("front_camera", stamp, valid=valid,
                                            confidence=confidence, lateral_offset_m=lateral, heading_error_rad=heading))

    def frame(self, state="NORMAL", imu_stale=False, confidence=.95, steering=.4,
              odom=True, lane=True, intersection=False, speed=1.):
        self.node.nominal_callback(Command(steering=steering))
        self.node.quality_callback(self.message(state=state, reason="test", gps_valid=state == "NORMAL",
                                                gps_blackout=state == "GPS_BLACKOUT", gps_recovering=False,
                                                imu_stale=imu_stale, confidence=1. if state == "NORMAL" else 0.))
        if odom:
            self.node.odom_callback(self.message("map", twist=NS(twist=NS(linear=NS(x=speed, y=0.)))))
        if lane:
            self.lane(confidence=confidence)
        self.node.intersection_callback(Message(data=intersection))
        self.node.publish_command(None)
        output = self.node.output_pub.publish.call_args.args[0]
        status = json.loads(self.node.status_pub.publish.call_args.args[0].data)
        self.advance()
        return output, status

    def enter(self):
        for _ in range(25):
            self.frame()
        for _ in range(15):
            output, status = self.frame("GPS_BLACKOUT")
        return output, status

    def test_duplicate_lane_images_do_not_build_primary_streak(self):
        for _ in range(6):
            self.lane(stamp=100.)
            self.advance()
        self.assertFalse(self.node.lane_is_primary_usable(self.now))

    def test_assist_confidence_does_not_count_toward_primary_streak(self):
        for _ in range(4):
            self.lane(confidence=.6)
            self.advance()
        self.lane(confidence=.95)
        self.assertFalse(self.node.lane_is_primary_usable(self.now))

    def test_new_nan_lane_cannot_use_old_filter_history(self):
        self.lane()
        self.advance()
        self.lane(lateral=float("nan"))
        self.assertFalse(self.node.lane_is_usable(self.now))

    def test_camera_gap_requires_new_stable_sequence(self):
        for _ in range(5):
            self.lane()
            self.advance()
        self.advance(1.)
        self.lane()
        self.assertFalse(self.node.lane_is_primary_usable(self.now))

    def test_old_or_future_lane_is_not_usable(self):
        for stamp in (0., 90., 101.):
            self.lane(stamp=stamp)
            self.assertFalse(self.node.lane_is_usable(self.now))

    def test_missing_speed_cannot_leave_accel_command(self):
        command = Command()
        self.node.apply_fallback_speed_cap(command)
        self.assertEqual((command.accel, command.brake), (0., 1.))

    def test_replayed_odometry_does_not_refresh_speed(self):
        self.frame()
        self.advance(1.)
        self.node.odom_callback(self.message("map", 100., twist=NS(twist=NS(linear=NS(x=1., y=0.)))))
        self.assertIsNone(self.node.measured_speed_mps())

    def test_imu_loss_during_blackout_forces_stop(self):
        self.enter()
        output, _ = self.frame("GPS_BLACKOUT", imu_stale=True)
        self.assertEqual((output.accel, output.brake), (0., 1.))

    def test_entry_delay_does_not_fall_through_to_degraded_assist(self):
        for _ in range(25):
            self.frame()
        output, status = self.frame("GPS_BLACKOUT")
        self.assertEqual(output.brake, 1.)
        self.assertNotEqual(status["mode"], "degraded_camera_assist")

    def test_blackout_uses_lane_not_unreliable_path_steering(self):
        self.enter()
        for _ in range(20):
            output, status = self.frame("GPS_BLACKOUT", steering=-.6)
        self.assertEqual(status["mode"], "gps_blackout_camera_fallback")
        self.assertAlmostEqual(output.steering, self.node.lane_steering())

    def test_recovery_does_not_immediately_jump_to_nominal(self):
        self.enter()
        previous = self.node.last_output_steering
        output, status = self.frame("NORMAL", steering=-.6)
        self.assertLessEqual(abs(output.steering - previous), self.node.max_steering_rate * .05 + 1e-8)
        self.assertNotEqual(status["mode"], "normal_nominal")
        for _ in range(70):
            output, status = self.frame("NORMAL", steering=-.6)
        self.assertEqual(status["mode"], "normal_nominal")
        self.assertAlmostEqual(output.steering, -.6)

    def test_low_confidence_grace_removes_accel_then_stops(self):
        self.enter()
        output, _ = self.frame("GPS_BLACKOUT", confidence=.4)
        self.assertEqual(output.accel, 0.)
        self.assertGreater(output.brake, 0.)
        for _ in range(8):
            output, _ = self.frame("GPS_BLACKOUT", confidence=.4)
        self.assertEqual(output.brake, 1.)

    def test_no_normal_anchor_means_no_blackout_departure(self):
        for _ in range(30):
            output, _ = self.frame("GPS_BLACKOUT")
        self.assertEqual(output.brake, 1.)

    def test_unknown_quality_state_stops(self):
        self.enter()
        output, _ = self.frame("NOT_A_STATE")
        self.assertEqual(output.brake, 1.)

    def test_bad_nominal_cannot_escape_in_normal_mode(self):
        self.frame()
        self.node.nominal_callback(Command(accel=float("nan")))
        self.node.publish_command(None)
        output = self.node.output_pub.publish.call_args.args[0]
        self.assertEqual((output.accel, output.brake), (0., 1.))

    def test_blackout_budget_stays_exhausted_across_one_normal_frame(self):
        self.enter()
        self.node.blackout_max_duration_sec = .8
        for _ in range(5):
            output, status = self.frame("GPS_BLACKOUT")
        self.assertTrue(status["blackout_budget_exhausted"])
        self.assertEqual(output.brake, 1.)
        self.frame("NORMAL")
        output, status = self.frame("GPS_BLACKOUT")
        self.assertTrue(status["blackout_budget_exhausted"])
        self.assertEqual(output.brake, 1.)

    def test_blackout_distance_budget_uses_speed_integral(self):
        self.enter()
        self.node.blackout_max_distance_m = .8
        for _ in range(10):
            output, status = self.frame("GPS_BLACKOUT")
        self.assertTrue(status["blackout_budget_exhausted"])
        self.assertEqual(output.brake, 1.)

    def test_clock_reset_discards_cached_commands_and_lane_history(self):
        self.enter()
        self.ros_now = 1.
        self.node.publish_command(None)
        output = self.node.output_pub.publish.call_args.args[0]
        self.assertEqual(output.brake, 1.)
        self.assertFalse(self.node.had_normal)
        self.assertFalse(self.node.lane_is_primary_usable(self.now))

    def test_intersection_timeout_cannot_clear_previous_detection(self):
        self.enter()
        self.node.intersection_callback(Message(data=True))
        self.advance(1.)
        self.assertTrue(self.node.intersection_is_active(self.now))

    def test_braking_does_not_snap_steering_to_zero(self):
        self.enter()
        previous = self.node.last_output_steering
        output, _ = self.frame("GPS_BLACKOUT", imu_stale=True)
        self.assertEqual(output.brake, 1.)
        self.assertEqual(output.steering, previous)

    def test_new_primary_sequence_required_after_lane_loss(self):
        self.enter()
        for _ in range(8):
            self.frame("GPS_BLACKOUT", confidence=.4)
        output, _ = self.frame("GPS_BLACKOUT")
        self.assertEqual(output.brake, 1.)
        for _ in range(5):
            output, status = self.frame("GPS_BLACKOUT")
        self.assertEqual(status["mode"], "gps_blackout_camera_fallback")

    def test_recovery_wait_restarts_if_quality_drops_again(self):
        self.enter()
        for _ in range(15):
            self.frame("NORMAL")
        self.frame("GPS_BLACKOUT")
        output, status = self.frame("NORMAL", steering=-.6)
        self.assertEqual(status["mode"], "recovery_camera")

    def test_lane_backlog_burst_does_not_prove_continuous_visibility(self):
        for i in range(5):
            self.lane(stamp=99.8 + i * .05)
        self.assertFalse(self.node.lane_is_primary_usable(self.now))
        for _ in range(5):
            self.advance()
            self.lane()
        self.assertTrue(self.node.lane_is_primary_usable(self.now))

    def test_frozen_image_brakes_before_transport_timeout_then_requires_new_frames(self):
        self.enter()
        previous = self.node.last_output_steering
        for _ in range(4):
            output, status = self.frame("GPS_BLACKOUT", lane=False)
            if status["mode"] == "gps_blackout_camera_fallback":
                previous = output.steering
        self.assertEqual(status["mode"], "lane_loss_braking")
        self.assertEqual(output.accel, 0.)
        self.assertGreater(output.brake, 0.)
        self.assertEqual(output.steering, previous)
        output, status = self.frame("GPS_BLACKOUT")
        self.assertFalse(status["lane_primary_usable"])
        for _ in range(5):
            output, status = self.frame("GPS_BLACKOUT")
        self.assertEqual(status["mode"], "gps_blackout_camera_fallback")

    def test_conflicting_duplicate_lane_revokes_old_permission(self):
        self.enter()
        self.lane(valid=False, stamp=self.node.source_stamps["lane"])
        output, status = self.frame("GPS_BLACKOUT", lane=False)
        self.assertEqual((output.accel, output.brake), (0., 1.))
        self.assertFalse(status["lane_primary_usable"])

    def test_filter_does_not_keep_old_geometry_at_low_frame_rate(self):
        for _ in range(5):
            self.lane(lateral=.2)
            self.advance(.1)
        self.assertLessEqual(len(self.node.lateral_history), 4)
        self.assertEqual(len(self.node.lateral_history), len(self.node.lane_history_times))

    def test_brief_bad_quality_between_ticks_restarts_recovery(self):
        self.enter()
        for _ in range(19):
            self.frame("NORMAL")
        self.node.quality_callback(self.message(state="GPS_BLACKOUT", reason="brief_loss", gps_valid=False,
                                                gps_blackout=True, gps_recovering=False,
                                                imu_stale=False, confidence=0.))
        self.advance(.01)
        output, status = self.frame("NORMAL")
        self.assertEqual(status["mode"], "recovery_camera")
        for _ in range(6):
            _, status = self.frame("NORMAL")
        self.assertNotEqual(status["mode"], "normal_nominal")

    def test_speed_loss_latches_distance_budget_until_gps_recovery(self):
        self.enter()
        for _ in range(12):
            self.frame("GPS_BLACKOUT", odom=False)
        output, status = self.frame("GPS_BLACKOUT")
        self.assertTrue(status["blackout_budget_exhausted"])
        self.assertEqual((output.accel, output.brake), (0., 1.))
        for _ in range(50):
            _, status = self.frame("NORMAL")
        self.assertEqual(status["mode"], "normal_nominal")
        self.assertFalse(status["blackout_budget_exhausted"])

    def test_braking_interval_is_included_in_distance_budget(self):
        self.enter()
        self.frame("GPS_BLACKOUT", speed=2.)
        before = self.node.untrusted_distance
        self.frame("GPS_BLACKOUT", speed=0.)
        self.assertGreaterEqual(self.node.untrusted_distance - before, .1 - 1e-9)

    def test_overflowed_speed_magnitude_stops_without_invalid_json(self):
        self.enter()
        self.node.odom_callback(self.message("map", twist=NS(twist=NS(linear=NS(x=1.7e308, y=1.7e308)))))
        output, status = self.frame("GPS_BLACKOUT", odom=False)
        self.assertEqual((output.accel, output.brake), (0., 1.))
        self.assertTrue(status["blackout_budget_exhausted"])

    def test_invalid_lane_reference_or_confirmation_settings_are_rejected(self):
        for key, value in (("~lane_preview_distance_m", float("nan")),
                           ("~lane_heading_distance_m", 6.), ("~wheelbase_m", 0.),
                           ("~lane_control_timeout_sec", .4), ("~lane_stable_sec", 0.),
                           ("~lateral_gain", -1.)):
            with self.subTest(key=key, value=value):
                self.params.clear()
                self.params[key] = value
                with self.assertRaises(ValueError):
                    self.module.CameraLocalizationFallbackController()


class QualityMonitorTest(unittest.TestCase):
    def setUp(self):
        self.fixture = FallbackTest()
        self.fixture.setUp()
        self.module = self.fixture.load("sensor_quality_monitor")
        self.node = self.module.SensorQualityMonitor()

    def sensors(self, valid=True, blackout=False, imu_nan=False, stamp=None):
        c = self.fixture
        self.node.gps_callback(c.message("map", stamp, pose=NS(pose=NS(position=NS(x=1., y=2.))),
                                       twist=NS(twist=NS(linear=NS(x=1., y=0.)))))
        self.node.imu_callback(c.message("imu", stamp, orientation=NS(x=0., y=0., z=0., w=1.),
                                       angular_velocity=NS(x=0., y=0., z=float("nan") if imu_nan else 0.),
                                       linear_acceleration=NS(x=0., y=0., z=9.8)))
        self.node.health_callback(c.message("gps", stamp, valid=valid, blackout=blackout, age_sec=0.,
                                          state="GPS_BLACKOUT" if blackout else "GPS_OK", reason="test"))
        return self.node.snapshot()

    def test_invalid_gps_health_is_not_normal_even_with_fresh_packets(self):
        self.assertEqual(self.sensors(valid=False)["state"], "SENSOR_DEGRADED")

    def test_good_sensor_frames_are_normal(self):
        self.assertEqual(self.sensors()["state"], "NORMAL")

    def test_nan_imu_blocks_blackout_lane_permission(self):
        state = self.sensors(valid=False, blackout=True, imu_nan=True)
        self.assertEqual(state["state"], "SENSOR_DEGRADED")
        self.assertTrue(state["gps_blackout"])
        self.assertTrue(state["imu_stale"])

    def test_gps_only_blackout_preserves_imu_health(self):
        state = self.sensors(valid=False, blackout=True)
        self.assertEqual(state["state"], "GPS_BLACKOUT")
        self.assertFalse(state["imu_stale"])

    def test_repeated_stamps_and_old_source_do_not_refresh_health(self):
        self.sensors()
        self.fixture.advance(1.)
        state = self.sensors(stamp=100.)
        self.assertNotEqual(state["state"], "NORMAL")
        self.assertTrue(state["imu_stale"])

    def test_clock_reset_requires_new_sensor_epoch(self):
        self.sensors()
        self.fixture.ros_now = 1.
        self.assertNotEqual(self.node.snapshot()["state"], "NORMAL")
        self.assertEqual(self.sensors()["state"], "NORMAL")


if __name__ == "__main__":
    unittest.main()
