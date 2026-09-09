"""Stop-line edge cases found in the second review (no ROS required)."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from stopline_control.core import AccelRiseLimiter, StopLineControllerCore


class StopLineRegressionTest(unittest.TestCase):
    def test_conflicting_duplicate_revokes_green_until_new_confirmation(self):
        for state, confidence, valid in (("RED", 1., True), ("UNKNOWN", 1., True),
                                         ("GREEN", .1, True), ("GREEN", 1., False)):
            with self.subTest(state=state, confidence=confidence, valid=valid):
                core = StopLineControllerCore()
                self.signal(core, 10.)
                self.line(core, 10., .5)
                core.update(10., 10., 0.)
                self.signal(core, 10.1, "GREEN")
                core.update(10.1, 10.1, 0.)
                self.signal(core, 10.45, "GREEN")
                self.assertEqual(core.update(10.45, 10.45, 0.).mode, "NOMINAL")
                self.assertFalse(core.observe_signal(state, confidence, valid, 10.45, 10.5, 10.5))
                self.assertEqual(core.update(10.5, 10.5, 0.).brake, 1.)
                self.assertEqual(core.last_signal_stamp, 10.45)
                self.signal(core, 10.55, "GREEN")
                self.assertEqual(core.update(10.55, 10.55, 0.).brake, 1.)
                self.signal(core, 10.9, "GREEN")
                self.assertEqual(core.update(10.9, 10.9, 0.).mode, "NOMINAL")

    def test_identical_duplicate_does_not_refresh_or_revoke_signal(self):
        core = StopLineControllerCore()
        self.signal(core, 10., "GREEN")
        self.assertFalse(core.observe_signal("GREEN", 1., True, 10., 10.1, 10.1))
        self.assertEqual(core.signal.received, 10.)
        self.assertEqual(core.green_samples, 1)
        self.assertFalse(core.stop_requested)

    def test_cached_green_cannot_release_after_control_update_gap(self):
        core = StopLineControllerCore()
        self.signal(core, 10.)
        self.line(core, 10., .5)
        core.update(10., 10., 0.)
        self.signal(core, 10.1, "GREEN")
        core.update(10.1, 10.1, 0.)
        self.signal(core, 10.45, "GREEN")
        self.assertEqual(core.update(10.45, 10.45, 0.).mode, "NOMINAL")
        # Last GREEN is within its 0.8s timeout, but control was blind >0.5s.
        self.assertEqual(core.update(11.05, 11.05, 0.).brake, 1.)

    def test_missing_speed_with_green_and_no_previous_stop_fails_closed(self):
        core = StopLineControllerCore()
        self.signal(core, 10., "GREEN")
        decision = core.update(10., 10., None)
        self.assertEqual((decision.accel_limit, decision.brake), (0., 1.))

    def test_invalid_clock_cannot_preserve_confirmed_green(self):
        core = StopLineControllerCore()
        self.signal(core, 10., "GREEN")
        core.update(10., 10., 0.)
        self.signal(core, 10.35, "GREEN")
        self.assertEqual(core.update(10.35, 10.35, 0.).mode, "NOMINAL")
        self.assertEqual(core.update(float("nan"), 10.4, 0.).reason, "invalid_clock")
        self.assertEqual(core.update(10.45, 10.45, 0.).brake, 1.)

    @staticmethod
    def signal(core, stamp, state="RED", received=None):
        now = stamp if received is None else received
        core.observe_signal(state, 1., True, stamp, now, stamp)

    @staticmethod
    def line(core, stamp, distance, received=None, ros_now=None):
        received = stamp if received is None else received
        core.observe_line(distance, 1., True, stamp, received,
                          stamp if ros_now is None else ros_now)

    def test_delayed_line_uses_motion_during_braking_not_latest_speed(self):
        core = StopLineControllerCore()
        self.signal(core, 10.)
        for i in range(5):
            core.update(10. + i * .1, 10. + i * .1, 4. - i * .6)
        self.line(core, 10., 4., received=10.5, ros_now=10.5)
        decision = core.update(10.5, 10.5, 1.)
        # 4 -> 1 m/s over 0.5s means 1.25m travel, NOT 1*0.5m.
        self.assertAlmostEqual(decision.distance_m, 2.75)

    def test_delayed_line_without_motion_history_cannot_authorize_accel(self):
        core = StopLineControllerCore()
        self.signal(core, 10.5)
        self.line(core, 10., 4., received=10.5, ros_now=10.5)
        decision = core.update(10.5, 10.5, 1.)
        self.assertEqual(decision.brake, 1.)
        self.assertIsNone(decision.distance_m)

    def test_expired_active_stop_cannot_switch_to_a_farther_line(self):
        core = StopLineControllerCore(max_dead_reckoning_sec=.3)
        self.signal(core, 10.)
        self.line(core, 10., 2.)
        core.update(10., 10., 0.)
        core.update(10.4, 10.4, 0.)
        self.line(core, 10.5, 10.)
        decision = core.update(10.5, 10.5, 0.)
        self.assertEqual((decision.accel_limit, decision.brake), (0., 1.))

    def test_motion_gap_cannot_be_hidden_by_a_new_farther_line(self):
        core = StopLineControllerCore()
        self.signal(core, 10.)
        self.line(core, 10., 2.)
        core.update(10., 10., 1.)
        self.line(core, 11., 10.)
        decision = core.update(11., 11., 1.)
        self.assertEqual(decision.brake, 1.)

    def test_green_cannot_release_hold_without_valid_speed(self):
        core = StopLineControllerCore()
        self.signal(core, 10.)
        self.line(core, 10., .5)
        core.update(10., 10., 0.)
        self.signal(core, 10.1, "GREEN")
        core.update(10.1, 10.1, None)
        self.signal(core, 10.45, "GREEN")
        decision = core.update(10.45, 10.45, None)
        self.assertEqual(decision.brake, 1.)
        self.assertTrue(core.holding)
        # Speed recovery alone must not resurrect green recorded while blind.
        self.assertEqual(core.update(10.5, 10.5, 0.).brake, 1.)
        self.signal(core, 10.55, "GREEN")
        self.assertEqual(core.update(10.55, 10.55, 0.).brake, 1.)
        self.signal(core, 10.9, "GREEN")
        self.assertEqual(core.update(10.9, 10.9, 0.).mode, "NOMINAL")

    def test_prediction_limit_checks_ros_time_in_accelerated_replay(self):
        core = StopLineControllerCore(max_dead_reckoning_sec=.3)
        self.signal(core, 10., received=100.)
        self.line(core, 10., 4., received=100.)
        core.update(100., 10., 0.)
        decision = core.update(100.04, 10.4, 0.)
        self.assertEqual(decision.brake, 1.)

    def test_low_speed_inside_stop_tolerance_does_not_release_brake(self):
        core = StopLineControllerCore()
        self.signal(core, 10.)
        self.line(core, 10., .52)
        decision = core.update(10., 10., .1)
        self.assertEqual((decision.mode, decision.accel_limit, decision.brake), ("HOLD", 0., 1.))

    def test_same_map_target_can_recover_but_other_target_cannot(self):
        for identity, expected_brake in (("junction_A", 0.), ("junction_B", 1.), (None, 1.)):
            with self.subTest(identity=identity):
                core = StopLineControllerCore()
                self.signal(core, 10.)
                core.observe_line(10., 1., True, 10., 10., 10., target_id="junction_A")
                core.update(10., 10., 1.)
                core.observe_line(9., 1., True, 11., 11., 11., target_id=identity)
                decision = core.update(11., 11., 1.)
                self.assertEqual(decision.brake, expected_brake)
                self.assertEqual(core.tracking_fault, identity != "junction_A")

    def test_interpolated_frame_time_during_acceleration(self):
        core = StopLineControllerCore()
        for i in range(5):
            core.update(10. + i * .1, 10. + i * .1, 1. + 2. * i * .1)
        self.signal(core, 10.5)
        self.line(core, 10.15, 5., received=10.5, ros_now=10.5)
        decision = core.update(10.5, 10.5, 2.)
        self.assertAlmostEqual(decision.distance_m, 5. - .35 * (1.3 + 2.) / 2.)

    def test_history_hole_is_not_bridged_with_stationary_latest_speed(self):
        core = StopLineControllerCore()
        core.update(10., 10., 4.)
        core.update(10.2, 10.2, None)
        self.signal(core, 10.4)
        self.line(core, 10.1, 5., received=10.4, ros_now=10.4)
        self.assertEqual(core.update(10.4, 10.4, 0.).brake, 1.)

    def test_clock_reset_clears_speed_history(self):
        core = StopLineControllerCore()
        core.update(10., 10., 4.)
        core.update(10.1, 1., 0.)
        self.assertFalse(core.motion_history)


class AccelRiseTest(unittest.TestCase):
    def test_restart_ramps_and_braking_resets_ramp_immediately(self):
        limiter = AccelRiseLimiter()
        self.assertEqual(limiter.limit(1., 0., 10., 10.), 0.)
        self.assertAlmostEqual(limiter.limit(1., 0., 10.05, 10.05), .025)
        self.assertEqual(limiter.limit(1., .7, 10.1, 10.1), 0.)
        self.assertAlmostEqual(limiter.limit(1., 0., 10.15, 10.15), .025)
        self.assertEqual(limiter.limit(0., 0., 10.2, 10.2), 0.)

    def test_pause_gap_and_clock_reset_cannot_accumulate_throttle(self):
        limiter = AccelRiseLimiter()
        limiter.limit(1., 0., 10., 10.)
        self.assertEqual(limiter.limit(1., 0., 10.1, 10.), 0.)
        self.assertEqual(limiter.limit(1., 0., 11., 11.), 0.)
        limiter.limit(1., 0., 11.1, 11.1)
        self.assertEqual(limiter.limit(1., 0., 11.2, 1.), 0.)

    def test_invalid_limiter_parameters_fail_at_startup(self):
        for value in (0., -1., float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                AccelRiseLimiter(value)


if __name__ == "__main__":
    unittest.main()
