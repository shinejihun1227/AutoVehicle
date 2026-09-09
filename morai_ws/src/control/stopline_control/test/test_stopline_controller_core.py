#!/usr/bin/env python3
"""Temporal regression scenarios; no ROS, simulator or model imports required."""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))
from stopline_control.core import Sample, StopLineControllerCore


class StopLineCoreTest(unittest.TestCase):
    def setUp(self):
        self.core = StopLineControllerCore()

    def signal(self, t, state="RED", confidence=1.0):
        return self.core.observe_signal(state, confidence, True, t, t, t)

    def line(self, t, distance):
        return self.core.observe_line(distance, 1.0, True, t, t, t)

    def step(self, t, speed=2.0):
        return self.core.update(t, t, speed)

    def test_old_future_zero_and_replayed_stamps_rejected(self):
        for stamp in (0.0, 8.0, 11.0):
            self.assertFalse(self.core.observe_line(5., 1., True, stamp, 10., 10.))
        self.assertTrue(self.line(10., 5.))
        self.assertFalse(self.line(10., 50.))
        self.assertFalse(self.line(9.9, 50.))
        self.assertFalse(Sample(10., 10., None).fresh(10.1, 12., 0.8))

    def test_red_without_distance_stops_instead_of_passing_nominal(self):
        self.signal(10.)
        self.assertEqual(self.step(10.).brake, 1.)

    def test_loss_of_line_and_signal_does_not_release_stop(self):
        self.signal(10.)
        self.line(10., 6.)
        self.step(10.)
        for i in range(1, 31):
            t = 10. + i * 0.05
            self.core.observe_line(0., 0., False, t, t, t)
            d = self.step(t)
        self.assertTrue(self.core.stop_requested)
        self.assertAlmostEqual(d.distance_m, 3., places=5)
        self.assertEqual(d.mode, "APPROACH")
        self.assertGreater(d.brake, 0.)

    def test_late_distance_compensates_processing_delay_and_front_offset(self):
        self.core = StopLineControllerCore(front_reference_offset_m=2.)
        for i in range(5):
            self.step(9.5 + i * 0.1)
        self.signal(10.)
        self.core.observe_line(10., 1., True, 9.5, 10., 10.)
        self.assertAlmostEqual(self.step(10.).distance_m, 7.)

    def test_hold_requires_new_continuous_explicit_green(self):
        self.signal(10.)
        self.line(10., 0.5)
        self.assertEqual(self.step(10., 0.).mode, "HOLD")
        self.signal(10.1, "UNKNOWN")
        self.assertEqual(self.step(10.1, 0.).brake, 1.)
        self.signal(10.2, "GREEN")
        self.assertEqual(self.step(10.2, 0.).brake, 1.)
        self.assertFalse(self.signal(10.2, "GREEN"))  # duplicate cannot confirm
        self.signal(10.55, "GREEN")
        self.assertEqual(self.step(10.55, 0.).mode, "NOMINAL")

    def test_delayed_green_cannot_release_hold(self):
        self.signal(10.)
        self.line(10., 0.5)
        self.step(10., 0.)
        self.assertFalse(self.core.observe_signal("GREEN", 1., True, 8., 10.1, 10.1))
        self.assertEqual(self.step(10.1, 0.).brake, 1.)
        self.signal(10.2, "GREEN", 0.1)
        self.assertEqual(self.step(10.2, 0.).brake, 1.)

    def test_long_sensor_gap_forgets_distance_and_stops(self):
        self.signal(10.)
        self.line(10., 6.)
        self.step(10.)
        self.assertEqual(self.step(11.).brake, 1.)
        self.assertIsNone(self.core.distance)

    def test_prediction_is_time_bounded(self):
        self.core = StopLineControllerCore(max_dead_reckoning_sec=0.3)
        self.signal(10.)
        self.line(10., 15.)
        self.step(10.)
        for i in range(1, 9):
            d = self.step(10. + i * 0.05)
        self.assertEqual(d.brake, 1.)
        self.assertEqual(d.reason, "stopline_tracking_lost_awaiting_green")

    def test_motion_uses_ros_elapsed_time_not_playback_wall_time(self):
        self.signal(10.)
        self.line(10., 6.)
        self.step(10.)
        decision = self.core.update(10.1, 10.05, 2.)
        self.assertAlmostEqual(decision.distance_m, 5.9)

    def test_unknown_after_passed_line_does_not_revive_consumed_detection(self):
        self.signal(10., "GREEN")
        self.line(10., 0.05)
        self.step(10.)
        self.signal(10.4, "GREEN")
        self.step(10.4)
        self.assertIsNone(self.core.distance)
        self.signal(10.41, "UNKNOWN")
        self.assertFalse(self.core.stop_requested)

    def test_clock_reset_drops_history_and_requires_green(self):
        self.signal(10.)
        self.line(10., 6.)
        self.step(10.)
        d = self.core.update(10.1, 2., 2.)
        self.assertEqual(d.reason, "clock_reset")
        self.assertEqual(d.brake, 1.)
        self.assertTrue(self.core.holding)

    def test_missing_odometry_stops_and_invalidates_prediction(self):
        self.signal(10.)
        self.line(10., 6.)
        self.step(10.)
        self.assertEqual(self.step(10.1, None).brake, 1.)
        self.assertIsNone(self.core.distance)

    def test_line_before_odometry_preserves_unknown_stop_intent(self):
        self.line(10., 3.)
        self.assertEqual(self.step(10., None).brake, 1.)
        self.assertIsNone(self.core.consumed_line_stamp)
        decision = self.step(10.05, 4.)
        self.assertEqual(decision.mode, "APPROACH")
        self.assertGreater(decision.brake, 0.)
        self.assertAlmostEqual(decision.distance_m, 2.8)

    def test_unknown_between_ticks_cannot_be_erased_by_one_green(self):
        self.signal(9.6, "GREEN")
        self.step(9.6, 1.)
        self.signal(10., "GREEN")
        self.line(10., 0.8)
        self.assertEqual(self.step(10., 1.).mode, "NOMINAL")
        self.signal(10.01, "UNKNOWN")
        self.signal(10.02, "GREEN")
        self.assertTrue(self.core.stop_requested)
        self.assertGreater(self.step(10.05, 1.).brake, 0.)

    def test_next_junction_accepts_new_line_after_passing_old_line(self):
        self.signal(10., "GREEN")
        self.line(10., 2.)
        self.step(10.)
        for i in range(1, 201):
            t = 10. + i * 0.05
            self.signal(t, "GREEN")
            self.step(t)
        self.signal(20.1, "RED")
        self.line(20.1, 10.)
        self.assertEqual(self.step(20.1).mode, "APPROACH")
        self.assertAlmostEqual(self.core.distance, 10.)

    def test_expired_target_allows_next_junction_after_confirmed_green(self):
        self.core = StopLineControllerCore(max_dead_reckoning_sec=0.3)
        self.signal(10.)
        self.line(10., 2.)
        self.step(10., 0.)
        self.assertEqual(self.step(10.4, 0.).brake, 1.)
        self.line(10.5, 10.)
        self.assertEqual(self.step(10.5, 0.).brake, 1.)
        self.signal(10.6, "GREEN")
        self.step(10.6, 0.)
        self.signal(10.95, "GREEN")
        self.assertEqual(self.step(10.95, 0.).mode, "NOMINAL")
        self.signal(11., "RED")
        self.line(11., 10.)
        self.assertEqual(self.step(11., 0.).mode, "APPROACH")
        self.assertAlmostEqual(self.core.distance, 10.)

    def test_next_farther_stopline_cannot_move_active_stop_target(self):
        self.signal(10.)
        self.line(10., 6.)
        self.step(10.)
        self.line(10.1, 30.)
        self.assertLess(self.step(10.1).distance_m, 6.)

    def test_high_speed_brakes_outside_old_20m_window(self):
        self.signal(10.)
        self.line(10., 80.)
        self.assertGreater(self.step(10., 60. / 3.6).brake, 0.)

    def test_confirmed_green_without_stop_request_passes(self):
        self.signal(9.6, "GREEN")
        self.step(9.6)
        self.signal(10., "GREEN")
        self.line(10., 5.)
        self.assertEqual(self.step(10.).mode, "NOMINAL")

    def test_low_speed_closed_loop_stops_despite_camera_blind_near_field(self):
        # Ideal pedal plant only: verify sequencing, not MORAI calibration.
        speed, distance, dt = 2., 12., 0.05
        modes = set()
        for i in range(400):
            t = 10. + i * dt
            self.signal(t)
            if distance >= 6.:
                self.line(t, distance)
            decision = self.step(t, speed)
            modes.add(decision.mode)
            if decision.mode == "HOLD" and speed <= 0.15:
                break
            accel = min(0.5 if speed < 2. else 0., decision.accel_limit)
            if decision.brake > 0:
                accel = 0.
            new_speed = max(0., speed + (accel - 1.5 * decision.brake) * dt)
            distance -= (speed + new_speed) * 0.5 * dt
            speed = new_speed
        self.assertIn("APPROACH", modes)
        self.assertEqual(decision.mode, "HOLD")
        self.assertGreater(distance, 0.)
        self.assertLess(distance, 1.5)


if __name__ == "__main__":
    unittest.main()
