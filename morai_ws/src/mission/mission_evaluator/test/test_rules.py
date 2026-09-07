#!/usr/bin/env python3
import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from mission_evaluator.rules import CollisionRule, LaneContactRule, SpeedRule, StartRule


class RulesTest(unittest.TestCase):
    def test_start_deadline_success(self):
        rule = StartRule(route_length_m=100.0)
        rule.update(0.0, 0.0, 1.0)
        self.assertEqual(rule.state, "RUNNING")
        self.assertIsNone(rule.update(10.0, 5.0, 10.0))
        self.assertEqual(rule.state, "SUCCESS")

    def test_start_deadline_failure(self):
        rule = StartRule(route_length_m=100.0)
        rule.update(0.0, 0.0, 1.0)
        event = rule.update(60.1, 4.9, 10.0)
        self.assertEqual(rule.state, "FAILED")
        self.assertEqual(event.reason, "start_progress_deadline_missed")

    def test_speed_immediate_and_sustained_penalties(self):
        rule = SpeedRule()
        self.assertEqual(len(rule.update(0.0, 61.0)), 1)
        self.assertEqual(len(rule.update(2.9, 61.0)), 0)
        self.assertEqual(len(rule.update(3.0, 61.0)), 1)
        self.assertEqual(len(rule.update(6.0, 61.0)), 1)
        self.assertEqual(len(rule.update(7.0, 60.0)), 0)

    def test_lane_duration_and_blackout_exemption(self):
        rule = LaneContactRule()
        rule.update(0.0, True)
        self.assertEqual(len(rule.update(2.9, True)), 0)
        self.assertEqual(len(rule.update(3.0, True)), 1)
        self.assertEqual(len(rule.update(6.0, True)), 1)
        self.assertEqual(len(rule.update(7.0, True, exempt=True)), 0)

    def test_collision_recontact_only_after_release(self):
        rule = CollisionRule()
        self.assertEqual(len(rule.update(0.0, ["object:7"])), 1)
        self.assertEqual(len(rule.update(0.2, ["object:7"])), 0)
        self.assertEqual(len(rule.update(1.0, [])), 0)
        self.assertEqual(len(rule.update(1.1, ["object:7"])), 1)


if __name__ == "__main__":
    unittest.main()
