import unittest

from turn_signal_controller.scheduler import (
    LEFT,
    OFF,
    Maneuver,
    TurnSignalScheduler,
    parse_maneuvers,
)


class TurnSignalSchedulerTest(unittest.TestCase):
    def test_turns_on_five_seconds_before_event(self):
        scheduler = TurnSignalScheduler(
            [Maneuver("left_1", 100.0, LEFT, end_s_m=120.0)],
            lead_time_sec=5.0,
            min_prediction_speed_mps=1.0,
        )

        self.assertEqual(scheduler.update(89.0, 2.0, 0.0).direction, OFF)
        decision = scheduler.update(90.0, 2.0, 1.0)
        self.assertEqual(decision.direction, LEFT)
        self.assertEqual(decision.phase, "lead")
        self.assertAlmostEqual(decision.eta_sec, 5.0)

    def test_stays_on_until_end_distance(self):
        scheduler = TurnSignalScheduler(
            [Maneuver("left_1", 100.0, LEFT, end_s_m=120.0)],
            lead_time_sec=5.0,
            min_prediction_speed_mps=1.0,
        )

        scheduler.update(95.0, 1.0, 0.0)
        self.assertEqual(scheduler.update(110.0, 1.0, 2.0).direction, LEFT)
        self.assertEqual(scheduler.update(120.0, 1.0, 3.0).direction, OFF)

    def test_parse_alias_and_duration_event(self):
        maneuvers = parse_maneuvers(
            [{"distance_m": 10, "direction": "우회전", "duration_sec": 3}]
        )
        self.assertEqual(maneuvers[0].direction, "RIGHT")
        self.assertIsNone(maneuvers[0].end_s_m)
        self.assertEqual(maneuvers[0].duration_sec, 3.0)


if __name__ == "__main__":
    unittest.main()
