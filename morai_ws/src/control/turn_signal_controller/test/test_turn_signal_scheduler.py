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

    def test_duration_starts_at_maneuver_not_early_lamp_activation(self):
        scheduler = TurnSignalScheduler([Maneuver("left", 100., LEFT, duration_sec=3.)])
        self.assertEqual(scheduler.update(95., 1., 0.).phase, "lead")
        self.assertEqual(scheduler.update(95., 0., 30.).phase, "lead")
        self.assertEqual(scheduler.update(100., 1., 31.).direction, LEFT)
        self.assertEqual(scheduler.update(102., 1., 33.9).direction, LEFT)
        self.assertEqual(scheduler.update(103., 1., 34.).direction, OFF)

    def test_bad_measurement_does_not_consume_event(self):
        scheduler = TurnSignalScheduler([Maneuver("left", 100., LEFT, end_s_m=120.)])
        self.assertEqual(scheduler.update(99., float("nan"), 1.).direction, OFF)
        self.assertIsNone(scheduler.active)

    def test_clock_reset_rearms_duration_event(self):
        scheduler = TurnSignalScheduler([Maneuver("left", 100., LEFT, duration_sec=3.)])
        scheduler.update(100., 1., 100.)
        self.assertEqual(scheduler.update(104., 1., 104.).direction, OFF)
        self.assertEqual(scheduler.update(100., 1., 1.).direction, LEFT)


if __name__ == "__main__":
    unittest.main()
