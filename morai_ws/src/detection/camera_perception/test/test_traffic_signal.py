import unittest

from camera_perception.traffic_signal import (
    TrafficSignalStopLatch,
    traffic_signal_has_green,
    traffic_signal_requires_stop,
    traffic_bbox_plausible,
)


class TrafficSignalTest(unittest.TestCase):
    def test_oblique_and_lower_signal_boxes_survive_to_route_association(self):
        self.assertTrue(traffic_bbox_plausible(600., 400., 8., 30., 480))
        self.assertTrue(traffic_bbox_plausible(50., 90., 30., 8., 480))
        for values in ((-1., 100., 8., 30., 480), (30., 100., 0., 30., 480),
                       (30., float("nan"), 8., 30., 480), (30., 500., 8., 30., 480)):
            self.assertFalse(traffic_bbox_plausible(*values))

    def test_stops_for_red_only_and_all_yellow_variants(self):
        for class_name in (
            "Red",
            "Red_Yellow",
            "Yellow",
            "yellow_left",
            "Amber",
        ):
            self.assertTrue(
                traffic_signal_requires_stop([class_name]),
                msg=class_name,
            )

    def test_turn_arrow_does_not_authorize_straight_travel(self):
        for names in (["Red_Left"], ["Red_Right"], ["Red", "Left"], ["RED", "Green_Left"]):
            self.assertTrue(traffic_signal_requires_stop(names))

    def test_no_detection_is_unknown_and_combined_green_is_supported(self):
        self.assertFalse(traffic_signal_requires_stop([]))
        self.assertFalse(traffic_signal_requires_stop(["Green_Left"]))
        self.assertTrue(traffic_signal_requires_stop(["Green", "Green_Left", "Left"]))

    def test_only_unambiguous_recognized_green_allows_straight(self):
        green_classes = (
            "Green",
            "Green_Left",
            "Green_Right",
        )
        for class_name in green_classes:
            self.assertTrue(traffic_signal_has_green([class_name]), msg=class_name)
            self.assertFalse(
                traffic_signal_requires_stop([class_name]),
                msg=class_name,
            )

        for names in (["Red", "Green", "Yellow"], ["Red", "Green_Left"],
                      ["RED_Green left"], ["RED_Green right"], ["Green_Arrow"],
                      ["Yellow_Green_Arrow"], ["greenish"]):
            self.assertFalse(traffic_signal_has_green(names), names)
            self.assertTrue(traffic_signal_requires_stop(names), names)

    def test_empty_frames_never_release_stop(self):
        latch = TrafficSignalStopLatch(clear_confirmation_s=0.5)
        self.assertTrue(latch.update(["Red"], 10.0))
        self.assertTrue(latch.update([], 10.1))
        self.assertTrue(latch.update([], 10.5))
        self.assertTrue(latch.update([], 10.6))

    def test_green_requires_continuous_distinct_frames(self):
        latch = TrafficSignalStopLatch(clear_confirmation_s=0.5)
        self.assertTrue(latch.update(["Red"], 10.0))
        self.assertTrue(latch.update(["Red", "Green", "Yellow"], 10.1))
        self.assertTrue(latch.update(["Green"], 10.2))
        self.assertTrue(latch.update(["Green"], 10.2, received_sec=10.6))
        self.assertTrue(latch.update(["Green"], 10.6))
        self.assertFalse(latch.update(["Green"], 10.7))

    def test_burst_of_old_green_frames_cannot_release_immediately(self):
        latch = TrafficSignalStopLatch()
        latch.update(["Red"], 10., 20.)
        self.assertTrue(latch.update(["Green"], 10.1, 20.01))
        self.assertTrue(latch.update(["Green"], 10.6, 20.02))

    def test_gap_unknown_reordering_and_conflict_restart_confirmation(self):
        for interruption in ([], ["Red"], ["Green", "Red"]):
            latch = TrafficSignalStopLatch()
            latch.update(["Green"], 10.)
            self.assertTrue(latch.update(interruption, 10.4))
            self.assertTrue(latch.update(["Green"], 10.5))
            self.assertFalse(latch.update(["Green"], 11.))
        latch = TrafficSignalStopLatch()
        latch.update(["Green"], 10.)
        self.assertTrue(latch.update(["Green"], 11.))
        self.assertFalse(latch.update(["Green"], 11.5))
        self.assertTrue(latch.update(["Red"], 11.5))
        self.assertTrue(latch.update(["Green"], 11.6))
        self.assertTrue(latch.update(["Green"], 11.55))
        self.assertTrue(latch.update(["Green"], 12.))

    def test_invalid_parameters_and_zero_stamp(self):
        for value in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                TrafficSignalStopLatch(clear_confirmation_s=value)
        latch = TrafficSignalStopLatch(clear_confirmation_s=0.)
        self.assertTrue(latch.update(["Green"], 0.))
        self.assertTrue(latch.update(["Green"], 1.))
        self.assertFalse(latch.update(["Green"], 1.1))


if __name__ == "__main__":
    unittest.main()
