"""Stop-line permissions across callback/timer ordering (mock ROS/UDP only)."""

import copy
import unittest

import test_maneuver_fusion as fixtures


class SignalTemporalSafetyTest(unittest.TestCase):
    def setUp(self):
        self.case = fixtures.StrictFusionTest()
        self.case.setUp()
        self.c = self.case.fixture
        self.node = self.case.node

    def image(self, state, stamp=None):
        message = copy.deepcopy(self.node.samples["objects"].value)
        message.header.stamp = fixtures.Stamp(self.c.now if stamp is None else stamp)
        message.objects[0].class_name = state
        self.node.observe(message, "objects")

    def pose(self, stamp=None):
        message = copy.deepcopy(self.node.samples["odom"].value)
        message.header.stamp = fixtures.Stamp(self.c.now if stamp is None else stamp)
        self.node.observe(message, "odom")

    def test_red_then_arrow_between_ticks_requires_reconfirmation(self):
        self.case.authorize()
        self.image("RED")
        self.c.now += .02
        output, status = self.case.frame("LEFT")
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)
        for _ in range(25):
            output, status = self.case.frame("LEFT")
        self.assertTrue(status["permission"])

    def test_intertick_red_cannot_be_erased_at_axle_crossing(self):
        self.case.authorize()
        self.image("RED")
        self.c.now += .02
        output, status = self.case.frame("LEFT", x=87.1)
        self.assertFalse(status["event"]["committed"])
        self.assertTrue(status["entry_fault"])
        self.assertEqual(output.brake, 1.)

    def test_unknown_between_ticks_also_discards_old_permission(self):
        self.case.authorize()
        self.image("UNKNOWN")
        self.c.now += .02
        output, status = self.case.frame("LEFT")
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)

    def test_late_pose_retries_original_image_once_without_new_frame(self):
        self.case.frame()
        self.c.now += .15
        source_stamp = self.c.now
        self.node.pose_history.clear()
        self.pose(stamp=source_stamp - .1)
        self.image("LEFT")
        output, status = self.c.tick(refresh=False)
        self.assertEqual(status["signal_selection_reason"], "signal_pose_unsynchronized")
        self.assertEqual(output.brake, 1.)
        self.c.now += .02
        self.pose(stamp=source_stamp)
        _, status = self.c.tick(refresh=False)
        self.assertEqual(status["signal_selection_reason"], "signal_unconfirmed")
        self.assertEqual(status["selected_signal_id"], "own")
        for _ in range(6):
            self.c.now += .05
            _, status = self.c.tick(refresh=False)
        self.assertFalse(status["permission"])

    def test_overflow_does_not_silently_keep_old_green(self):
        self.case.authorize()
        for _ in range(40):
            self.image("LEFT")
            self.c.now += .001
        self.pose()
        output, status = self.c.tick(refresh=False)
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)
        self.assertEqual(status["signal_queue_overflows"], 1)
        self.assertEqual(status["signal_selection_reason"], "signal_queue_overflow")

    def test_missing_pose_for_older_red_cannot_be_skipped_for_newer_arrow(self):
        self.case.authorize()
        self.c.now += .15
        self.node.pose_history.clear()
        self.image("RED")
        self.c.now += .1
        self.pose()
        self.image("LEFT")
        output, status = self.c.tick(refresh=False)
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)

    def test_late_pose_cannot_refresh_expired_image(self):
        self.case.frame()
        self.c.now += .15
        source_stamp = self.c.now
        self.node.pose_history.clear()
        self.pose(stamp=source_stamp - .1)
        self.image("LEFT")
        self.c.tick(refresh=False)
        self.c.now += .81
        self.pose(stamp=source_stamp)
        output, status = self.c.tick(refresh=False)
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)
        self.assertEqual(status["signal_pending_frames"], 0)

    def test_conflicting_same_stamp_cannot_keep_confirmed_arrow(self):
        self.case.authorize()
        self.image("RED", stamp=self.node.samples["objects"].stamp)
        output, status = self.c.tick(refresh=False)
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)
        self.assertEqual(status["signal_selection_reason"], "signal_duplicate_conflict")

    def test_identical_duplicate_does_not_revoke_or_add_confirmation(self):
        self.case.authorize()
        self.image("LEFT", stamp=self.node.samples["objects"].stamp)
        output, status = self.c.tick(refresh=False)
        self.assertTrue(status["permission"])
        self.assertEqual(output.brake, 0.)

    def test_signal_subscriber_has_bounded_observation_buffer(self):
        calls = self.c.ros.Subscriber.call_args_list
        for topic in ("/detection/traffic_light", "/perception/traffic_light/directional_state"):
            call = next(call for call in calls if call.args[0] == topic)
            self.assertEqual(call.kwargs["queue_size"], 32)


class LegacySignalTemporalSafetyTest(unittest.TestCase):
    def test_red_then_arrow_between_ticks_revokes_legacy_core_green(self):
        c = fixtures.FusionNodeTest()
        c.setUp()
        c.run_ticks(120)
        self.assertTrue(c.node.enter_permission)
        c.sample("signal", state="RED", confidence=.9, valid=True)
        c.now += .02
        output, status = c.tick(signal="LEFT")
        self.assertFalse(status["permission"])
        self.assertEqual(output.brake, 1.)


if __name__ == "__main__":
    unittest.main()
