"""Green entry followed by late stop detections/empty CAM4 frames."""
import copy
import unittest
from types import SimpleNamespace as NS
import test_maneuver_fusion as fixtures
import test_signal_association as signals
from camera_perception.traffic_signal import (
    directional_observation, count_signal_housings, signal_confidence_valid,
)
from turn_signal_controller.signal_association import associate_weighted, Selection, SignalConfirmation


class RouteEntryReleaseTest(unittest.TestCase):
    def setUp(self):
        self.case=fixtures.StrictFusionTest();self.case.setUp()
        self.node=self.case.node
        self.node.require_context=False
        self.node.route_camera_calibrated=True
        self.case.context.update(direction='STRAIGHT',signal_ids=['own'],entry_s=90.,exit_s=125.,
            direction_source='fixture',heading_change_deg=0.,geometry_direction='STRAIGHT')
        self.node.route_signal_contexts=[self.case.context]

    def authorize(self):
        for _ in range(25):output,status=self.case.frame('GREEN')
        self.assertTrue(status['permission'],status)
        self.assertEqual(output.brake,0.)

    def test_late_red_or_yellow_cannot_cancel_crossing_then_strand_on_empty_frames(self):
        for state in ('RED','YELLOW','UNKNOWN'):
            with self.subTest(state=state):
                self.setUp();self.authorize()
                output,status=self.case.frame(state,x=87.1,no_objects=state=='UNKNOWN')
                self.assertTrue(status['event']['committed'],status)
                self.assertFalse(status['entry_fault'])
                self.assertEqual(output.brake,0.)
                for _ in range(40):
                    output,status=self.case.frame('UNKNOWN',x=87.1,no_objects=True)
                    self.assertTrue(status['event']['committed'])
                    self.assertEqual(output.brake,0.,status['reason'])

    def test_red_seen_before_entry_still_revokes_green(self):
        self.authorize()
        output,status=self.case.frame('RED',x=86.)
        self.assertFalse(status['permission'])
        output,status=self.case.frame('UNKNOWN',x=87.1,no_objects=True)
        self.assertTrue(status['entry_fault'])
        self.assertEqual(output.brake,1.)

    def test_queued_red_captured_before_crossing_is_not_hidden_by_newer_frame(self):
        self.authorize();c=self.case.fixture
        for key in ('odom','objects'):
            message=copy.deepcopy(self.node.samples[key].value)
            message.header.stamp=fixtures.Stamp(c.now)
            if key=='objects':message.objects[0].class_name='RED'
            self.node.observe(message,key)
        c.now+=.02
        output,status=self.case.frame('GREEN',x=87.1)
        self.assertTrue(status['entry_fault'])
        self.assertEqual(output.brake,1.)

    def test_empty_array_without_any_prior_green_cannot_grant_entry(self):
        self.case.frame('RED')
        output,status=self.case.frame('UNKNOWN',x=87.1,no_objects=True)
        self.assertFalse(status['event']['committed'])
        self.assertTrue(status['entry_fault'])
        self.assertEqual(output.brake,1.)

    def test_obstacle_still_brakes_a_provisional_crossing(self):
        self.authorize()
        output,status=self.case.frame('RED',x=87.1,obstacle=True)
        self.assertEqual(output.brake,1.)
        self.assertFalse(status['event']['committed'])
        self.assertIn('pedestrian',status['reason'])

    def test_current_junction_does_not_reapply_approach_speed_after_commit(self):
        self.authorize();self.node.speed_governor_enabled=True
        output,status=self.case.frame('RED',x=87.1)
        self.assertTrue(status['event']['committed'])
        self.assertIsNone(self.node.governor_stopline_distance_m)


class SignalConfidenceFloorTest(unittest.TestCase):
    def test_floor_is_exclusive_in_direction_count_ranking_and_confirmation(self):
        for score in (0.,.5,.599999,.6,float('nan'),float('inf'),1.1):
            with self.subTest(score=score):
                box=signals.detection('GREEN',conf=score)
                self.assertFalse(signal_confidence_valid(score))
                self.assertEqual(directional_observation([box])[0],'UNKNOWN')
                self.assertEqual(count_signal_housings([box]),0)
                self.assertFalse(associate_weighted([box],{'own':(320,240)}).valid)
                confirmation=SignalConfirmation()
                for i in range(12):result=confirmation.update(Selection('GREEN',score,True,'own','matched'),100+i*.05)
                self.assertFalse(result.valid)
        box=signals.detection('GREEN',conf=.600001)
        self.assertEqual(directional_observation([box])[0],'GREEN')
        self.assertTrue(associate_weighted([box],{'own':(320,240)}).valid)

    def test_old_lower_parameter_cannot_accept_weak_signal(self):
        self.assertEqual(directional_observation([signals.detection('RED',conf=.6)],.4)[0],'UNKNOWN')

    def test_low_red_is_removed_before_weighting_against_good_green(self):
        chosen=associate_weighted([signals.detection('RED',conf=.6),signals.detection('GREEN',conf=.9)],{'own':(320,240)})
        self.assertTrue(chosen.valid)
        self.assertEqual(chosen.state,'GREEN')

    def test_weak_detection_geometry_cannot_veto_a_good_route_signal(self):
        weak=signals.detection('RED',conf=.6)
        weak.x_center=float('nan')
        chosen=associate_weighted([weak,signals.detection('GREEN',conf=.9)],{'own':(320,240)})
        self.assertTrue(chosen.valid)
        self.assertEqual(chosen.state,'GREEN')
