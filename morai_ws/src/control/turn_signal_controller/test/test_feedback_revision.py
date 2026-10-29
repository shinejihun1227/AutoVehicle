"""Reported mixed signal boxes and stop-before-right-turn regressions."""
import unittest
from types import SimpleNamespace as NS
import test_signal_association as signals
import test_maneuver_fusion as fixtures
from turn_signal_controller.signal_association import associate_weighted
from turn_signal_controller.stop_then_turn import stop_then_turn


class WeightedSignalTest(unittest.TestCase):
    def test_weaker_false_red_does_not_veto_route_green(self):
        boxes = [signals.detection('RED', conf=.66), signals.detection('GREEN', conf=.90)]
        chosen = associate_weighted(boxes, {'own': (320, 240)})
        self.assertEqual(chosen.state, 'GREEN')
        self.assertTrue(chosen.valid)

    def test_high_confidence_other_road_is_excluded(self):
        boxes = [signals.detection('RED', conf=.7), signals.detection('GREEN', x=500, conf=.99)]
        self.assertEqual(associate_weighted(boxes, {'own': (320, 240)},
                                          {'other': (500, 240)}).state, 'RED')

    def test_near_tie_stays_unknown_and_order_is_irrelevant(self):
        boxes = [signals.detection('RED', conf=.81), signals.detection('GREEN', conf=.82)]
        for order in (boxes, boxes[::-1]):
            self.assertFalse(associate_weighted(order, {'own': (320, 240)}).valid)

    def test_no_projected_route_cannot_take_screen_winner(self):
        self.assertFalse(associate_weighted([signals.detection()], {}).valid)

    def test_same_housing_red_arrow_permission_is_preserved(self):
        boxes = [signals.detection('RED'), signals.detection('LEFT')]
        self.assertEqual(associate_weighted(boxes, {'own': (320, 240)}).state, 'RED_LEFT')


class StopThenTurnTest(unittest.TestCase):
    def stopped(self, event, gap=0., speed=0., valid=True):
        return [stop_then_turn(event, speed, gap, 10+i*.05, valid) for i in range(35)]

    def test_full_dwell_near_line_then_persistent_release(self):
        event = {}
        values = self.stopped(event)
        self.assertFalse(any(values[:20]))
        self.assertTrue(values[-1])
        self.assertTrue(stop_then_turn(event, 4., -4., 12., True))
        self.assertFalse(stop_then_turn({}, 4., -4., 12., True))

    def test_far_stop_rolling_and_invalid_pose_cannot_authorize(self):
        for options in ({'gap': 12.}, {'speed': .5}, {'valid': False}):
            self.assertFalse(any(self.stopped({}, **options)))

    def test_scheduling_pause_restarts_dwell(self):
        event = {}
        for i in range(15):
            stop_then_turn(event, 0., 0., 10+i*.05, True)
        self.assertFalse(stop_then_turn(event, 0., 0., 12., True))
        self.assertNotIn('right_stop_since', event)


class RightTurnNodeTest(unittest.TestCase):
    def setUp(self):
        self.case = fixtures.StrictFusionTest()
        self.case.setUp()
        self.node = self.case.node
        self.node.core.hold_distance_m = 0.5
        self.node.core_params['hold_distance_m'] = 0.5
        self.node.require_context = False
        self.node.route_camera_calibrated = True
        self.node.test_without_turn_signals = True
        context = self.case.context
        context.update(direction='RIGHT', signal_ids=['own'], entry_s=90., exit_s=120.,
                       direction_source='fixture', heading_change_deg=-90., geometry_direction='RIGHT')
        self.node.route_signal_contexts = [context]
        self.node.stop_then_right_context_ids = {'junction'}

    def test_stop_then_release_without_signal_and_retain_obstacle_braking(self):
        for i in range(45):
            output, status = self.case.frame('RED', line=True)
            if i < 20:
                self.assertFalse(status['permission'])
        self.assertTrue(status['permission'], status)
        self.assertEqual(output.brake, 0.)
        self.assertTrue(self.node.event['right_stop_released'])
        output, status = self.case.frame('UNKNOWN', no_objects=True, obstacle=True)
        self.assertEqual(output.brake, 1.)
        self.assertFalse(status['permission'])

    def test_green_passes_without_stopping_far_upstream(self):
        for _ in range(45):
            _, status = self.case.frame('GREEN', x=75.)
        self.assertTrue(status['permission'], status)
        self.assertFalse(self.node.event.get('right_stop_released', False))
        self.assertEqual(self.node.event['right_entry_policy'], 'green_priority')

    def test_green_at_stop_target_releases_and_missing_head_does_not_restop(self):
        for _ in range(15):
            output, status = self.case.frame('GREEN')
        self.assertTrue(status['permission'], status)
        self.assertEqual(output.brake, 0.)
        output, status = self.case.frame('UNKNOWN', no_objects=True)
        self.assertTrue(status['permission'], status)

    def test_red_before_entry_revokes_green_and_requires_stop(self):
        for _ in range(15):
            _, status = self.case.frame('GREEN', x=75.)
        self.assertTrue(status['permission'])
        _, status = self.case.frame('RED', x=75.)
        self.assertFalse(status['permission'])


class NarrowLaneTest(unittest.TestCase):
    def test_two_observed_narrow_boundaries_enable_aid_without_accepting_too_small_gap(self):
        from curvature_speed_purepursuit.lane_centering import lane_quality
        for width, accepted in ((2.5, True), (2.2, False)):
            info = dict(lane_valid=True, frame_id='base_link', center_source='both',
                        coordinate_convention={'x': 'forward_m', 'y': 'left_m'},
                        lane_width_m=width, centerline_points=[[7., 0.], [14., 0.]])
            for side, sign in (('left', 1), ('right', -1)):
                info[side+'_lane'] = dict(detected=True, from_guide=False, coasted=False,
                    confidence=.9, n_points=10, x_range_m=[7., 14.])
                info[side+'_boundary_points'] = [[7., sign*width/2], [14., sign*width/2]]
            self.assertEqual(lane_quality(info, .65)[0], accepted)
