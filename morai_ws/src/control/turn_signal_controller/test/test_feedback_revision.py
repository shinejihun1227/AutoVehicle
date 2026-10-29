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
            if i < 10:
                self.assertFalse(status['permission'])
        self.assertTrue(status['permission'], status)
        self.assertEqual(output.brake, 0.)
        self.assertTrue(self.node.event['right_stop_released'])
        output, status = self.case.frame('UNKNOWN', no_objects=True, obstacle=True)
        self.assertEqual(output.brake, 1.)
        self.assertFalse(status['permission'])

    def test_first_route_green_releases_without_stop_even_upstream(self):
        output, status = self.case.frame('GREEN', x=75.)
        self.assertTrue(status['permission'], status)
        self.assertEqual(output.brake, 0.)
        self.assertEqual(status['entry_policy'], 'green_priority')
        self.assertEqual(status['selected_signal_state'], 'GREEN')
        self.assertFalse(self.node.event.get('right_stop_released', False))

    def test_red_to_green_releases_on_first_frame_without_finishing_dwell(self):
        self.case.frame('RED', line=True)
        _, waiting = self.case.frame('RED', line=True)
        self.assertFalse(waiting['permission'])
        output, status = self.case.frame('GREEN', line=True)
        self.assertTrue(status['permission'], status)
        self.assertEqual(output.brake, 0.)
        self.assertEqual(status['entry_policy'], 'green_priority')
        self.assertFalse(self.node.event.get('right_stop_released', False))

    def test_red_stop_releases_in_half_second_without_second_green_confirmation(self):
        released = None
        for i in range(17):
            output, status = self.case.frame('RED', line=True)
            if status['permission']:
                released = i*.05
                break
        self.assertIsNotNone(released, status)
        self.assertGreaterEqual(released, .5)
        self.assertLessEqual(released, .600001)
        self.assertEqual(output.brake, 0.)
        self.assertEqual(status['entry_policy'], 'stop_then_right_released')

    def test_red_permission_persists_through_red_and_empty_frames_after_stop(self):
        for _ in range(17):output, status = self.case.frame('RED', line=True)
        for state in ('RED', 'UNKNOWN', 'YELLOW'):
            output, status = self.case.frame(state, no_objects=state=='UNKNOWN')
            self.assertTrue(status['permission'], status)
            self.assertEqual(output.brake, 0.)

    def test_green_release_survives_immediate_empty_frame(self):
        self.case.frame('GREEN')
        output, status = self.case.frame('UNKNOWN', no_objects=True)
        self.assertTrue(status['permission'], status)
        self.assertEqual(output.brake, 0.)

    def test_fast_release_slew_stays_below_nominal_and_restores_outside_fourth(self):
        accelerations=[]
        for _ in range(7):
            output, status=self.case.frame('GREEN')
            self.assertEqual(output.brake,0.)
            self.assertLessEqual(output.accel,.6)
            accelerations.append(output.accel)
        self.assertAlmostEqual(accelerations[5],.25,places=5)
        self.assertEqual(status['accel_rise_rate_per_sec'],1.)
        self.node.stop_then_right_context_ids.clear()
        _, status=self.case.frame('GREEN')
        self.assertEqual(status['accel_rise_rate_per_sec'],.5)

    def test_far_red_stop_cannot_release_and_obstacle_overrides_first_green(self):
        for _ in range(30):_, status=self.case.frame('RED',x=75.)
        self.assertFalse(status['permission'])
        output, status=self.case.frame('GREEN',x=75.,obstacle=True)
        self.assertFalse(status['permission'])
        self.assertEqual((output.accel,output.brake),(0.,1.))

    def test_saved_global_false_does_not_disable_fourth_junction_green_priority(self):
        self.node.right_on_green=False
        output,status=self.case.frame('GREEN')
        self.assertTrue(status['permission'],status)
        self.assertEqual(output.brake,0.)
        self.assertIn('RIGHT',status['signal_allowed_directions'])
        other=dict(self.node.event,id='different_junction')
        self.assertFalse(self.node.event_signal_permits('GREEN',other))

    def test_red_to_green_releases_while_car_is_still_moving(self):
        self.node.vehicle_speed_topic='/Ego_topic';self.node.require_vehicle_speed=True
        for state in ('RED','GREEN'):
            now=self.case.fixture.now
            self.node.vehicle_speed_source.observe(1.,now,now,now)
            output,status=self.case.frame(state,x=75.)
        self.assertTrue(status['permission'],status)
        self.assertEqual(output.brake,0.)
        self.assertFalse(self.node.event.get('right_stop_released',False))

    def test_red_stop_release_does_not_reapply_this_junction_approach_cap(self):
        self.node.speed_governor_enabled=True
        for _ in range(17):output,status=self.case.frame('RED',line=True)
        self.assertTrue(status['permission'],status)
        self.assertIsNone(self.node.governor_stopline_distance_m)


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
