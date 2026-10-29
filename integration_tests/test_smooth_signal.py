"""Short control gaps and ranked signal evidence using actual runtime code."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
from purepursuit_mgeo.command_continuity import continue_command
from camera_perception.signal_detection import signal_visual_evidence,signal_label_known
from turn_signal_controller.signal_association import associate


def command(a=.6,b=0.,s=.1):
    return NS(longlCmdType=1,accel=a,brake=b,steering=s,velocity=0.,acceleration=0.)


class ContinuityTests(unittest.TestCase):
    def test_short_gap_keeps_positive_bounded_accel(self):
        original=command()
        result=continue_command(original,100.,100.4,.3,.2,True)
        self.assertGreater(result.accel,0.)
        self.assertLessEqual(result.accel,original.accel)
        self.assertEqual(result.brake,0.)
        self.assertEqual(result.steering,original.steering)
        self.assertEqual(original.accel,.6)

    def test_repeated_hold_does_not_extend_original_deadline(self):
        for t in (100.31,100.4,100.49):
            self.assertIsNotNone(continue_command(command(),100.,t,.3,.2,True))
        self.assertIsNone(continue_command(command(),100.,100.501,.3,.2,True))

    def test_startup_health_stop_and_malformed_cannot_accelerate(self):
        for message,healthy in ((None,True),(command(b=1),True),(command(),False),
                                (command(a=float('nan')),True),(command(s=2.),True)):
            self.assertIsNone(continue_command(message,100.,100.4,.3,.2,healthy))
        self.assertIsNone(continue_command(command(),100.,99.9,.3,.2,True))


class MuxGapTests(unittest.TestCase):
    def setUp(self):
        self.now=100.36
        ros=NS(get_time=lambda:self.now,Time=NS(now=lambda:NS(to_sec=lambda:self.now)))
        spec=importlib.util.spec_from_file_location('_smooth_mux',ROOT/'src/control/purepursuit_mgeo/scripts/no_obstacle_integration_node.py')
        module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{'rospy':ros,'geometry_msgs.msg':NS(PoseStamped=NS),
             'nav_msgs.msg':NS(Path=NS,Odometry=NS),'std_msgs.msg':NS(Bool=NS,String=NS),
             'morai_msgs.msg':NS(CtrlCmd=lambda:command(0.,0.,0.))}):
            spec.loader.exec_module(module)
        module.time=NS(monotonic=lambda:self.now)
        n=self.n=module.Integration.__new__(module.Integration)
        n.lock=threading.RLock();n.points=[NS(x=0.,y=0.),NS(x=1.,y=0.)];n.s=[0.,1.]
        n.command_grace=.2;n.mission_timeout=.5;n.last_path_publish=None
        n.path=NS(header=NS(stamp=None),poses=[])
        n.nominal=(command(),100.)
        n.decision=({'active':False},100.36);n.driver=None
        self.pose()
        for key in ('path_pub','stop_pub','request_pub','cmd_pub','diag'):setattr(n,key,Mock())

    def pose(self):
        self.n.odom=(NS(header=NS(stamp=NS(to_sec=lambda:self.now)),
            pose=NS(pose=NS(position=NS(x=0.,y=0.))),
            twist=NS(twist=NS(linear=NS(x=5.,y=0.)))),self.now)

    def test_actual_mux_continues_then_expires(self):
        self.n.tick(None)
        self.assertGreater(self.n.cmd_pub.publish.call_args.args[0].accel,0.)
        self.assertTrue(json.loads(self.n.diag.publish.call_args.args[0].data)['continuation_active'])
        self.now=100.51;self.pose();self.n.tick(None)
        self.assertEqual(self.n.cmd_pub.publish.call_args.args[0].brake,1.)

    def test_new_stop_and_roundabout_owner_are_immediate(self):
        self.n.nominal=(command(0.,1.),self.now)
        self.n.tick(None)
        self.assertEqual(self.n.cmd_pub.publish.call_args.args[0].brake,1.)
        self.n.nominal=(command(),self.now)
        self.n.decision=({'active':True},self.now)
        self.n.driver=({'active':True,'driver_input_fresh':True,
            'accel_cmd':0.,'brake_cmd':1.,'steering_cmd':0.},self.now)
        self.n.tick(None)
        self.assertEqual(self.n.cmd_pub.publish.call_args.args[0].brake,1.)

    def test_stale_pose_or_unknown_owner_never_holds_accel(self):
        self.n.decision=None;self.n.tick(None)
        self.assertEqual(self.n.cmd_pub.publish.call_args.args[0].brake,1.)
        self.n.decision=({'active':False},self.now)
        self.n.odom=(self.n.odom[0],99.)
        self.n.tick(None)
        self.assertEqual(self.n.cmd_pub.publish.call_args.args[0].brake,1.)

    def test_missing_first_command_does_not_create_acceleration(self):
        self.n.nominal=None
        self.n.tick(None)
        self.assertEqual(self.n.cmd_pub.publish.call_args.args[0].brake,1.)

    def test_path_serialization_is_outside_lock_after_command(self):
        order=[]
        self.n.cmd_pub.publish.side_effect=lambda *_:order.append('command')
        def path(*_):
            self.assertFalse(self.n.lock._is_owned())
            order.append('path')
        self.n.path_pub.publish.side_effect=path
        self.n.tick(None)
        self.assertEqual(order,['command','path'])


def obj(state,score,x=100.):
    return NS(class_name=state,conf=score,x_center=x,y_center=100.,width=30.,height=10.)


class SignalRankingTests(unittest.TestCase):
    def test_same_housing_clear_winner_is_selected(self):
        candidates=[obj('GREEN',.94),obj('RED',.55)]
        result=associate(candidates,{'our_head':(100.,100.)},allow_target_group=True,weighted_group=True)
        self.assertTrue(result.valid)
        self.assertEqual(result.state,'GREEN')

    def test_red_can_win_and_close_scores_stay_unknown(self):
        for red,green,expected in ((.96,.6,'RED'),(.8,.82,'UNKNOWN')):
            result=associate([obj('RED',red),obj('GREEN',green)],{'ours':(100.,100.)},
                allow_target_group=True,weighted_group=True)
            self.assertEqual(result.state,expected)

    def test_correlated_heads_rank_but_other_groups_cannot(self):
        values=[obj('GREEN',.95),obj('RED',.6,180.)]
        targets={'a':(100.,100.),'b':(180.,100.)}
        result=associate(values,targets,allow_target_group=True,allow_correlated_pair=True,weighted_group=True)
        self.assertEqual(result.state,'GREEN')
        result=associate(values,targets,allow_target_group=True,weighted_group=True)
        self.assertFalse(result.valid)

    def test_higher_score_from_another_junction_never_votes(self):
        result=associate([obj('RED',.8),obj('GREEN',.99,400.)],{'ours':(100.,100.)},
            {'other':(400.,100.)},allow_target_group=True,weighted_group=True)
        self.assertEqual(result.state,'RED')
        result=associate([obj('GREEN',.99)],{'ours':(100.,100.)},{'other':(100.,100.)},
            allow_target_group=True,weighted_group=True)
        self.assertFalse(result.valid)

    def test_lit_red_and_arrow_keep_directional_meaning(self):
        result=associate([obj('RED',.95),obj('LEFT',.7,110.)],{'ours':(100.,100.)},
            allow_target_group=True,weighted_group=True)
        self.assertEqual(result.state,'RED_LEFT')


class PixelImage:
    shape=(50,120,3)
    def __init__(self,background=(20,20,20),lamp=None):self.background,self.lamp=background,lamp
    def __getitem__(self,index):
        y,x=index
        return self.lamp if self.lamp and 25<=x<31 and 7<=y<13 else self.background


class VisualSignalTests(unittest.TestCase):
    def test_dark_housing_with_lit_green_is_supported(self):
        valid,_=signal_visual_evidence(PixelImage(lamp=(0,240,0)),(20,5,60,15),'Green')
        self.assertTrue(valid)

    def test_plain_colored_sign_and_wrong_lamp_color_rejected(self):
        for image in (PixelImage(background=(0,240,0)),PixelImage(background=(200,200,200)),
                      PixelImage(lamp=(0,0,240)),PixelImage()):
            self.assertFalse(signal_visual_evidence(image,(20,5,60,15),'Green')[0])

    def test_labels_are_model_classes_not_color_substrings(self):
        for label in ('red_car','green_sign','bright_building','Guardrail','box'):
            self.assertFalse(signal_label_known(label))
        for label in ('Green','Red','red_left','red_yellow','traffic light red'):
            self.assertTrue(signal_label_known(label))


class ReferenceHeartbeatTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0,str(ROOT/'src/experimental/curvature_speed_purepursuit/test'))
        from test_adaptive_node import AdaptiveTest
        self.case=AdaptiveTest();self.case.setUp()
        self.n=self.case.node

    def test_unchanged_geometry_publishes_new_source_stamp_without_replanning(self):
        path=self.case.path()
        self.n._active_path_cb(path)
        self.n._publish_active_path()
        self.n._make_speed_profile=Mock(wraps=self.n._make_speed_profile)
        self.case.now+=.1
        path=self.case.path();stamp=self.case.now
        path.header.stamp=NS(to_sec=lambda:stamp)
        self.n._active_path_cb(path)
        self.case.now+=.2
        self.n._publish_active_path()
        self.n._make_speed_profile.assert_not_called()
        self.assertEqual(self.n.reference_pub.publish.call_args.args[0].header.stamp.to_sec(),stamp)

    def test_path_parser_does_not_hold_control_lock(self):
        path=self.case.path()
        def parser(*args,**kwargs):
            self.assertFalse(self.n._lock._is_owned())
            return path,False,5.,'normal'
        self.case.module.read_trajectory=parser
        self.n._trajectory_cb(NS(data='test'))
        self.assertFalse(self.n.stop_required)

if __name__=='__main__':unittest.main()
