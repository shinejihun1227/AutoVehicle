"""Behavioral checks for the actual merged entry points, without ROS I/O."""
import json
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/experimental/curvature_speed_purepursuit/test')]
from test_adaptive_node import AdaptiveTest, Message
from curvature_speed_purepursuit.planner import build_speed_profile, PathPoint, cumulative_arc_lengths
from curvature_speed_purepursuit.lane_centering import lane_quality
from purepursuit_mgeo.route_avoidance_gate import RouteAvoidanceGate, route_fingerprint
from stopline_control.core import StopLineControllerCore
from turn_signal_controller.signal_association import associate


class MergedAdaptiveTest(AdaptiveTest):
    def test_loop_goal_does_not_brake(self):
        self.node.loop_route=True
        self.assertLess(self.tick(x=100.).brake,1.)

    def test_invalid_new_path_cancels_previous_permission(self):
        self.tick(); path=self.path();path.poses=[]
        self.node._active_path_cb(path)
        self.assertFalse(self.node.active_path_received)

    def test_narrow_camera_feedback_is_applied_and_fades_on_lane_change(self):
        self.tick()
        self.node.enable_lane_centering=True
        self.node._measured_speed=lambda *_: (5., None)
        # Actual observed 2.4m lane, center 0.3m left of vehicle.
        info=dict(lane_valid=True,frame_id='base_link',
            coordinate_convention=dict(x='forward_m',y='left_m'),center_source='both',
            lane_width_m=2.4,centerline_points=[[0.,.3],[20.,.3]],
            left_boundary_points=[[0.,1.5],[20.,1.5]],
            right_boundary_points=[[0.,-.9],[20.,-.9]])
        for side in ('left','right'):
            info[side+'_lane']=dict(detected=True,from_guide=False,coasted=False,
                confidence=.95,n_points=30,x_range_m=[0.,20.])
        for _ in range(12):
            self.now+=.1
            self.tick()
            info['timestamp']=self.now
            self.node._lane_change_cb(Message(False))
            self.node._lane_cb(Message(json.dumps(info)))
            steering,status=self.node._lane_correction(0.,5.,self.now,self.now,.1,False)
        self.assertTrue(status['active'],status)
        self.assertGreater(steering,0.)
        self.node._lane_change_cb(Message(True))
        _,status=self.node._lane_correction(0.,5.,self.now,self.now,.1,False)
        self.assertFalse(status['active'])


class FinalContracts(unittest.TestCase):
    def test_stop_target_is_bumper_seven_metres(self):
        core=StopLineControllerCore(front_reference_offset_m=3.845,hold_distance_m=7.)
        core.observe_signal('RED',1.,True,100.,100.,100.)
        core.observe_line(10.845,1.,True,100.,100.,100.,target_id='junction')
        result=core.update(100.,100.,0.)
        self.assertEqual(result.target_speed_kph,0.)
        self.assertGreater(result.brake,0.)
        self.assertAlmostEqual(core.distance,7.)

    def test_curve_boost_is_ten_kph_and_physical_cap_wins(self):
        for curvature in (0.,1/40,1/25,1/5):
            s=[0.,100.,200.];curves=[curvature]*3
            v=build_speed_profile(s,curves,53.2/3.6,.45,1.,1.5,
                final_speed_mps=53.2/3.6,speed_boost_kph=10.,physical_lateral_accel_mps2=2.)[1]
            old=math.sqrt(.45/curvature)*3.6 if curvature else 43.2
            self.assertLessEqual(v*3.6,min(53.2,old+10)+1e-6)
            if curvature:self.assertLessEqual(v*v*curvature,2.+1e-6)

    def test_projection_slack_does_not_allow_other_route_red(self):
        obj=NS(x_center=130.,y_center=100.,width=10.,height=10.,conf=.9,class_name='GREEN')
        result=associate([obj],{'ours':(100.,100.)},allow_target_group=True,gate_px=40.)
        self.assertTrue(result.valid)
        result=associate([obj],{'ours':(100.,100.)},{'other':(130.,100.)},True,gate_px=40.)
        self.assertFalse(result.valid)

    def test_region_window_does_not_jump_to_a_nearby_return_road(self):
        points=[PathPoint(i,0.) for i in range(201)]+[PathPoint(i,3.) for i in range(200,-1,-1)]
        gate=RouteAvoidanceGate(points,[dict(name='dynamic_obstacle',start_xyz=[40,0,0],end_xyz=[80,0,0])])
        self.assertTrue(gate.update(45.,0.)['allowed'])
        self.assertLess(gate.update(46.,3.)['progress_s_m'],100.)
        with self.assertRaises(ValueError):RouteAvoidanceGate(points,[],expected_fingerprint='wrong')

    def test_actual_launch_wires_both_patches_and_seven_metre_goal(self):
        launch=ET.parse(ROOT/'src/bringup/morai_bringup/launch/integrated_obstacle_v61.launch').getroot()
        self.assertEqual(launch.find("arg[@name='stopline_hold_distance_m']").get('default'),'7.0')
        self.assertEqual(launch.find("arg[@name='max_speed_kph']").get('default'),'53.2')
        self.assertEqual(launch.find("arg[@name='nominal_gap_grace_sec']").get('default'),'0.2')
        self.assertEqual(launch.find("arg[@name='yolo_signal_visual_filter']").get('default'),'1')
        camera=ET.parse(ROOT/'src/detection/camera_perception/launch/camera_perception.launch').getroot()
        self.assertIn('--signal-visual-filter $(arg yolo_signal_visual_filter)',
                      camera.find("node[@name='yolo_camera']").get('args'))
        for name in ('frenet_fsm_controller','curvature_signal_controller'):
            self.assertIsNotNone(launch.find("node[@name='%s']/rosparam[@file='$(arg avoidance_config_file)']"%name))
        pp=launch.find("node[@name='curvature_speed_purepursuit']")
        self.assertEqual(pp.find("param[@name='trajectory_topic']").get('value'),'/highway_lane_strategy/trajectory')
        wrapper=(ROOT/'src/control/purepursuit_mgeo/scripts/v61_highway_with_avoidance_node.py').read_text()
        self.assertIn('"highway_lane_strategy_node.py"',wrapper)

    def test_runtime_signal_and_dynamic_configs_construct_together(self):
        import yaml
        sys.path.insert(0,str(ROOT/'src/control/turn_signal_controller/test'))
        from test_maneuver_fusion import FusionNodeTest
        from curvature_speed_purepursuit.planner import load_path_file
        case=FusionNodeTest();case.setUp()
        params=yaml.safe_load((ROOT/'config/curvature_signal.yaml').read_text())
        params.update(yaml.safe_load((ROOT/'src/control/purepursuit_mgeo/config/avoidance_regions.yaml').read_text()))
        case.params.update({'~'+key:value for key,value in params.items()})
        case.params.update({'~path_file':str(ROOT/'data/routes/2026_molit_comp_global_path.txt'),
            '~signal_mgeo_path':str(ROOT/'src/detection/camera_perception/lane/mgeo/R_KR_PR_K-city_2025'),
            '~hold_distance_m':7.,'~require_calibrated_route_camera':True,'~maneuvers':[]})
        case.module.load_path_file=load_path_file
        node=case.module.ManeuverFusionNode()
        self.assertEqual(len(node.route_signal_contexts),5)
        self.assertTrue(node.route_camera_calibrated)
        self.assertIsNotNone(node.dynamic_zone_guard)
        self.assertEqual(node.core.hold_distance_m,7.)
        self.assertEqual(node.signal_pose_pair_tolerance,.15)
        self.assertTrue(node.weighted_signal_group)

    def test_v9_status_has_the_heartbeat_required_by_avoidance(self):
        import ast
        import copy
        import __future__
        from unittest.mock import Mock
        from purepursuit_mgeo.plan_transport import trajectory_payload
        tree=ast.parse((ROOT/'src/control/purepursuit_mgeo/scripts/highway_lane_strategy_node.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='HighwayLaneStrategyNode')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_publish')
        scope=dict(copy=copy,json=json,trajectory_payload=trajectory_payload,
            String=lambda **kw:NS(**kw),Bool=lambda **kw:NS(**kw),Float64=lambda **kw:NS(**kw))
        exec(compile(ast.Module(body=[method],type_ignores=[]),'<actual V9 publish>','exec',flags=__future__.annotations.compiler_flag),scope)
        path=NS(header=NS(frame_id='map',stamp=NS(to_sec=lambda:100.)),
            poses=[NS(pose=NS(position=NS(x=x,y=0.,z=0.))) for x in (0.,1.)])
        node=NS(map_frame='map',trajectory_seq=0,_limit_speed_rate=lambda speed,dt:speed,
            _handoff_permitted=lambda:True,state='OFF',LANE_CHANGE='LANE_CHANGE',lane_changes_done=0)
        for name in ('trajectory_pub','path_pub','stop_pub','speed_pub','active_pub','fast_change_pub','state_pub'):
            setattr(node,name,Mock())
        scope['_publish'](node,path,False,5.,False,{},NS(to_sec=lambda:100.),.05)
        status=json.loads(node.state_pub.publish.call_args.args[0].data)
        self.assertEqual(status['timestamp'],100.)
        self.assertFalse(status['active'])

if __name__=='__main__':unittest.main()
