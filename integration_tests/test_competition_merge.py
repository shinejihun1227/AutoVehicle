"""Exercise the new composition, including unchanged-geometry zone transitions."""
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/experimental/curvature_speed_purepursuit/test'),
              str(ROOT/'src/control/purepursuit_mgeo/test')]
import test_adaptive_node as adaptive
import test_highway_overtake_node as highway
from purepursuit_mgeo import highway_zone


def wrapper(script, team):
    spec=importlib.util.spec_from_file_location('_merged_'+script,
        ROOT/'src/control/purepursuit_mgeo/scripts'/script)
    module=importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {'rospy':team.rospy,'std_msgs.msg':NS(Bool=adaptive.Message)}), \
            patch.object(highway_zone,'load_script',return_value=team):
        spec.loader.exec_module(module)
    return module


class IntegratedCompetitionLaunchTests(unittest.TestCase):
    def test_single_chain_and_atomic_plan_inputs(self):
        r=ET.parse(ROOT/'src/bringup/morai_bringup/launch/integrated_obstacle_v61.launch').getroot()
        self.assertEqual(r.find("arg[@name='ego_status_port']").get('default'),'9081')
        self.assertEqual(r.find("arg[@name='status_protocol']").get('default'),'competition')
        self.assertEqual(r.find("arg[@name='stopline_hold_distance_m']").get('default'),'7.0')
        self.assertEqual(r.find("arg[@name='max_speed_kph']").get('default'),'53.2')
        self.assertLess(float(r.find("arg[@name='highway_max_speed_kph']").get('default')),60.)
        for name,kind in [('curvature_speed_purepursuit','curvature_pp_overtake_node.py'),
                          ('curvature_signal_controller','maneuver_fusion_overtake_node.py')]:
            n=r.find("node[@name='%s']"%name)
            self.assertEqual((n.get('pkg'),n.get('type')),('purepursuit_mgeo',kind))
        for name,topic in [('highway_lane_strategy','/integration/avoidance/trajectory'),
                           ('curvature_speed_purepursuit','/highway_lane_strategy/trajectory')]:
            n=r.find("node[@name='%s']"%name)
            key='base_trajectory_topic' if name.startswith('highway') else 'trajectory_topic'
            self.assertEqual(n.find("param[@name='%s']"%key).get('value'),topic)
        self.assertEqual(len(r.findall("node[@type='no_obstacle_integration_node.py']")),1)
        cmake=(ROOT/'src/control/purepursuit_mgeo/CMakeLists.txt').read_text()
        for name in ('highway_overtake_node.py','highway_overtake_escape_node.py',
                     'curvature_pp_overtake_node.py','maneuver_fusion_overtake_node.py'):
            self.assertIn('scripts/'+name,cmake)

    def test_competition_mode_and_port_reach_udp_receiver(self):
        for f in ('morai_udp_ekf_purepursuit.launch','integrated_no_obstacle_v61.launch'):
            r=ET.parse(ROOT/'src/bringup/morai_bringup/launch'/f).getroot()
            self.assertEqual(r.find("arg[@name='ego_status_port']").get('default'),'9081')
            self.assertIsNotNone(r.find(".//include/arg[@name='status_protocol']"))
        r=ET.parse(ROOT/'src/localization/morai_udp_drive_bridge/launch/morai_udp_drive_bridge.launch')
        self.assertEqual(r.find("arg[@name='status_port']").get('default'),'9081')
        self.assertEqual(r.find("node/param[@name='status_protocol']").get('value'),'$(arg status_protocol)')


class ZoneAndSpeedIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.case=adaptive.AdaptiveTest();self.case.setUp()

    def test_identical_path_raises_and_restores_profile_on_zone_change(self):
        case=self.case
        mod=wrapper('curvature_pp_overtake_node.py',case.module)
        node=mod.HighwayCurvaturePurePursuit()
        node.highway_max_speed_kph=59.;node.highway_lateral_accel_mps2=2.
        case.node=node
        with patch.object(highway_zone.time,'monotonic',lambda:case.now):
            case.tick(speed_limit=59./3.6)
            self.assertAlmostEqual(node.active_speed_profile[-1]*3.6,30.)
            node.highway_zone.on_msg(NS(data=True))
            case.tick(speed_limit=59./3.6)
            self.assertAlmostEqual(node.active_speed_profile[-1]*3.6,59.)
            node.highway_zone.on_msg(NS(data=False))
            # No new path callback: leaving the zone still removes the raised cap.
            case.now+=.05;node._control_cb(None)
            self.assertAlmostEqual(node.active_speed_profile[-1]*3.6,30.)
            node.highway_zone.on_msg(NS(data=True))
            case.tick(speed_limit=59./3.6)
            case.now+=.6;node._control_cb(None)
            self.assertAlmostEqual(node.active_speed_profile[-1]*3.6,30.)
            self.assertEqual(node.command_pub.publish.call_args.args[0].brake,1.)

    def test_longitudinal_status_speed_survives_unavailable_yz(self):
        case=self.case
        msg=NS(header=NS(stamp=NS(to_sec=lambda:case.now)),
               velocity=NS(x=10.,y=float('nan'),z=float('nan')))
        case.node._vehicle_speed_cb(msg)
        self.assertEqual(case.node.vehicle_speed_source.sample(case.now,case.now),10.)


class NewHighwayMissionInterfaceTests(unittest.TestCase):
    def setUp(self):
        self.h=highway.OvertakeNodeTest();self.h.setUp()

    def test_highway_state_is_readable_by_wait5_ownership(self):
        self.h.drive(self.h.zone[0]+5.,4)
        state=self.h.state()
        self.assertEqual(state['state'],'LANE_CHANGE')
        self.assertTrue(state['active'])
        self.assertFalse(state['stop'])
        self.assertAlmostEqual(state['timestamp'],self.h.clock.now)

    def test_stale_obstacle_header_does_not_renew_lidar_health(self):
        node=self.h.node
        good=highway.Msg(obstacles=[],header=NS(stamp=highway.Stamp(self.h.clock.now),frame_id='map'))
        node._obstacles_cb(good)
        sample=node.planner.obstacles
        self.h.clock.now+=1.
        node._obstacles_cb(good)
        self.assertIs(node.planner.obstacles,sample)
