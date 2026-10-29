import ast
import importlib.util
import json
import math
from pathlib import Path
import sys
import shutil
import subprocess
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET
import test_migration as migration
from purepursuit_mgeo.roundabout_contract import RoundaboutScope

WS=migration.WS


class LaunchAndInputContractTest(unittest.TestCase):
    def test_original_competition_bridge_ekf_and_final_signal_controller_are_byte_identical(self):
        # Pinned integration baseline, canonical text independent of checkout OS.
        import hashlib
        evidence=json.loads((WS.parent/'JI_UJIN_PROVENANCE.json').read_text())
        for item in evidence['preserved_main_files']:
            actual=hashlib.sha256((WS.parent/item['path']).read_bytes().replace(b'\r\n',b'\n')).hexdigest()
            self.assertEqual(actual,item['sha256'],item['path'])

    def test_launch_uses_existing_output_chain_and_enables_scoped_extensions(self):
        root=ET.parse(WS/'src/bringup/morai_bringup/launch/final_ws_curvature_signal.launch').getroot()
        pp=root.find("node[@type='curvature_speed_purepursuit_node.py']")
        self.assertEqual(pp.find("param[@name='command_topic']").get('value'),'/control/ctrl_cmd')
        self.assertEqual(pp.find("param[@name='enable_roundabout_mission']").get('value'),'$(arg enable_roundabout_mission)')
        self.assertEqual(pp.find("param[@name='enable_tunnel_lane_priority']").get('value'),'$(arg enable_tunnel_lane_priority)')
        self.assertIsNotNone(root.find("node[@type='yujin_roundabout_gate_node.py']"))
        base=root.find('include')
        self.assertEqual(base.find("arg[@name='ekf_competition_speed_topic']").get('value'),'$(arg vehicle_speed_topic)')

    def test_vendor_runtime_matches_pinned_upstream_snapshot(self):
        import hashlib
        evidence=json.loads((WS.parent/'JI_UJIN_PROVENANCE.json').read_text())
        directory=WS/'src/control/purepursuit_mgeo/src/purepursuit_mgeo/yujin_roundabout'
        self.assertEqual(set(evidence['upstream']['installed_sha256']),{'__init__.py','core.py','entry_decision.py','prediction.py'})
        for name,digest in evidence['upstream']['installed_sha256'].items():
            self.assertEqual(hashlib.sha256((directory/name).read_bytes().replace(b'\r\n',b'\n')).hexdigest(),digest)

    def test_host_preparation_preserves_user_network_and_caps_without_starting_control(self):
        bash=Path('C:/Program Files/Git/bin/bash.exe')
        executable=str(bash) if bash.exists() else shutil.which('bash')
        if not executable:self.skipTest('bash unavailable')
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);new=root/'new';old=root/'old';new.mkdir();old.mkdir()
            source=WS/'docker/final_ws'
            for name in ('prepare_ji_ujin.sh','highway.env.example','highway-test.env.example'):
                shutil.copyfile(source/name,new/name)
            (old/'highway.env').write_text('UBUNTU_IP=192.168.10.21\nMORAI_IP=192.168.10.14\nCONTAINER_NAME=original\nTORCH_FLAVOR=cu121\n',newline='\n')
            saved='COMPETITION_STATUS_PORT=9099\nMAX_SPEED_KPH=35\nSTOPLINE_HOLD_DISTANCE_M=10\nLANE_INFO_DEVICE=cuda\nLIDAR_X_M=0.0\n'
            (old/'highway-test.env').write_text(saved,newline='\n')
            result=subprocess.run([executable,str(new/'prepare_ji_ujin.sh'),str(old),'--lidar-bdt'],capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            network=(new/'highway.env').read_text();config=(new/'highway-test.env').read_text()
            for text in ('COMPETITION_STATUS_PORT=9099','MAX_SPEED_KPH=35','STOPLINE_HOLD_DISTANCE_M=10','LIDAR_X_M=1.22','ROUNDABOUT_ENABLED=true','LANE_INFO_DEVICE=auto'):
                self.assertIn(text,config)
            self.assertIn('TORCH_FLAVOR=cu121',network)
            self.assertIn('CONTAINER_NAME=morai-ji-ujin-net10',network)
            self.assertEqual((old/'highway-test.env').read_text(),saved)


class RealMainMissionTest(unittest.TestCase):
    def setUp(self):
        case=migration.control_fixture.StartupTest();case.setUp();self.case=case
        from curvature_speed_purepursuit.planner import load_path_file
        self.points=load_path_file(WS/'data/routes/2026_molit_comp_global_path.txt')
        case.points=self.points
        case.params.update({'~enable_roundabout_mission':True,
            '~path_file':str(WS/'data/routes/2026_molit_comp_global_path.txt'),
            '~roundabout_config_file':str(WS/'config/roundabout_v61.yaml'),
            '~roundabout_circulating_path_file':str(WS/'data/routes/roundabout_circulating_path.txt')})
        case.node=case.module.CurvatureSpeedPurePursuitNode()

    def test_stale_gate_brakes_inside_region_without_reporting_goal(self):
        p=self.points[1740];cmd=self.case.tick(x=p.x,y=p.y)
        self.assertEqual((cmd.accel,cmd.brake),(0.,1.))
        self.assertFalse(self.case.node.goal_pub.publish.call_args.args[0].data)

    def test_fresh_gate_caps_actual_pid_target_and_cannot_raise_main_speed(self):
        c=self.case;p=self.points[1740]
        for i in range(40):
            status=dict(timestamp=c.now,active=True,target_speed=2.,stop_required=False,
                emergency=False,decision_reason='yield_approach',
                main_path_sha256=c.node.mission_gate.scope.path_sha)
            c.node.roundabout_callback(NS(data=json.dumps(status)))
            cmd=c.tick(x=p.x,y=p.y)
            self.assertLessEqual(c.node.speed_command_pub.publish.call_args.args[0].data,7.2+1e-6)
        self.assertGreater(cmd.accel,0.)


class RoundaboutAdapterTest(unittest.TestCase):
    def setUp(self):
        self.now=100.;self.ros=Mock()
        cfg={'~config_file':str(WS/'config/roundabout_v61.yaml'),
             '~path_file':str(WS/'data/routes/2026_molit_comp_global_path.txt'),
             '~circulating_file':str(WS/'data/routes/roundabout_circulating_path.txt')}
        self.ros.get_param.side_effect=lambda key,default=None:cfg.get(key,default)
        self.ros.get_time.side_effect=lambda:self.now
        class Text:
            def __init__(self,data):self.data=data
        modules={'rospy':self.ros,'nav_msgs.msg':NS(Odometry=object),
                 'std_msgs.msg':NS(String=Text),'morai_msgs.msg':NS(EgoVehicleStatus=object),
                 'lidar_perception.msg':NS(LidarObstacleArray=object)}
        path=WS/'src/control/purepursuit_mgeo/scripts/yujin_roundabout_gate_node.py'
        spec=importlib.util.spec_from_file_location('adapter_test',path)
        self.module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules):
            spec.loader.exec_module(self.module)
            self.module.time=NS(monotonic=lambda:self.now)
            self.node=self.module.RoundaboutGateNode()

    def frame(self, include_tracks=True):
        node=self.node;i=node.scope.config['entry_decision_stop_index'];p,q=node.scope.path[i:i+2]
        yaw=math.atan2(q.y-p.y,q.x-p.x)
        header=NS(stamp=NS(to_sec=lambda:self.now),frame_id='map')
        node.odom_callback(NS(header=header,pose=NS(pose=NS(position=NS(x=p.x,y=p.y),
            orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))))))
        node.speed_callback(NS(header=header,velocity=NS(x=0.,y=float('nan'),z=float('nan'))))
        if include_tracks:node.tracks_callback(NS(header=header,obstacle_count=0,obstacles=[]))
        node.tick(None)
        data=json.loads(node.pub.publish.call_args.args[0].data)
        self.now+=.05
        return data

    def test_real_adapter_uses_competition_x_and_publishes_only_decisions(self):
        for _ in range(15):state=self.frame()
        self.assertTrue(state['allowed'])
        topics=[c.args[0] for c in self.ros.Publisher.call_args_list]
        self.assertEqual(topics,['/roundabout_v61/status'])

    def test_missing_lidar_is_not_empty_and_invalidates_entry(self):
        state=self.frame(False)
        self.assertTrue(state['stop_required'])
        self.assertEqual(state['decision_reason'],'obstacles_stale_or_invalid')
