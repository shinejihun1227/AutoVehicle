"""Build-time graph contracts without starting ROS or a vehicle controller."""
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

WS=Path(__file__).resolve().parents[1]


class BuildLaunchContractTest(unittest.TestCase):
    def setUp(self):
        path=WS/'docker/final_ws/check_highway_launch.py'
        spec=importlib.util.spec_from_file_location('build_launch_contract',path)
        self.check=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.check)
        self.calls=[]

    def resolved_inventory(self, command, **kwargs):
        # Representative fully expanded node inventories: no fake ROS nodes run.
        self.calls.append(command)
        if 'final_ws_highway_bringup.launch' in command:
            names=['/adaptive_curvature_purepursuit','/control_mux',
                   '/avoidance_path_manager','/highway_lane_strategy','/lane_camera']
            if 'enable_yolo:=true' in command:names.append('/yolo_camera')
        else:
            names=['/curvature_speed_purepursuit','/curvature_signal_controller',
                   '/highway_lane_camera','/lane_info_contract','/yolo_camera',
                   '/traffic_light_stop','/camera_debug_dashboard']
            if 'enable_roundabout_mission:=true' in command:
                names+=['/morai_lidar_udp_source','/roundabout_lidar_tracking',
                        '/yujin_roundabout_gate']
        return '\n'.join(names)+'\n'

    def test_build_accepts_scoped_lidar_for_monitor_and_drive_and_checks_disabled_mode(self):
        with patch.object(self.check.subprocess,'check_output',side_effect=self.resolved_inventory):
            self.check.main()
        profile=[c for c in self.calls if 'final_ws_curvature_signal.launch' in c]
        combinations={(next(a for a in c if a.startswith('enable_control:=')),
                       next(a for a in c if a.startswith('enable_roundabout_mission:='))) for c in profile}
        self.assertEqual(len(combinations),4)

    def test_build_still_rejects_missing_roundabout_decision_node(self):
        def missing(command,**kwargs):
            return self.resolved_inventory(command,**kwargs).replace('/yujin_roundabout_gate\n','')
        with patch.object(self.check.subprocess,'check_output',side_effect=missing):
            with self.assertRaises(AssertionError):self.check.main()

    def test_build_still_rejects_an_unexpected_second_lidar_receiver(self):
        def duplicate_receiver(command,**kwargs):
            text=self.resolved_inventory(command,**kwargs)
            return text+'/another_lidar_receiver\n' if 'final_ws_curvature_signal.launch' in command else text
        with patch.object(self.check.subprocess,'check_output',side_effect=duplicate_receiver):
            with self.assertRaises(AssertionError):self.check.main()


class LocalImageContractTest(unittest.TestCase):
    def test_start_never_pulls_or_runs_a_missing_project_image(self):
        bash=Path('C:/Program Files/Git/bin/bash.exe')
        executable=str(bash) if bash.exists() else shutil.which('bash')
        if not executable:self.skipTest('bash unavailable')
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);scripts=root/'docker/final_ws';scripts.mkdir(parents=True)
            shutil.copyfile(WS/'docker/final_ws/run_highway.sh',scripts/'run_highway.sh')
            fake=root/'bin';fake.mkdir()
            docker=fake/'docker'
            docker.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$*" >> "$DOCKER_TRACE"\n'
                'case "$*" in\n*" info") exit 0;;\n*) exit 1;;\nesac\n',encoding='utf8')
            docker.chmod(0o755)
            trace=root/'trace.txt'
            env=dict(os.environ,PATH=str(fake)+os.pathsep+os.environ['PATH'],DOCKER_TRACE=str(trace))
            result=subprocess.run([executable,str(scripts/'run_highway.sh'),'start'],env=env,capture_output=True)
            self.assertEqual(result.returncode,2,result.stderr)
            self.assertIn(b'Local image',result.stderr)
            calls=trace.read_text().splitlines()
            self.assertFalse(any(' run ' in c or ' pull ' in c for c in calls),calls)
