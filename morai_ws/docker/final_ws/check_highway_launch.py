#!/usr/bin/env python3
"""Resolve complete launch includes with real ROS; no sensors/control started."""
import os
from pathlib import Path
import subprocess


def main():
    ws=Path(__file__).resolve().parents[2]
    env=dict(os.environ, MORAI_WS=str(ws))
    for control,yolo in (('false','true'),('true','true'),('false','false')):
        output=subprocess.check_output(['roslaunch','--nodes','morai_bringup',
            'final_ws_highway_bringup.launch','workspace_path:='+str(ws),
            'enable_control:='+control,'enable_yolo:='+yolo],env=env,text=True)
        names=output.splitlines()
        assert len(names)==len(set(names)), output
        for required in ('/adaptive_curvature_purepursuit','/control_mux',
                         '/highway_lane_camera','/lane_info_contract',
                         '/avoidance_path_manager','/highway_lane_strategy'):
            assert required in names, (required,output)
        assert '/purepursuit_mgeo' not in names and '/curvature_speed_purepursuit' not in names
    print('HIGHWAY_LAUNCH_OK: ROS resolved all includes; control was not started')
    for control in ('false', 'true'):
        output=subprocess.check_output(['roslaunch','--nodes','morai_bringup',
            'final_ws_curvature_signal.launch','workspace_path:='+str(ws),
            'enable_control:='+control],env=env,text=True)
        names=output.splitlines()
        assert len(names)==len(set(names)), output
        for required in ('/curvature_speed_purepursuit','/curvature_signal_controller',
                         '/highway_lane_camera','/lane_info_contract',
                         '/yolo_camera','/traffic_light_stop'):
            assert required in names, (required,output)
        for excluded in ('/control_mux','/final_ws_stopline_controller',
                         '/avoidance_path_manager','/bypass_lane_guard',
                         '/avoidance_frenet_debug','/highway_lane_strategy',
                         '/roi_sensor_safety_adapter','/intersection_environment',
                         '/pedestrian_crossing_fusion','/highway_environment_gate'):
            assert excluded not in names, (excluded,output)
        assert not any('lidar' in name.lower() for name in names), output
    print('CURVATURE_SIGNAL_LAUNCH_OK: ROS resolved monitor/drive; control was not started')


if __name__=='__main__': main()
