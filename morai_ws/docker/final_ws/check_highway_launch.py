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


if __name__=='__main__': main()
