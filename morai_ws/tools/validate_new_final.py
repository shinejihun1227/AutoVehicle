#!/usr/bin/env python3
"""Isolated offline checks for the final_ws Competition-only migration."""
import ast
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

WS=Path(__file__).resolve().parents[1]
SUITES=[
    'test_new_final',
    'src/localization/morai_udp_drive_bridge/test',
    'src/localization/morai_udp_bridge/test',
    'src/localization/gps_mgeo_converter/test',
    'src/experimental/curvature_speed_purepursuit/test',
    'src/control/stopline_control/test',
    'src/control/turn_signal_controller/test',
    'src/control/purepursuit_mgeo/test',
    'src/detection/lidar_perception/test',
    'src/detection/camera_perception/test',
    'src/experimental/stability_stack/test',
    'src/mission/mission_evaluator/test',
]

def main():
    logs=WS/'test-results/new_final';logs.mkdir(parents=True,exist_ok=True)
    tmp=logs/'tmp';tmp.mkdir(exist_ok=True)
    env=dict(os.environ,TEMP=str(tmp),TMP=str(tmp),TMPDIR=str(tmp),PYTHONUTF8='1')
    env['PYTHONPATH']=os.pathsep.join([*(str(p) for p in (WS/'src').glob('*/*/src')),
                                     os.environ.get('PYTHONPATH','')])
    errors=[]
    for p in (WS/'src').rglob('*.py'):
        try:ast.parse(p.read_text(encoding='utf-8-sig'),feature_version=(3,8))
        except (SyntaxError,UnicodeError) as exc:errors.append([str(p),str(exc)])
    for p in (WS/'src').rglob('*.launch'):ET.parse(p)
    results=[]
    for suite in SUITES:
        r=subprocess.run([sys.executable,'-X','utf8','-m','unittest','discover','-s',suite,'-v'],
                         cwd=WS,env=env,capture_output=True)
        output=(r.stdout+r.stderr).decode('utf8',errors='replace')
        (logs/(suite.replace('/','_')+'.log')).write_text(output,encoding='utf8')
        count=re.search(r'Ran (\d+) tests?',output)
        result=dict(suite=suite,exit_code=r.returncode,tests=int(count[1]) if count else 0)
        results.append(result);print(result,flush=True)
        if r.returncode:
            print('\n'.join(line for line in output.splitlines() if line.startswith(('FAIL:','ERROR:'))))
    report=dict(python38_syntax_errors=errors,suites=results,
                total_tests=sum(r['tests'] for r in results),ros_build=False,morai_driving=False,
                live_competition_packet_verified=False)
    (logs/'summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8',newline='\n')
    return int(bool(errors) or any(r['exit_code'] for r in results))

if __name__=='__main__':sys.exit(main())
