#!/usr/bin/env python3
"""Run ROS-free regression suites in isolated processes and keep full logs."""
import ast
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
SUITES=[
 'integration_tests/wait5_complete',
 'src/localization/morai_udp_drive_bridge/test',
 ('src/control/purepursuit_mgeo/test','test_highway_overtake*.py'),
 'integration_tests/patches',
 'integration_tests',
 'src/experimental/curvature_speed_purepursuit/test',
 'src/control/stopline_control/test',
 'src/control/turn_signal_controller/test',
 ('src/detection/camera_perception/test','test_camera_timestamps.py'),
 ('src/detection/camera_perception/test','test_traffic_signal.py'),
]

def main():
    logs=ROOT/'test-results'
    logs.mkdir(exist_ok=True)
    temp=logs/'tmp';temp.mkdir(exist_ok=True)
    env=dict(os.environ, PYTHONUTF8='1', TMP=str(temp),TEMP=str(temp),TMPDIR=str(temp))
    env['PYTHONPATH']=os.pathsep.join([*(str(p) for p in (ROOT/'src').glob('*/*/src')),
                                     os.environ.get('PYTHONPATH','')])
    syntax=[]
    for p in (ROOT/'src').rglob('*.py'):
        try:ast.parse(p.read_text(encoding='utf-8-sig'))
        except (SyntaxError,UnicodeError) as e:syntax.append([str(p.relative_to(ROOT)),str(e)])
    for p in (ROOT/'src').rglob('*.launch'):ET.parse(p)
    results=[]
    for suite in SUITES:
        folder,pattern=(suite,'test*.py') if isinstance(suite,str) else suite
        label=folder if isinstance(suite,str) else folder+':'+pattern
        result=subprocess.run([sys.executable,'-X','utf8','-m','unittest','discover','-s',folder,'-p',pattern,'-v'],
            cwd=ROOT,env=env,capture_output=True)
        text=(result.stdout+result.stderr).decode('utf-8',errors='replace')
        (logs/(label.replace('/','_').replace(':','_').replace('*','all')+'.log')).write_text(text,encoding='utf-8')
        import re
        ran=re.search(r'Ran (\d+) tests?',text)
        summary=dict(suite=label,exit_code=result.returncode,tests=int(ran[1]) if ran else 0)
        results.append(summary);print(summary,flush=True)
        if result.returncode:
            print('\n'.join(line for line in text.splitlines() if line.startswith(('ERROR:','FAIL:'))))
    report=dict(syntax_errors=syntax,suites=results,ros_build=False,morai_driving=False,
                preview_and_http_verified=False)
    (logs/'summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return int(bool(syntax) or any(r['exit_code'] for r in results))

if __name__=='__main__':sys.exit(main())
