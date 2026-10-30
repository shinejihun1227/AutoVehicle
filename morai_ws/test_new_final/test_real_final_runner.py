"""Execute the Ubuntu wrapper's setup/tuning/show paths without Docker."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

REPO=Path(__file__).resolve().parents[2]


class RealFinalRunner(unittest.TestCase):
    def test_setup_tune_show_preserve_personal_ports_and_stop_clearance(self):
        bash=Path('C:/Program Files/Git/bin/bash.exe')
        bash=str(bash) if bash.exists() else shutil.which('bash')
        if not bash:self.skipTest('Bash unavailable')
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); scripts=root/'morai_ws/docker/final_ws'
            scripts.mkdir(parents=True)
            shutil.copyfile(REPO/'RUN_REAL_FINAL.sh',root/'RUN_REAL_FINAL.sh')
            for name in ('run_test.sh','curvature_runtime_files.sh','highway.env.example','highway-test.env.example'):
                shutil.copyfile(REPO/'morai_ws/docker/final_ws'/name,scripts/name)
            def run(action):
                r=subprocess.run([bash,str(root/'RUN_REAL_FINAL.sh'),action],capture_output=True)
                self.assertEqual(r.returncode,0,r.stderr)
                return r.stdout.decode()
            run('setup')
            env=scripts/'highway-test.env'
            content=env.read_text().replace('COMPETITION_STATUS_PORT=9081','COMPETITION_STATUS_PORT=9099')
            content=content.replace('STOPLINE_HOLD_DISTANCE_M=0.5','STOPLINE_HOLD_DISTANCE_M=10.0')
            content=content.replace('HIGHWAY_ENABLED=true','HIGHWAY_ENABLED=false')
            env.write_text(content)
            run('tune')
            saved=env.read_text()
            for preserved in ('COMPETITION_STATUS_PORT=9099','STOPLINE_HOLD_DISTANCE_M=10.0','HIGHWAY_ENABLED=false'):
                self.assertIn(preserved,saved)
            shown=run('show')
            self.assertIn('max_speed_kph=42.0',shown)
            self.assertIn('lane_centering_weight:=0.55',shown)
            self.assertIn('lane_info_device:=auto',shown)
