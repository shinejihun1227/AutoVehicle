"""Host viewer scripts with Docker/browser/Xauthority stubs; no GUI or ROS."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent
BASH = 'C:/Program Files/Git/bin/bash.exe' if os.name == 'nt' else shutil.which('bash')


def shell_path(path):
    value = str(path).replace('\\', '/')
    return '/' + value[0].lower() + value[2:] if len(value) > 1 and value[1] == ':' else value


@unittest.skipUnless(BASH and Path(BASH).exists(), 'bash required')
class ViewerTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='camera-viewer-')
        self.addCleanup(self.tmp.cleanup)
        self.task = Path(self.tmp.name)
        self.bin = self.task/'bin'
        self.bin.mkdir()
        (self.task/'socket').mkdir()
        for name in ('open_camera_rviz.sh', 'open_camera_dashboard.sh'):
            source = (ROOT/name).read_text(encoding='utf-8')
            # Only the unavailable test-machine X socket location is substituted.
            source = source.replace('/tmp/.X11-unix', shell_path(self.task/'socket'))
            (self.task/name).write_text(source, encoding='utf-8', newline='\n')
        (self.task/'highway.env').write_text('CONTAINER_NAME=test-drive\nUBUNTU_IP=192.168.0.185\n', newline='\n')
        shutil.copyfile(ROOT/'cameras.rviz', self.task/'cameras.rviz')
        self.env = dict(os.environ, DISPLAY=':0', TEST_RUNNING='true', TEST_COOKIE='true',
                        TEST_CURL='0', TEST_BROWSER='0', TEST_LOG=shell_path(self.task/'calls'))
        self.stub('docker', '''shift 2
case "$1" in
 info) exit 0 ;;
 inspect)
  if [[ "$3" == '{{.Image}}' ]]; then echo sha256:testimage; else echo "$TEST_RUNNING"; fi ;;
 exec) echo http://192.168.0.185:11311 ;;
 run) printf '%s\\n' "$@" >> "$TEST_LOG" ;;
 *) exit 3 ;;
esac
''')
        self.stub('xauth', '''if [[ "$1" == nlist ]]; then
  if [[ "$TEST_COOKIE" == true ]]; then echo 0100FAKE_TEST_COOKIE; fi
else
  cat >/dev/null
fi
''')
        self.stub('curl', 'exit "$TEST_CURL"\n')
        self.stub('xdg-open', 'echo "BROWSER $*" >> "$TEST_LOG"; exit "$TEST_BROWSER"\n')

    def stub(self, name, source):
        path = self.bin/name
        path.write_text('#!/usr/bin/env bash\n' + source, newline='\n')
        path.chmod(0o755)

    def run_script(self, name):
        # Positional arguments avoid quoting/substitution of fixture paths.
        return subprocess.run([BASH, '-c', 'export PATH="$1:/usr/bin:/bin:$PATH"; bash "$2"',
                               'viewer-test', shell_path(self.bin), shell_path(self.task/name)],
                              env=self.env, capture_output=True, text=True, encoding='utf-8',
                              errors='replace', timeout=20)

    def test_rviz_is_separate_and_has_no_drive_launch_or_broad_xhost_grant(self):
        result = self.run_script('open_camera_rviz.sh')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = (self.task/'calls').read_text().splitlines()
        self.assertIn('--rm', args)
        self.assertIn('sha256:testimage', args)
        self.assertIn('ROS_MASTER_URI=http://192.168.0.185:11311', args)
        self.assertIn('ROS_IP=192.168.0.185', args)
        self.assertIn('source /opt/ros/noetic/setup.bash && exec rosrun rviz rviz -d /tmp/cameras.rviz', args)
        self.assertTrue(all('roslaunch' not in arg and 'xhost' not in arg for arg in args))
        authority = next(arg for arg in args if 'dst=/tmp/camera-viewer.xauth' in arg)
        self.assertTrue(authority.endswith(',readonly'))

    def test_no_gui_no_cookie_or_stopped_container_never_starts_viewer(self):
        for key, value in [('DISPLAY', ''), ('TEST_COOKIE', 'false'), ('TEST_RUNNING', 'false')]:
            with self.subTest(key=key):
                previous = self.env[key]
                self.env[key] = value
                result = self.run_script('open_camera_rviz.sh')
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse((self.task/'calls').exists())
                self.env[key] = previous

    def test_dashboard_unavailable_is_reported_without_opening_a_browser(self):
        self.env['TEST_CURL'] = '7'
        result = self.run_script('open_camera_dashboard.sh')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('not responding', result.stderr)
        self.assertFalse((self.task/'calls').exists())

    @unittest.skipIf(hasattr(os, 'geteuid') and os.geteuid() == 0, 'browser intentionally refuses root')
    def test_browser_failure_is_visible_and_success_is_reported(self):
        self.env['TEST_BROWSER'] = '1'
        result = self.run_script('open_camera_dashboard.sh')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Browser could not be opened', result.stderr)
        self.env['TEST_BROWSER'] = '0'
        result = self.run_script('open_camera_dashboard.sh')
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
