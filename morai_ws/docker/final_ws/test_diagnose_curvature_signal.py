"""Diagnostic formatting stays compact without obscuring missing inputs."""
import importlib.util
from contextlib import redirect_stdout
import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location('diagnose_signal', Path(__file__).with_name('diagnose_curvature_signal.py'))
diagnostic = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnostic)


class DiagnosticTest(unittest.TestCase):
    def test_diagnosis_completes_without_inputs_and_never_publishes(self):
        now = [0.]
        def advance(seconds):
            now[0] += seconds
        subscribers = []
        def subscribe(*args, **kwargs):
            sub = Mock()
            subscribers.append(sub)
            return sub
        ros = NS(init_node=Mock(), get_param=lambda key, default=None: default,
                 is_shutdown=lambda: False, AnyMsg=object, Subscriber=subscribe,
                 Time=NS(now=lambda: NS(to_sec=lambda: 100.)),
                 get_master=lambda: NS(getSystemState=lambda: (1, '', ([], [], []))))
        modules = {'rospy': ros, 'rospkg': NS(RosPack=Mock(side_effect=RuntimeError('missing package'))),
                   'morai_msgs.msg': NS(CtrlCmd=object),
                   'nav_msgs.msg': NS(Odometry=object, Path=object), 'std_msgs.msg': NS(String=object)}
        output = io.StringIO()
        with patch.dict(sys.modules, modules), patch.object(diagnostic, 'recent_errors', return_value=[]), \
             patch.object(diagnostic, 'time', NS(monotonic=lambda: now[0], sleep=advance)), redirect_stdout(output):
            diagnostic.main()
        self.assertEqual(output.getvalue().count('count=0 age=NONE pub=NONE'), 7)
        self.assertIn('CURVATURE_SOURCE_ERROR missing package', output.getvalue())
        self.assertIn('DIAGNOSIS_DONE', output.getvalue())
        for sub in subscribers:
            sub.unregister.assert_called_once()
        # ros exposes neither Publisher nor set_param; an attempted write would fail.

    def test_all_stop_reasons_survive_the_summary(self):
        msg = NS(data='{"mode":"SAFE_STOP","reason":"reference_path_not_received,nominal_stale_or_not_type1",'
                 '"reference_path_match":false,"progress_s_m":null,"accel":0,"brake":1,"private_extra":42}')
        result = diagnostic.summarize('status', msg, 0)
        self.assertIn('reference_path_not_received', result['reason'])
        self.assertIn('nominal_stale_or_not_type1', result['reason'])
        self.assertIsNone(result['progress_s_m'])
        self.assertNotIn('private_extra', result)

    def test_wrong_command_type_is_visible(self):
        self.assertEqual(diagnostic.summarize('nominal', NS(longlCmdType=2, accel=0., brake=1.), 0),
                         dict(type=2, accel=0., brake=1.))

    def test_stale_pose_frame_and_age_are_preserved(self):
        msg = NS(header=NS(frame_id='odom', stamp=NS(to_sec=lambda: 95)),
                 pose=NS(pose=NS(position=NS(x=4.5, y=7.))),
                 twist=NS(twist=NS(linear=NS(x=3., y=4.))))
        result = diagnostic.summarize('odom', msg, 100)
        self.assertEqual((result['frame'], result['source_age_s'], result['speed_kph']), ('odom', 5., 18.))

    def test_latched_path_is_not_mistaken_for_stale_odometry(self):
        result = diagnostic.summarize('reference', NS(header=NS(frame_id='map'), poses=[1, 2]), 100)
        self.assertEqual(result, dict(frame='map', points=2))

    def test_log_tail_is_bounded_and_includes_start_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'roslaunch-test.log'
            path.write_text('old Error: outside tail\n' + 'INFO ok\n'*12000 +
                            'ImportError: missing planner\nprocess has died: curvature\n')
            lines = list(diagnostic.recent_errors(directory))
            self.assertEqual(len(lines), 2)
            self.assertIn('ImportError', lines[0])
            self.assertIn('process has died', lines[1])
        self.assertEqual(list(diagnostic.recent_errors(directory)), [])


if __name__ == '__main__':
    unittest.main()
