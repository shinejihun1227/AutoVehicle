"""Exercise ROI's real stabilizer with synthetic detector results (no model/GPU)."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np


class LaneInfoIntegrationTest(unittest.TestCase):
    def test_current_detector_geometry_supports_fresh_held_and_invalid_outputs(self):
        path = Path(__file__).resolve().parents[1] / 'lane/live_lane_info_publisher_v2.py'
        spec = importlib.util.spec_from_file_location('_real_roi_stabilizer', path)
        module = importlib.util.module_from_spec(spec)
        modules = {'rospy': Mock(), 'std_msgs.msg': NS(String=Mock()),
                   'lane_detection': NS(LaneDetector=Mock(), default_checkpoint=lambda: '',
                                        EVAL_X_NEAR=7., EVAL_X_FAR=14., LANE_WIDTH_M=3.3),
                   'morai_camera': NS(DEFAULT_IP='0.0.0.0', DEFAULT_PORT=1101, CameraStream=Mock())}
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)

        def lane(y, lane_id):
            return NS(coef=np.array([0., 0., y]), x_range=(3., 30.), age=5, n_points=100,
                      name='white_dash', is_dashed=True, track_id=1, lane_id=lane_id)

        left, right = lane(1.65, -1), lane(-1.65, 1)
        res = NS(ego_left=left, ego_right=right, lanes=[left, right], infer_ms=20., post_ms=5.,
                 lateral_error=lambda: 0., heading_error=lambda: 0., stopline_dist=12.)
        stabilizer = module.LaneOutputStabilizer()
        fresh = stabilizer.update(res, now=100.)
        self.assertEqual(fresh['output_status'], 'FRESH')
        self.assertEqual(fresh['frame_id'], 'base_link')
        self.assertEqual(fresh['lane_width_m'], 3.3)
        self.assertGreaterEqual(len(fresh['centerline_points']), 3)
        missing = NS(ego_left=None, ego_right=None, lanes=[], infer_ms=20., post_ms=5.,
                     lateral_error=lambda: None, heading_error=lambda: None, stopline_dist=None)
        held = stabilizer.update(missing, now=100.01)
        self.assertEqual(held['output_status'], 'HELD')
        self.assertEqual(stabilizer.last_good_t, 100.)
        invalid = stabilizer.update(missing, now=110.)
        self.assertFalse(invalid['lane_valid'])
        self.assertEqual(invalid['centerline_points'], [])

