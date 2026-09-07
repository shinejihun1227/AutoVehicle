#!/usr/bin/env python3
"""ROS가 설치되지 않은 개발 PC에서도 실제 controller core를 검증한다."""

import importlib.util
import os
import sys
import types
import unittest


def _stub_module(name, **attributes):
    module = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


_stub_module("rospy")
morai_msgs = _stub_module("morai_msgs")
morai_msgs.msg = _stub_module("morai_msgs.msg", CtrlCmd=object)
perception_msgs = _stub_module("morai_perception_msgs")
perception_msgs.msg = _stub_module(
    "morai_perception_msgs.msg", StopLineDetection=object, TrafficLight=object
)
nav_msgs = _stub_module("nav_msgs")
nav_msgs.msg = _stub_module("nav_msgs.msg", Odometry=object)
std_msgs = _stub_module("std_msgs")
std_msgs.msg = _stub_module("std_msgs.msg", Bool=object, String=object)

SCRIPT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts", "stopline_controller.py")
)
spec = importlib.util.spec_from_file_location("stopline_controller_under_test", SCRIPT)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


class StopLineCoreTest(unittest.TestCase):
    def setUp(self):
        self.core = module.StopLineControllerCore()

    def test_signal_clear_passes_nominal(self):
        decision = self.core.decide(
            signal_stop_required=False,
            stopline_valid=True,
            stopline_distance_m=0.3,
            stopline_confidence=1.0,
            speed_mps=2.0,
        )
        self.assertEqual(decision.mode, "nominal")
        self.assertEqual(decision.brake, 0.0)

    def test_close_stopline_holds_full_brake(self):
        decision = self.core.decide(
            signal_stop_required=True,
            stopline_valid=True,
            stopline_distance_m=0.4,
            stopline_confidence=1.0,
            speed_mps=2.0,
        )
        self.assertEqual(decision.mode, "hold_stop")
        self.assertEqual(decision.brake, 1.0)
        self.assertEqual(decision.accel, 0.0)

    def test_fast_approach_increases_brake(self):
        decision = self.core.decide(
            signal_stop_required=True,
            stopline_valid=True,
            stopline_distance_m=2.0,
            stopline_confidence=1.0,
            speed_mps=5.0,
        )
        self.assertEqual(decision.mode, "approach_stopline")
        self.assertGreater(decision.brake, 0.0)
        self.assertEqual(decision.accel, 0.0)

    def test_missing_speed_is_fail_safe(self):
        decision = self.core.decide(
            signal_stop_required=True,
            stopline_valid=True,
            stopline_distance_m=3.0,
            stopline_confidence=1.0,
            speed_mps=None,
        )
        self.assertEqual(decision.mode, "stop")
        self.assertEqual(decision.brake, 1.0)


if __name__ == "__main__":
    unittest.main()
