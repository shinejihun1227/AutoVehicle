"""Schema regressions for the official beta_drive/main field difference.

Fixtures describe generated ROS1 slots from the upstream .msg definitions;
they do not replace a catkin build or a test with MORAI.
"""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


DIRECTORY = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("message_check", DIRECTORY / "check_morai_messages.py")
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)

BETA = {
    "CtrlCmd": """int32 longlCmdType
float64 accel
float64 brake
float64 steering
float64 velocity
float64 acceleration""",
    "EgoVehicleStatus": """Header header
int32 unique_id
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
float64 heading
float32 accel
float32 brake
float32 wheel_angle
float32 lateral_offset""",
    "GPSMessage": """Header header
float64 latitude
float64 longitude
float64 altitude
float64 eastOffset
float64 northOffset
int16 status""",
}


def classes(definitions):
    result = {}
    for name, definition in definitions.items():
        fields = [line.split() for line in definition.splitlines()]
        result[name] = type(name, (), {
            "__slots__": [field for kind, field in fields],
            "_slot_types": ["std_msgs/Header" if kind == "Header" else kind for kind, field in fields],
        })
    return result


class MessageContractTest(unittest.TestCase):
    def test_competition_fields_pass_with_additional_telemetry(self):
        self.assertEqual(checker.validate_messages(classes(BETA)), [])

    def test_main_renamed_steering_and_wheel_angle_are_rejected(self):
        definitions = dict(BETA)
        definitions["CtrlCmd"] = definitions["CtrlCmd"].replace("steering", "front_steer") + "\nfloat64 rear_steer"
        definitions["EgoVehicleStatus"] = definitions["EgoVehicleStatus"].replace("wheel_angle", "front_steer_angle")
        errors = checker.validate_messages(classes(definitions))
        self.assertEqual(len(errors), 2)
        self.assertIn("CtrlCmd.steering", errors[0])
        self.assertIn("EgoVehicleStatus.wheel_angle", errors[1])

    def test_incompatible_field_type_is_rejected(self):
        definitions = dict(BETA)
        definitions["GPSMessage"] = definitions["GPSMessage"].replace("int16 status", "int32 status")
        self.assertEqual(checker.validate_messages(classes(definitions)),
                         ["GPSMessage.status: expected int16, found int32"])

    def test_model_smoke_exits_nonzero_before_loading_models_on_wrong_sdk(self):
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory) / "morai_msgs"
            package.mkdir()
            (package / "__init__.py").write_text("", encoding="utf-8")
            (package / "msg.py").write_text(
                "class CtrlCmd:\n    __slots__ = ('front_steer',)\n    _slot_types = ('float64',)\n"
                "class EgoVehicleStatus:\n    __slots__ = ()\n    _slot_types = ()\n"
                "class GPSMessage:\n    __slots__ = ()\n    _slot_types = ()\n", encoding="utf-8")
            env = dict(os.environ, PYTHONPATH=directory, PYTHONDONTWRITEBYTECODE="1")
            completed = subprocess.run([sys.executable, "-B", str(DIRECTORY / "smoke_models.py")],
                                       env=env, capture_output=True, text=True, timeout=10)
            self.assertEqual(completed.returncode, 1)
            self.assertIn("MORAI_MESSAGES_FAIL", completed.stderr)
            self.assertIn("CtrlCmd.steering", completed.stderr)
            self.assertNotIn("Traceback", completed.stderr)


if __name__ == "__main__":
    unittest.main()
