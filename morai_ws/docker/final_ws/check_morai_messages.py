#!/usr/bin/env python3
"""Check generated ROS1 messages used by final_ws, without starting ROS or UDP.

The competition SDK is MORAI-ROS_morai_msgs/beta_drive. Import success alone
does not detect main's incompatible front_steer/front_steer_angle fields.
"""
import sys


REQUIRED = {
    "CtrlCmd": {
        "longlCmdType": "int32", "accel": "float64", "brake": "float64",
        "steering": "float64", "velocity": "float64", "acceleration": "float64",
    },
    "EgoVehicleStatus": {
        "header": "std_msgs/Header", "unique_id": "int32",
        "acceleration": "geometry_msgs/Vector3", "position": "geometry_msgs/Vector3",
        "velocity": "geometry_msgs/Vector3", "heading": "float64",
        "accel": "float32", "brake": "float32", "wheel_angle": "float32",
    },
    "GPSMessage": {
        "header": "std_msgs/Header", "latitude": "float64", "longitude": "float64",
        "altitude": "float64", "eastOffset": "float64", "northOffset": "float64",
        "status": "int16",
    },
}


def validate_messages(classes):
    errors = []
    for name, expected in REQUIRED.items():
        cls = classes.get(name)
        actual = dict(zip(getattr(cls, "__slots__", ()), getattr(cls, "_slot_types", ())))
        for field, field_type in expected.items():
            if actual.get(field) != field_type:
                errors.append("{}.{}: expected {}, found {}".format(
                    name, field, field_type, actual.get(field, "MISSING")))
    return errors


def main():
    try:
        from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, GPSMessage
    except ImportError as exc:
        print("MORAI_MESSAGES_FAIL: {}. Build beta_drive and source devel/setup.bash.".format(exc),
              file=sys.stderr)
        return 1
    classes = {cls.__name__: cls for cls in (CtrlCmd, EgoVehicleStatus, GPSMessage)}
    for name, cls in classes.items():
        print(name, getattr(sys.modules.get(cls.__module__), "__file__", "unknown source"))
    errors = validate_messages(classes)
    if errors:
        print("MORAI_MESSAGES_FAIL:\n" + "\n".join(errors), file=sys.stderr)
        print("Use MORAI-ROS_morai_msgs/beta_drive, rebuild this workspace, then source its "
              "devel/setup.bash in a new terminal. Stop and restart existing ROS nodes.",
              file=sys.stderr)
        return 1
    print("MORAI_MESSAGES_OK: beta_drive-compatible fields (not a sensor or driving test)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
