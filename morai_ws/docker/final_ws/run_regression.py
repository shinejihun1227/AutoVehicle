#!/usr/bin/env python3
"""Run the driving regressions in isolated Python processes, without ROS/UDP.

Use the same Python interpreter as the Docker runtime. Camera inference and
ROS boundaries are mocked by the tests; this does not replace smoke_models.py
or a catkin build, and never starts a vehicle controller.
"""
import argparse
import os
from pathlib import Path
import subprocess
import sys


SUITES = {
    "camera": "src/detection/camera_perception/test",
    "stopline": "src/control/stopline_control/test",
    "turn": "src/control/turn_signal_controller/test",
    "blackout": "src/experimental/stability_stack/test",
    "curvature": "src/experimental/curvature_speed_purepursuit/test",
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--suite", choices=list(SUITES), help="Run only one suite; default: all five")
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    selected = {args.suite: SUITES[args.suite]} if args.suite else SUITES
    env = os.environ.copy()
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    source_roots = [workspace / "src/detection/camera_perception/src",
                    workspace / "src/control/stopline_control/src",
                    workspace / "src/control/turn_signal_controller/src",
                    workspace / "src/experimental/curvature_speed_purepursuit/src"]
    env["PYTHONPATH"] = os.pathsep.join([str(p) for p in source_roots]
                                      + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else []))
    failed = []
    for name, relative in selected.items():
        directory = workspace / relative
        if not directory.is_dir() or not any(directory.glob("test*.py")):
            parser.error("Missing test suite: " + str(directory))
        print("\nREGRESSION_START " + name, flush=True)
        result = subprocess.run([sys.executable, "-B", "-m", "unittest", "discover",
                                 "-s", str(directory), "-p", "test*.py"],
                                cwd=str(workspace), env=env)
        if result.returncode:
            failed.append(name)
    if failed:
        print("REGRESSION_FAIL " + ", ".join(failed), flush=True)
        return 1
    print("REGRESSION_PASS {} suites (offline; not a MORAI driving result)".format(len(selected)), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
