#!/usr/bin/env python3
"""Offline preflight: check official checkpoints/highway against the chosen map."""

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from mission_evaluator.course import Course, Route


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path",required=True)
    parser.add_argument("--mgeo",required=True)
    parser.add_argument("--rules",default=str(Path(__file__).resolve().parents[1]/"config"/"competition_rules_v1_1.json"))
    args = parser.parse_args()
    rules = json.loads(Path(args.rules).read_text(encoding="utf-8"))
    course = Course(Route.load(args.path),rules,args.mgeo)
    print(json.dumps(course.report(),ensure_ascii=False,indent=2,allow_nan=False))


if __name__ == "__main__":
    main()
