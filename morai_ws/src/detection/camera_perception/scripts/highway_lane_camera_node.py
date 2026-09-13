#!/usr/bin/env python3
"""Run the pinned dev/test_highway model with the local ROS/UDP transport."""
import runpy
import sys
from pathlib import Path
import rospkg

if __name__ == '__main__':
    root = Path(rospkg.RosPack().get_path('camera_perception')) / 'post_processing'
    sys.path.insert(0, str(root))
    runpy.run_path(str(root / 'real_lane_node.py'), run_name='__main__')
