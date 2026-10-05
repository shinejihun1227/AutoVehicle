#!/usr/bin/env python3
"""Read-only eight-second sample of the avoidance/merge control chain."""
import collections
import json
import math
import threading
import time

import rospy
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from morai_msgs.msg import CtrlCmd
from lidar_perception.msg import LidarObstacleArray
from morai_perception_msgs.msg import SafetyStop


def summarize(msg):
    if isinstance(msg, String):
        data = json.loads(msg.data)
        keys = ('state', 'mode', 'reason', 'reasons', 'stop', 'stop_required',
                'active_source', 'planner', 'freshness', 'commit', 'base', 'lane',
                'base_bypass_road_check', 'lane_change_reject', 'output', 'candidate_diag',
                'mission_request_active', 'lane_change_enabled', 'base_stop', 'base_reason',
                'trajectory_reason', 'trajectory_seq', 'measured_speed_kph', 'target_speed_kph',
                'target_speed_mps', 'lane_valid', 'confidence', 'lane_width_m', 'valid', 'left')
        return {k: data[k] for k in keys if k in data}
    if isinstance(msg, CtrlCmd):
        return dict(accel=round(msg.accel, 3), brake=round(msg.brake, 3),
                    steering=round(getattr(msg, 'steering', getattr(msg, 'front_steer', 0.0)), 3))
    if isinstance(msg, Odometry):
        v = msg.twist.twist.linear
        return dict(speed_kph=round(math.hypot(v.x, v.y)*3.6, 2),
                    frame=msg.header.frame_id, source_age_s=round((rospy.Time.now()-msg.header.stamp).to_sec(), 3))
    if isinstance(msg, LidarObstacleArray):
        return dict(obstacles=len(msg.obstacles), frame=msg.header.frame_id,
                    source_age_s=round((rospy.Time.now()-msg.header.stamp).to_sec(), 3))
    if isinstance(msg, SafetyStop):
        return dict(stop_required=msg.stop_required, reason=msg.reason)
    return dict(value=msg.data)


def main():
    rospy.init_node('highway_read_only_diagnosis', anonymous=True)
    for name in ('/bypass_lane_guard/require_atomic_plan_path',
                 '/avoidance_path_manager/require_atomic_plan_path',
                 '/highway_lane_strategy/base_trajectory_topic',
                 '/adaptive_curvature_purepursuit/trajectory_topic'):
        print(name + ' = ' + str(rospy.get_param(name, 'NOT_SET')), flush=True)
    topics = {
        '/bypass_lane_guard/status': String,
        '/avoidance_path_manager/state': String,
        '/highway_lane_strategy/state': String,
        '/control/curvature_status': String,
        '/control/stopline_status': String,
        '/control/mux_status': String,
        '/detection/fused_safety_stop': SafetyStop,
        '/control/ctrl_cmd': CtrlCmd,
        '/control/stopline_cmd': CtrlCmd,
        '/ctrl_cmd': CtrlCmd,
        '/localization/odometry': Odometry,
        '/perception/lidar/tracked_obstacles_map': LidarObstacleArray,
        '/perception/camera/lane_info': String,
        '/planning/highway_lane_change_request': Bool,
        '/perception/merge_gap/available': Bool,
        '/perception/merge_gap/unavailable': Bool,
        '/morai/lidar/merge_gap/results': String,
    }
    lock = threading.Lock()
    samples = {topic: dict(count=0, last=None, changes=collections.Counter()) for topic in topics}
    started = time.monotonic()

    def receive(msg, topic):
        try:
            data = summarize(msg)
        except Exception as exc:
            data = dict(decode_error=str(exc))
        with lock:
            sample = samples[topic]
            sample['count'] += 1
            sample['last'] = data
            sample['at'] = time.monotonic()
            state = {k: data[k] for k in ('state', 'mode', 'reason', 'reasons', 'stop', 'stop_required') if k in data}
            if state:
                key = json.dumps(state, ensure_ascii=False, sort_keys=True)
                sample['changes'][key] += 1
                if key != sample.get('previous'):
                    print('t=%.2fs %s %s' % (sample['at']-started, topic, key), flush=True)
                sample['previous'] = key

    subscribers = [rospy.Subscriber(topic, cls, receive, callback_args=topic, queue_size=10)
                   for topic, cls in topics.items()]
    print('READ_ONLY: collecting 8 seconds; no control/request is published.', flush=True)
    while not rospy.is_shutdown() and time.monotonic() - started < 8.0:
        time.sleep(0.05)
    elapsed = time.monotonic() - started
    with lock:
        for topic, sample in samples.items():
            result = dict(count=sample['count'], hz=round(sample['count']/elapsed, 1),
                          last_age_s=round(time.monotonic()-sample['at'], 2) if sample['count'] else None,
                          last=sample['last'], decisions=dict(sample['changes']))
            print(topic + ' ' + json.dumps(result, ensure_ascii=False, sort_keys=True))
    for sub in subscribers:
        sub.unregister()
    print('DIAGNOSIS_DONE: missing mission request is normal when no lane change was requested.')


if __name__ == '__main__':
    main()
