"""Atomic path/decision transport using existing std_msgs/String messages.

Separate Path/Bool/Float64 topics remain useful for RViz and diagnostics, but
their arrival order cannot be used as a control transaction. One JSON message
contains the geometry, its source stamp and the decision that approved it.
ROS imports are lazy so the contract can also be tested without a ROS master.
"""
from functools import wraps
import json
import math


def plan_locked(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        with self._plan_lock:
            return method(self, *args, **kwargs)
    return call


def path_payload(path):
    return dict(frame_id=path.header.frame_id, stamp=path.header.stamp.to_sec(),
                points=[[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                        for p in path.poses])


def validate_plan_status(payload):
    for key in ('planner_ready', 'avoidance_required', 'safe_path_available'):
        if type(payload[key]) is not bool:
            raise ValueError('plan_status_boolean:' + key)
    if type(payload['seq']) is not int or not 0 <= payload['seq'] <= 0xFFFFFFFF:
        raise ValueError('plan_status_sequence')


def read_path(payload, seq, frame, now, timeout, path_type=None, pose_type=None, stamp_type=None):
    if payload['frame_id'] != frame:
        raise ValueError('path_frame_mismatch')
    stamp = float(payload['stamp'])
    if not math.isfinite(stamp) or not -0.05 <= now - stamp <= timeout:
        raise ValueError('path_source_stale')
    points = payload['points']
    if not isinstance(points, list) or len(points) > 20000:
        raise ValueError('path_point_count')
    values = []
    for point in points:
        if len(point) != 3 or not all(math.isfinite(float(v)) for v in point):
            raise ValueError('path_nonfinite')
        values.append(tuple(float(v) for v in point))
    if path_type is None:
        from nav_msgs.msg import Path as path_type
        from geometry_msgs.msg import PoseStamped as pose_type
        from rospy import Time as stamp_type
    path = path_type()
    path.header.seq = int(seq)
    path.header.frame_id = frame
    path.header.stamp = stamp_type.from_sec(stamp)
    for x, y, z in values:
        pose = pose_type()
        pose.header = path.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def trajectory_payload(path, stop, speed, reason):
    return json.dumps(dict(schema=1, seq=path.header.seq, path=path_payload(path),
                           stop_required=bool(stop), target_speed_mps=float(speed),
                           reason=str(reason)), separators=(',', ':'), allow_nan=False)


def read_trajectory(data, frame, now, timeout, **types):
    payload = json.loads(data)
    if (payload['schema'] != 1 or type(payload['stop_required']) is not bool
            or type(payload['seq']) is not int or not 0 <= payload['seq'] <= 0xFFFFFFFF):
        raise ValueError('trajectory_schema')
    speed = float(payload['target_speed_mps'])
    if not math.isfinite(speed) or speed < 0.0:
        raise ValueError('trajectory_speed_invalid')
    path = read_path(payload['path'], payload['seq'], frame, now, timeout, **types)
    if not payload['stop_required'] and len(path.poses) < 2:
        raise ValueError('trajectory_path_missing')
    return path, payload['stop_required'], speed, str(payload.get('reason', ''))
