#!/usr/bin/env python3
"""Read-only profile 2 input sample. Run via docker exec -i ... python -.

The host supplies this script so diagnosis works before a container update.
No parameter, publisher, vehicle command or existing process is modified.
"""
import json
import math
import os
from pathlib import Path
import threading
import time


def summarize(kind, msg, ros_now):
    if kind == 'status':
        data = json.loads(msg.data)
        keys = ('mode', 'reason', 'reference_path_match', 'progress_s_m',
                'signal_selection_reason', 'accel', 'brake')
        return {key: data[key] for key in keys if key in data}
    if kind in ('nominal', 'final'):
        return dict(type=msg.longlCmdType, accel=round(msg.accel, 3), brake=round(msg.brake, 3))
    if kind == 'reference':
        return dict(frame=msg.header.frame_id, points=len(msg.poses))
    if kind == 'odom':
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        return dict(frame=msg.header.frame_id, x=round(p.x, 2), y=round(p.y, 2),
                    speed_kph=round(math.hypot(v.x, v.y)*3.6, 2),
                    source_age_s=round(ros_now-msg.header.stamp.to_sec(), 3))
    return {}  # GPS/IMU wire activity only; not a claim of valid localization.


def recent_errors(log_root):
    """Bound file reads and output; old log timestamps remain visible."""
    paths = list(Path(log_root).glob('roslaunch-*.log'))
    paths += list(Path(log_root).glob('curvature_speed_purepursuit-*.log'))
    paths += list(Path(log_root).glob('ekf_local_enu-*.log'))
    for path in sorted(paths, key=lambda p: p.stat().st_mtime, reverse=True)[:3]:
        with path.open('rb') as stream:
            stream.seek(max(0, path.stat().st_size-65536))
            lines = stream.read(65536).decode('utf-8', errors='replace').splitlines()
        matches = [line for line in lines if any(s in line for s in
                   ('Traceback', 'Error:', 'Exception:', 'process has died', 'ERROR'))]
        for line in matches[-3:]:
            yield path.name + ': ' + line[-700:]


def main():
    import rospy
    import rospkg
    from morai_msgs.msg import CtrlCmd
    from nav_msgs.msg import Odometry, Path as RosPath
    from std_msgs.msg import String

    for name in ('ROS_IP', 'ROS_HOSTNAME', 'ROS_MASTER_URI'):
        print(name + '=' + os.environ.get(name, '(unset)'), flush=True)
    rospy.init_node('curvature_signal_read_only_diagnosis', anonymous=True, disable_signals=True)
    for name in ('/curvature_speed_purepursuit/publish_command',
                 '/curvature_speed_purepursuit/command_topic',
                 '/morai_udp_drive_bridge/control_output_enabled', '/use_sim_time'):
        print(name + '=' + str(rospy.get_param(name, 'NOT_SET')), flush=True)
    try:
        path = Path(rospkg.RosPack().get_path('curvature_speed_purepursuit'))
        source = (path/'scripts/curvature_speed_purepursuit_node.py').read_text(encoding='utf-8')
        print('CURVATURE_SOURCE reference_path_code=%s path=%s' %
              ('/experimental/curvature_reference_path' in source, path), flush=True)
    except Exception as exc:
        print('CURVATURE_SOURCE_ERROR ' + str(exc), flush=True)

    topics = {
        'gps': ('/localization/gps', rospy.AnyMsg),
        'imu': ('/Imu', rospy.AnyMsg),
        'odom': ('/localization/odometry', Odometry),
        'reference': ('/experimental/curvature_reference_path', RosPath),
        'nominal': ('/control/ctrl_cmd', CtrlCmd),
        'final': ('/ctrl_cmd', CtrlCmd),
        'status': ('/control/maneuver_status', String),
    }
    samples, lock = {}, threading.Lock()

    def receive(msg, kind):
        with lock:
            count = samples.get(kind, (0,))[0]
            samples[kind] = (count+1, time.monotonic(), msg)

    subscribers = [rospy.Subscriber(topic, cls, receive, callback_args=kind, queue_size=1)
                   for kind, (topic, cls) in topics.items()]
    print('SAMPLING 6 seconds; no control is sent.', flush=True)
    started = time.monotonic()
    while not rospy.is_shutdown() and time.monotonic()-started < 6:
        time.sleep(.05)
    code, message, state = rospy.get_master().getSystemState()
    if code != 1:
        raise RuntimeError('ROS master: ' + str(message))
    publishers = dict(state[0])
    with lock:
        snapshot = dict(samples)
    for kind, (topic, _) in topics.items():
        count, received, msg = snapshot.get(kind, (0, 0, None))
        data = summarize(kind, msg, rospy.Time.now().to_sec()) if msg is not None else {}
        print('%s count=%d age=%s pub=%s %s' % (kind, count,
              round(time.monotonic()-received, 2) if count else 'NONE',
              ','.join(publishers.get(topic, [])) or 'NONE', json.dumps(data)), flush=True)
    # Distinguish a fresh pose outside the route from no localization at all.
    if 'odom' in snapshot:
        try:
            from curvature_speed_purepursuit.planner import load_path_file, cumulative_arc_lengths, nearest_projection
            path_file = rospy.get_param('/curvature_signal_controller/path_file')
            points = load_path_file(path_file)
            position = snapshot['odom'][2].pose.pose.position
            projection = nearest_projection(points, cumulative_arc_lengths(points), position.x, position.y)
            print('ROUTE distance_m=%.2f allowed_m=%s (latest pose, full route)' %
                  (projection.distance_m, rospy.get_param('/curvature_signal_controller/max_route_error_m', 3.0)), flush=True)
        except Exception as exc:
            print('ROUTE_CHECK_ERROR ' + str(exc), flush=True)
    for sub in subscribers:
        sub.unregister()
    log_root = Path(os.environ.get('ROS_LOG_DIR', str(Path.home()/'.ros/log')))/'latest'
    for line in recent_errors(log_root):
        print('LOG ' + line, flush=True)
    print('DIAGNOSIS_DONE', flush=True)


if __name__ == '__main__':
    main()
