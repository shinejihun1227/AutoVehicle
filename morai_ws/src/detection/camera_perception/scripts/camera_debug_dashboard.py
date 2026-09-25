#!/usr/bin/env python3
"""Display both existing camera inference streams and driving stop reasons."""
import json
import math
from pathlib import Path
import threading

import rospy
import rospkg
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float64, Bool
from nav_msgs.msg import Odometry
from morai_msgs.msg import CtrlCmd
from morai_perception_msgs.msg import StopLineDetection, TrafficLight
from camera_perception.debug_dashboard import Telemetry, create_server


def main():
    rospy.init_node('camera_debug_dashboard')
    store = Telemetry()
    subscriptions = []

    def subscribe(topic, cls, callback):
        subscriptions.append(rospy.Subscriber(topic, cls, callback, queue_size=1, buff_size=4*1024*1024))

    def json_callback(key, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('expected object')
            store.update(key, value, value.get('timestamp', value.get('stamp')))
        except (ValueError, TypeError) as exc:
            rospy.logwarn_throttle(5., 'Preview telemetry %s invalid: %s', key, exc)

    for camera in ('cam1', 'cam4'):
        for raw in (False, True):
            topic = '/debug/cameras/%s/%s/compressed' % (camera, 'raw' if raw else 'image')
            subscribe(topic, CompressedImage, lambda msg, c=camera, r=raw:
                      store.put_image(c, r, msg.data, msg.header.stamp.to_sec(), msg.header.seq))
        subscribe('/debug/cameras/' + camera + '/metadata', String,
                  lambda msg, c=camera: json_callback(c, msg))
    for key, topic in (('maneuver', '/control/maneuver_status'),
                       ('lane', '/perception/camera/lane_info')):
        subscribe(topic, String, lambda msg, k=key: json_callback(k, msg))
    for key, topic in (('nominal', '/control/ctrl_cmd'), ('final', '/ctrl_cmd')):
        subscribe(topic, CtrlCmd, lambda msg, k=key: store.update(k,
                  dict(accel=msg.accel, brake=msg.brake, steering=msg.steering)))
    subscribe('/localization/odometry', Odometry, lambda msg: store.update('odom',
              dict(speed_kph=math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)*3.6),
              msg.header.stamp.to_sec()))
    subscribe('/perception/camera/stopline', StopLineDetection, lambda msg: store.update('stopline',
              dict(valid=msg.valid, distance_m=msg.distance_m, confidence=msg.confidence), msg.header.stamp.to_sec()))
    subscribe('/perception/traffic_light/directional_state', TrafficLight, lambda msg: store.update('signal',
              dict(valid=msg.valid, state=msg.state, confidence=msg.confidence), msg.header.stamp.to_sec()))
    for key, topic, cls in (('speed_target', '/experimental/curvature_speed_command', Float64),
                            ('speed_limit', '/experimental/curvature_speed_limit', Float64),
                            ('goal', '/experimental/curvature_goal_reached', Bool)):
        subscribe(topic, cls, lambda msg, k=key: store.update(k, msg.data))

    def settings(_event=None):
        store.update('config', dict(
            control_output_enabled=rospy.get_param('/morai_udp_drive_bridge/control_output_enabled', None),
            signal_camera=rospy.get_param('/curvature_signal_controller/signal_camera', {})))
    settings()
    timer = rospy.Timer(rospy.Duration(1.), settings)
    package = Path(rospkg.RosPack().get_path('camera_perception'))
    server = create_server(store, (package / 'web/camera_dashboard.html').read_bytes(),
                           rospy.get_time, int(rospy.get_param('~port', 8765)))
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    rospy.loginfo('CAM1 + CAM4 dashboard: http://127.0.0.1:%d (read-only)', server.server_port)
    try:
        rospy.spin()
    finally:
        timer.shutdown()
        server.shutdown()
        server.server_close()


if __name__ == '__main__':
    main()
