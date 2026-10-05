"""Reuse /Imu orientation history for camera tracking; never bind another UDP port."""
from collections import deque
import math
import threading
import time


class RosImuHistory:
    def __init__(self, **_):
        self.samples = deque(maxlen=1000)
        self.lock = threading.Lock()
        self.sub = None

    def start(self):
        import rospy
        from sensor_msgs.msg import Imu
        self.sub = rospy.Subscriber('/Imu', Imu, self.callback, queue_size=100)
        return self

    def callback(self, msg):
        q = msg.orientation
        values = (q.x, q.y, q.z, q.w, msg.header.stamp.to_sec())
        if not all(math.isfinite(x) for x in values) or sum(x*x for x in values[:4]) < 0.5:
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        with self.lock:
            t = values[4]
            if self.samples and t < self.samples[-1][0]: self.samples.clear()
            if not self.samples or t > self.samples[-1][0]: self.samples.append((t, yaw))

    def delta_yaw(self, start, end):
        with self.lock:
            samples = list(self.samples)
        if len(samples) < 2: return None
        a = min(samples, key=lambda p: abs(p[0]-start))
        b = min(samples, key=lambda p: abs(p[0]-end))
        if max(abs(a[0]-start), abs(b[0]-end)) > 0.05: return None
        return math.atan2(math.sin(b[1]-a[1]), math.cos(b[1]-a[1]))

    def wait_first(self, timeout=3.0):
        until = time.monotonic()+timeout
        while time.monotonic() < until:
            with self.lock:
                if self.samples: return True
            time.sleep(0.02)
        return False

    def stop(self):
        if self.sub is not None: self.sub.unregister()
