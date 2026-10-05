"""Read-only, frame-matched previews from the existing inference processes.

The single-slot worker drops preview work under load; it never opens a camera
socket, runs a model or publishes a driving command.
"""
import json
import threading
import time


def render_lane(frame, mask, payload, camera, crop_top):
    import cv2
    import numpy as np
    output = frame.copy()
    top = crop_top if frame.shape[0] > camera.height else 0
    region = output[top:]
    labels = cv2.resize(mask, (region.shape[1], region.shape[0]), interpolation=cv2.INTER_NEAREST)
    # BGR: solid white, dashed cyan, yellow, stop-line red, guide violet.
    colors = np.array([[0, 0, 0], [230, 230, 230], [220, 200, 40],
                       [30, 210, 230], [60, 70, 230], [195, 110, 185]], dtype=np.uint8)
    valid = (labels > 0) & (labels < len(colors))
    region[valid] = (region[valid] * .5 + colors[labels[valid]] * .5).astype(np.uint8)

    def line(points, color):
        if not points:
            return
        xyz = np.array([[x, y, -.35] for x, y in points], dtype=float)
        uv, visible = camera.project(xyz)
        uv[:, 0] *= region.shape[1] / camera.width
        uv[:, 1] *= region.shape[0] / camera.height
        uv[:, 1] += top
        visible &= np.isfinite(uv).all(axis=1)
        # Clip individual segments; do not join disjoint visible sections.
        for i in range(1, len(uv)):
            if visible[i-1] and visible[i] and np.abs(uv[i-1:i+1]).max() < 100000:
                ok, a, b = cv2.clipLine((0, 0, output.shape[1], output.shape[0]),
                                       tuple(uv[i-1].astype(int)), tuple(uv[i].astype(int)))
                if ok:
                    cv2.line(output, a, b, color, 2)
    line(payload.get('left_boundary_points'), (220, 200, 40))
    line(payload.get('right_boundary_points'), (80, 200, 80))
    stop = payload.get('stopline')
    if stop:
        ys = np.linspace(*stop['y_range_m'], 25)
        line(list(zip(np.polyval(stop['coef'], ys), ys)), (60, 70, 230))
    return output


def render_objects(frame, detections):
    import cv2
    output = frame.copy()
    for x1, y1, x2, y2, label, score, color in detections:
        a = (max(0, int(x1)), max(0, int(y1)))
        b = (min(output.shape[1]-1, int(x2)), min(output.shape[0]-1, int(y2)))
        cv2.rectangle(output, a, b, color, 2)
        cv2.putText(output, '%s %.2f' % (label, score), (a[0], max(17, a[1]-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
    return output


class DebugImagePublisher:
    def __init__(self, camera_id, fps=5.0):
        import rospy
        from sensor_msgs.msg import CompressedImage, Image
        from std_msgs.msg import String
        self.ros, self.image_type, self.text_type = rospy, CompressedImage, String
        self.rviz_type = Image
        root = '/debug/cameras/' + camera_id
        self.image = rospy.Publisher(root + '/image/compressed', CompressedImage, queue_size=1)
        self.raw = rospy.Publisher(root + '/raw/compressed', CompressedImage, queue_size=1)
        self.rviz = rospy.Publisher(root + '/image', Image, queue_size=1)
        self.meta = rospy.Publisher(root + '/metadata', String, queue_size=1)
        self.camera_id = camera_id
        self.interval = 1.0 / max(1.0, min(float(fps), 10.0))
        self.condition = threading.Condition()
        self.pending, self.closed, self.last_submit = None, False, None
        self.worker = threading.Thread(target=self._run, name=camera_id + '-preview', daemon=True)
        self.worker.start()
        rospy.on_shutdown(self.close)

    def submit(self, frame, render, stamp, sequence, metadata):
        now = time.monotonic()
        with self.condition:
            if self.closed or (self.last_submit is not None and now-self.last_submit < self.interval):
                return
            self.last_submit = now
            self.pending = (frame, render, stamp, sequence, metadata)
            self.condition.notify()

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending is not None)
                if self.closed:
                    return
                job, self.pending = self.pending, None
            try:
                self._publish(*job)
            except Exception as exc:
                self.ros.logwarn_throttle(5., 'Camera preview failed (control unchanged): %s', exc)

    def _publish(self, frame, render, stamp, sequence, metadata):
        import cv2
        images = (render(), frame)
        messages = []
        for index, image in enumerate(images):
            if image.shape[1] > 800:
                image = cv2.resize(image, (800, round(image.shape[0]*800/image.shape[1])))
            # RViz's standard Image display needs no compressed transport plugin.
            # Only serialize this additional copy when an image viewer subscribes.
            if index == 0 and self.rviz.get_num_connections() > 0:
                preview = self.rviz_type()
                preview.header.stamp, preview.header.seq = stamp, int(sequence)
                preview.header.frame_id = self.camera_id
                preview.height, preview.width = image.shape[:2]
                preview.encoding, preview.is_bigendian = 'bgr8', 0
                preview.step, preview.data = preview.width*3, image.tobytes()
                self.rviz.publish(preview)
            ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                return
            msg = self.image_type()
            msg.header.stamp = stamp  # ORIGINAL frame receipt time, never encoding time.
            msg.header.seq = int(sequence)
            msg.header.frame_id = self.camera_id
            msg.format, msg.data = 'jpeg', encoded.tobytes()
            messages.append(msg)
        data = dict(metadata, sequence=int(sequence), stamp=stamp.to_sec(),
                    source_size=[frame.shape[1], frame.shape[0]])
        self.meta.publish(self.text_type(data=json.dumps(data, allow_nan=False)))
        self.image.publish(messages[0])
        self.raw.publish(messages[1])

    def close(self):
        with self.condition:
            self.closed, self.pending = True, None
            self.condition.notify_all()
        if threading.current_thread() is not self.worker:
            self.worker.join(timeout=1.)
