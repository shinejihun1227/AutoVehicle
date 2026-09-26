"""Read-only dashboard behavior, source freshness and real OpenCV overlays."""
import json
from pathlib import Path
import sys
import threading
import unittest
from types import SimpleNamespace as NS
from urllib.request import urlopen, Request
from urllib.error import HTTPError
from unittest.mock import Mock, patch

from camera_perception.debug_dashboard import Telemetry, create_server
from camera_perception.debug_images import DebugImagePublisher, render_lane, render_objects


class DashboardTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.store = Telemetry(lambda: self.now)
        self.store.update('config', dict(control_output_enabled=True))
        self.store.update('maneuver', dict(mode='NOMINAL', reason='nominal'))
        self.store.update('final', dict(accel=.2, brake=0.))

    def test_monitor_never_claims_vehicle_control_is_enabled(self):
        self.store.update('config', dict(control_output_enabled=False))
        self.assertEqual(self.store.snapshot(self.now)['diagnosis']['state'], 'monitor')

    def test_current_stop_and_selection_reason_are_both_visible(self):
        self.store.update('maneuver', dict(mode='SAFE_STOP', reason='unassociated_visible_signal',
                          signal_selection_reason='signal_camera_uncalibrated'))
        result = self.store.snapshot(self.now)['diagnosis']
        self.assertEqual(result['state'], 'stop')
        self.assertEqual({n['code'] for n in result['notes']},
                         {'unassociated_visible_signal', 'signal_camera_uncalibrated'})

    def test_sensor_only_launch_reports_old_or_unmarked_controller(self):
        self.store.update('config', dict(control_output_enabled=True,
                          require_route_signal_context=False,
                          stopline_requires_detected_signal=True))
        self.store.update('maneuver', dict(mode='SAFE_STOP', reason='unmapped_signal_or_stopline',
                          controller_profile='mgeo'))
        result = self.store.snapshot(self.now)['diagnosis']
        self.assertIn('controller_profile_mismatch', {n['code'] for n in result['notes']})

    def test_sensor_only_controller_with_line_without_light_does_not_report_map_failure(self):
        self.store.update('config', dict(control_output_enabled=True,
                          require_route_signal_context=False,
                          stopline_requires_detected_signal=True))
        self.store.update('maneuver', dict(mode='NOMINAL', reason='nominal',
                          controller_profile='sensor_only',
                          signal_selection_reason='awaiting_paired_signal_stopline'))
        result = self.store.snapshot(self.now)['diagnosis']
        self.assertNotIn('controller_profile_mismatch', {n['code'] for n in result['notes']})
        self.assertNotIn('unmapped_signal_or_stopline', {n['code'] for n in result['notes']})

    def test_stopped_publisher_cannot_leave_a_live_acceleration_banner(self):
        self.assertEqual(self.store.snapshot(self.now)['diagnosis']['state'], 'command')
        self.now += 1.1
        self.assertEqual(self.store.snapshot(self.now)['diagnosis']['state'], 'waiting')

    def test_future_and_old_source_commands_are_not_fresh(self):
        for stamp in (98., 102.):
            self.store.update('final', dict(accel=1., brake=0.), stamp)
            self.assertEqual(self.store.snapshot(self.now)['diagnosis']['state'], 'waiting')

    def test_images_and_nan_stopline_do_not_hide_their_source_age(self):
        self.store.put_image('cam1', False, b'old-jpeg', 97., 4)
        self.store.put_image('cam1', True, b'raw-jpeg', 97., 4)
        self.store.update('stopline', dict(distance_m=float('nan')), 98.)
        snapshot = self.store.snapshot(100.)
        self.assertEqual(snapshot['images']['cam1']['overlay']['source_age_s'], 3.)
        self.assertEqual(snapshot['images']['cam1']['raw']['sequence'], 4)
        self.assertIsNone(snapshot['values']['stopline']['value']['distance_m'])
        json.dumps(snapshot, allow_nan=False)

    def test_http_serves_status_images_and_rejects_control_posts(self):
        self.store.put_image('cam4', False, b'jpeg-test', 100., 1)
        server = create_server(self.store, b'<html>preview</html>', lambda: 100., port=0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = 'http://127.0.0.1:%d' % server.server_port
        try:
            with urlopen(base + '/api/status', timeout=2) as response:
                self.assertEqual(json.load(response)['diagnosis']['state'], 'command')
            with urlopen(base + '/image/cam4', timeout=2) as response:
                self.assertEqual(response.read(), b'jpeg-test')
                self.assertEqual(response.headers['Cache-Control'], 'no-store')
            for path, method, expected in (('/image/cam1', 'GET', 503),
                                            ('/../../config', 'GET', 404),
                                            ('/api/status', 'POST', 501)):
                with self.assertRaises(HTTPError) as error:
                    urlopen(Request(base + path, method=method), timeout=2)
                self.assertEqual(error.exception.code, expected)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


class PreviewImageTest(unittest.TestCase):
    def test_lane_mask_respects_crop_and_does_not_mutate_inference_input(self):
        import numpy as np
        frame = np.full((48, 64, 3), 100, np.uint8)
        original = frame.copy()
        mask = np.full((28, 64), 4, np.uint8)
        output = render_lane(frame, mask, {}, NS(height=28, width=64), 20)
        np.testing.assert_array_equal(frame, original)
        np.testing.assert_array_equal(output[:20], original[:20])
        self.assertGreater(int(output[35, 20, 2]), int(output[35, 20, 0]))

    def test_object_boxes_are_drawn_on_a_copy_of_the_exact_source(self):
        import numpy as np
        frame = np.zeros((48, 64, 3), np.uint8)
        output = render_objects(frame, ((8, 20, 35, 40, 'RED', .9, (0, 0, 255)),))
        self.assertEqual(int(frame.sum()), 0)
        self.assertEqual(int(output[20, 8, 2]), 255)

    def test_encoding_preserves_original_stamp_sequence_and_raw_frame(self):
        import numpy as np
        class Image:
            def __init__(self):
                self.header = NS()
        writer = DebugImagePublisher.__new__(DebugImagePublisher)
        writer.image_type, writer.text_type = Image, NS
        writer.image, writer.raw, writer.meta = Mock(), Mock(), Mock()
        writer.rviz_type, writer.rviz = Image, Mock()
        writer.rviz.get_num_connections.return_value = 1
        writer.camera_id = 'cam4'
        stamp = NS(to_sec=lambda: 72.)
        frame = np.zeros((48, 64, 3), np.uint8)
        writer._publish(frame, lambda: frame.copy(), stamp, 31, {'custom_loaded': True})
        for pub in (writer.image, writer.raw):
            msg = pub.publish.call_args.args[0]
            self.assertIs(msg.header.stamp, stamp)
            self.assertEqual(msg.header.seq, 31)
            self.assertTrue(msg.data.startswith(b'\xff\xd8'))
        data = json.loads(writer.meta.publish.call_args.args[0].data)
        self.assertEqual((data['stamp'], data['sequence']), (72., 31))
        preview = writer.rviz.publish.call_args.args[0]
        self.assertIs(preview.header.stamp, stamp)
        self.assertEqual(preview.header.seq, 31)
        self.assertEqual((preview.height, preview.width, preview.encoding, preview.step), (48, 64, 'bgr8', 192))
        np.testing.assert_array_equal(np.frombuffer(preview.data, np.uint8).reshape(48, 64, 3), frame)
        writer.rviz.publish.reset_mock()
        writer.rviz.get_num_connections.return_value = 0
        writer._publish(frame, lambda: frame.copy(), stamp, 32, {})
        writer.rviz.publish.assert_not_called()

    def test_rviz_receives_overlay_not_original_and_caps_preview_width(self):
        import numpy as np
        writer = DebugImagePublisher.__new__(DebugImagePublisher)
        class Image:
            def __init__(self):
                self.header = NS()
        writer.image_type, writer.text_type, writer.rviz_type = Image, NS, Image
        writer.image, writer.raw, writer.meta, writer.rviz = Mock(), Mock(), Mock(), Mock()
        writer.rviz.get_num_connections.return_value = 1
        writer.camera_id = 'cam1'
        frame = np.zeros((600, 1000, 3), np.uint8)
        overlay = frame.copy()
        overlay[:, :, 2] = 255
        writer._publish(frame, lambda: overlay, NS(to_sec=lambda: 1.), 1, {})
        preview = writer.rviz.publish.call_args.args[0]
        self.assertEqual((preview.width, preview.height), (800, 480))
        pixels = np.frombuffer(preview.data, np.uint8).reshape(480, 800, 3)
        self.assertTrue((pixels[:, :, 2] == 255).all())
        self.assertFalse(frame.any())

    def test_slow_preview_replaces_pending_work_without_waiting_for_encoding(self):
        writer = DebugImagePublisher.__new__(DebugImagePublisher)
        writer.condition = threading.Condition()
        writer.pending, writer.closed, writer.last_submit = None, False, None
        writer.interval = .2
        with patch('camera_perception.debug_images.time.monotonic', side_effect=[10., 10.1, 10.3]):
            for seq in (1, 2, 3):
                writer.submit(None, None, None, seq, {})
        self.assertEqual(writer.pending[3], 3)

    def test_launch_keeps_one_receiver_per_camera_and_enables_preview(self):
        import xml.etree.ElementTree as ET
        ws = Path(__file__).resolve().parents[4]
        root = ET.parse(ws / 'src/bringup/morai_bringup/launch/final_ws_curvature_signal.launch').getroot()
        lane = root.findall("node[@type='highway_lane_camera_node.py']")
        self.assertEqual(len(lane), 1)
        self.assertEqual(lane[0].find("env[@name='MORAI_CAMERA_DEBUG']").get('value'), '$(arg enable_camera_dashboard)')
        dashboard = root.find("node[@name='camera_debug_dashboard']")
        self.assertIsNotNone(dashboard)
        self.assertIsNone(dashboard.get('required'))  # A viewer failure must not kill the control graph.


if __name__ == '__main__':
    unittest.main()
