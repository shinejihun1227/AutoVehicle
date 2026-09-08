"""Receipt-time contracts, exercising real loops without ROS, models or GUI.

Run with: python -B -m unittest discover -s test -p 'test_*.py'
Clocks and inference are controlled; no simulator or wall-clock sleeps needed.
"""

import importlib.util
from pathlib import Path
import struct
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))


class Stamp:
    def __init__(self, seconds=0.0):
        self.seconds = seconds

    def __bool__(self):
        return self.seconds != 0.0

    def __eq__(self, other):
        return isinstance(other, Stamp) and self.seconds == other.seconds

    def to_sec(self):
        return self.seconds


class Header:
    def __init__(self, seq=0, stamp=None, frame_id=""):
        self.seq = seq
        self.stamp = Stamp() if stamp is None else stamp
        self.frame_id = frame_id


class Message:
    def __init__(self, **kwargs):
        self.header = Header()
        self.objects = []
        self.__dict__.update(kwargs)


class Image:
    def __init__(self, data=b"jpeg"):
        self.data = data
        self.size = len(data)
        self.shape = (48, 64, 3)

    def copy(self):
        return Image(self.data)


class Clock:
    def __init__(self, seconds=100.0):
        self.seconds = seconds

    def advance(self, seconds):
        self.seconds += seconds

    def monotonic(self):
        return self.seconds

    def stamp(self):
        return Stamp(self.seconds)


def load_module(relative_path, modules):
    name = "_timestamp_test_" + Path(relative_path).stem
    spec = importlib.util.spec_from_file_location(name, PACKAGE / relative_path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {**modules, name: module}):
        spec.loader.exec_module(module)
    return module


def vision_modules():
    cv2 = Mock(IMREAD_COLOR=1, FONT_HERSHEY_SIMPLEX=0, error=RuntimeError)
    cv2.imdecode.side_effect = lambda buffer, _mode: Image(buffer.data)
    cv2.waitKey.return_value = -1
    np = SimpleNamespace(uint8="uint8", frombuffer=lambda data, dtype: Image(data))
    return {"cv2": cv2, "numpy": np}


def ros_modules():
    publishers = {}

    def publisher(topic, *_args, **_kwargs):
        pub = Mock()
        pub.messages = []
        pub.publish.side_effect = pub.messages.append
        publishers[topic] = pub
        return pub

    ros = Mock()
    ros.Time = Mock(side_effect=Stamp)
    # Any accidental publication-time stamp will differ from frame receipt.
    ros.Time.now.return_value = Stamp(9000.0)
    ros.Publisher.side_effect = publisher
    ros.is_shutdown.return_value = False
    ros.get_param.side_effect = lambda _name, default: default
    ros.ROSException = RuntimeError
    return {
        "rospy": ros,
        "std_msgs.msg": SimpleNamespace(
            Header=Header, Bool=Message, Float64=Message, String=Message,
        ),
        "common.msg": SimpleNamespace(ObjectInfo=Message, ObjectInfoArray=Message),
        "morai_perception_msgs.msg": SimpleNamespace(
            LaneDetection=Message, StopLineDetection=Message, TrafficLight=Message,
        ),
    }, publishers


class CameraTimestampTest(unittest.TestCase):
    def setUp(self):
        self.vision = vision_modules()
        self.udp = load_module("src/camera_perception/camera_udp.py", self.vision)
        self.modules = {**self.vision, "camera_perception.camera_udp": self.udp}

    def frame(self, sequence, stamp=None):
        return self.udp.CameraFrame(
            sequence=sequence, sec=12, nsec=345, index=sequence,
            jpeg_data=bytes([sequence]), received_at=100.0 + sequence / 10.0,
            received_stamp=stamp,
        )

    def test_udp_receipt_precedes_assembly_and_latest_skips_without_restamping(self):
        clock = Clock()
        socket = Mock()
        with patch.object(self.udp.socket, "socket", return_value=socket), \
                patch.object(self.udp.threading, "Thread"):
            receiver = self.udp.LatestCameraReceiver(
                "127.0.0.1", 1101, stamp_clock=clock.stamp,
            )
        self.addCleanup(receiver.close)
        original_parse = receiver._packet.parsing

        def parse():
            clock.advance(0.03)  # Assembly delay must not renew receipt time.
            original_parse()

        def packet(tail, index):
            return (b"MOR" + struct.pack("<iiii", 12, 345, index, 1)
                    + b"j".ljust(64979, b"\0") + tail)

        self.udp.time = clock
        with patch.object(receiver._packet, "parsing", side_effect=parse):
            socket.recvfrom.side_effect = [(packet(b"MI", 1), None), StopIteration]
            with self.assertRaises(StopIteration):
                receiver._receive_loop()
            self.assertIsNone(receiver.wait_for_latest(timeout=0))
            socket.recvfrom.side_effect = [(packet(b"EI", 1), None), StopIteration]
            with self.assertRaises(StopIteration):
                receiver._receive_loop()
            first = receiver.wait_for_latest(timeout=0)
            self.assertAlmostEqual(first.received_at, 100.03)
            self.assertAlmostEqual(first.received_stamp.to_sec(), 100.03)
            self.assertEqual((first.sec, first.nsec), (12, 345))

            socket.recvfrom.side_effect = [
                (packet(b"EI", 2), None), (packet(b"EI", 3), None), StopIteration,
            ]
            with self.assertRaises(StopIteration):
                receiver._receive_loop()
            clock.advance(2.0)  # Consumer backlog must remain visible in age.
            newest = receiver.wait_for_latest(first.sequence, timeout=0)
            self.assertEqual(newest.sequence, 3)
            self.assertGreater(newest.received_stamp.to_sec(), first.received_stamp.to_sec())
            self.assertAlmostEqual(clock.seconds - newest.received_stamp.to_sec(), 2.03)
            self.assertIs(newest, receiver.wait_for_latest(first.sequence, timeout=0))
            self.assertIsNone(receiver.wait_for_latest(newest.sequence, timeout=0))

    def test_udp_without_clock_keeps_unknown_stamp(self):
        socket = Mock()
        with patch.object(self.udp.socket, "socket", return_value=socket), \
                patch.object(self.udp.threading, "Thread"):
            receiver = self.udp.LatestCameraReceiver("127.0.0.1", 1101)
        self.addCleanup(receiver.close)
        raw = b"MOR" + struct.pack("<iiii", 55, 123, 1, 1) + b"j" * 64979 + b"EI"
        socket.recvfrom.side_effect = [(raw, None), StopIteration]
        with self.assertRaises(StopIteration):
            receiver._receive_loop()
        self.assertIsNone(receiver.wait_for_latest(timeout=0).received_stamp)

    def test_lane_decode_keeps_image_and_receipt_metadata_together(self):
        camera = load_module("lane/morai_camera.py", self.modules)
        clock = Clock()
        stream = camera.CameraStream(stamp_clock=clock.stamp)
        self.assertEqual(stream.latest(), (None, -1))
        self.assertEqual(stream.latest_with_metadata(), (None, None))
        frames = [self.frame(i, Stamp(100.0 + i / 10.0)) for i in (1, 2, 3)]
        pending = iter(frames)
        decoded_snapshots = []

        def next_frame(*_args, **_kwargs):
            decoded_snapshots.append(stream.latest_with_metadata())
            try:
                return next(pending)
            except StopIteration:
                stream.stop()
                return None

        def decode(buffer, _mode):
            clock.advance(0.3)
            return None if buffer.data == b"\x02" else Image(buffer.data)

        receiver = Mock()
        receiver.wait_for_latest.side_effect = next_frame
        camera.LatestCameraReceiver = Mock(return_value=receiver)
        camera.cv2.imdecode.side_effect = decode
        stream._worker()
        image, metadata = stream.latest_with_metadata()
        self.assertEqual(image.data, b"\x03")
        self.assertIs(metadata, frames[2])
        self.assertEqual(metadata.received_stamp, Stamp(100.3))
        self.assertGreater(clock.seconds, metadata.received_stamp.to_sec())
        self.assertIs(decoded_snapshots[2][1], frames[0])  # Failed decode kept pair.
        self.assertEqual(stream.decode_errors, 1)
        self.assertEqual(stream.latest()[1], 3)  # Legacy interface unchanged.
        image.data = b"changed copy"
        self.assertEqual(stream.latest()[0].data, b"\x03")
        camera.LatestCameraReceiver.assert_called_once_with(
            stream.ip, stream.port, stamp_clock=clock.stamp,
        )
        receiver.close.assert_called_once()

    def run_overlay(self, stamps):
        ros_deps, publishers = ros_modules()
        ros = ros_deps["rospy"]
        clock = Clock()
        frames = [self.frame(i, stamp) for i, stamp in enumerate(stamps, 1)]
        # Repeated polls plus --every=2: only frames 2 and 4 are observations.
        snapshots = iter([frames[0], frames[1], frames[1], frames[2], frames[3]])
        stream = Mock()
        stream.start.return_value = stream

        def latest():
            try:
                frame = next(snapshots)
                return Image(frame.jpeg_data), frame
            except StopIteration:
                ros.is_shutdown.return_value = True
                return None, None

        stream.latest_with_metadata.side_effect = latest
        detector = Mock(ckpt_info={"epoch": 1, "backbone": "test"})
        distances = iter([8.25, None])

        def infer(_image):
            clock.advance(0.7)
            return SimpleNamespace(ego_left=None, ego_right=None, stopline_dist=next(distances))

        detector.run.side_effect = infer
        quality = Mock()
        quality.update.return_value = {
            "confidence": 0.0, "valid": False, "lateral_error": None, "heading_error": None,
        }
        deps = {
            **self.modules, **ros_deps,
            "lane_detection": SimpleNamespace(LaneDetector=Mock(return_value=detector), default_checkpoint=lambda: "test"),
            "lane_quality": SimpleNamespace(LaneQualityEstimator=Mock(return_value=quality)),
            "lane_viz": SimpleNamespace(draw=lambda *_a, **_kw: Image(), draw_bev=Mock()),
            "morai_camera": SimpleNamespace(
                DEFAULT_IP="127.0.0.1", DEFAULT_PORT=1101, CameraStream=Mock(return_value=stream),
            ),
        }
        overlay = load_module("lane/live_overlay.py", deps)
        with patch.dict(sys.modules, deps):
            overlay.main(["--ros-publish", "--no-values", "--every", "2"])
        deps["morai_camera"].CameraStream.assert_called_once_with(
            "127.0.0.1", 1101, stamp_clock=ros.Time.now,
        )
        ros.Time.now.assert_not_called()
        return publishers, frames

    def test_lane_publishes_inferred_frame_stamp_and_binary_stopline_quality(self):
        pubs, frames = self.run_overlay([Stamp(100.1), Stamp(100.2), Stamp(100.3), Stamp(100.4)])
        stops = pubs["/perception/camera/stopline"].messages
        lanes = pubs["/detection/lane"].messages
        self.assertEqual([msg.header.seq for msg in stops[:-1]], [2, 4])
        self.assertEqual([msg.header.stamp for msg in stops[:-1]], [frames[1].received_stamp, frames[3].received_stamp])
        self.assertEqual([msg.header.stamp for msg in lanes], [msg.header.stamp for msg in stops])
        self.assertTrue(all(msg.header.frame_id == "base_link" for msg in stops))
        self.assertEqual((stops[0].valid, stops[0].distance_m, stops[0].confidence), (True, 8.25, 1.0))
        self.assertEqual((lanes[0].valid, lanes[0].confidence), (False, 0.0))
        self.assertEqual((stops[1].valid, stops[1].confidence), (False, 0.0))
        self.assertEqual((stops[-1].valid, stops[-1].header.stamp), (False, Stamp()))

    def test_lane_unknown_and_zero_receipt_stamps_are_not_renewed(self):
        pubs, _ = self.run_overlay([Stamp(1.0), None, Stamp(2.0), Stamp()])
        self.assertEqual(
            [msg.header.stamp for msg in pubs["/perception/camera/stopline"].messages],
            [Stamp(), Stamp(), Stamp()],
        )

    def run_yolo(self, custom=True, stamps=None):
        ros_deps, publishers = ros_modules()
        ros = ros_deps["rospy"]
        clock = Clock(101.0)
        frames = [self.frame(i, stamp) for i, stamp in enumerate(
            stamps if stamps is not None else [Stamp(100.1), Stamp(100.2), Stamp(100.3)], 1,
        )]
        base_started = threading.Event()
        newer_pending = threading.Event()
        finished = threading.Event()
        received = iter(frames)
        base = Mock(names={})
        custom_model = Mock(names={})

        def base_predict(source, **_kwargs):
            if source.data == b"\x01":
                base_started.set()
                if not newer_pending.wait(5.0):
                    raise RuntimeError("newer frames were not queued")
            clock.advance(0.4)
            return [SimpleNamespace(boxes=[])]

        def custom_predict(**_kwargs):
            clock.advance(0.6)
            return [SimpleNamespace(boxes=[])]

        base.predict.side_effect = base_predict
        custom_model.predict.side_effect = custom_predict
        factory = Mock(side_effect=[base, custom_model] if custom else [base])
        receiver = Mock()

        def next_frame(*_args, **_kwargs):
            try:
                frame = next(received)
                if frame.sequence == 2 and not base_started.wait(5.0):
                    raise RuntimeError("inference did not start")
                return frame
            except StopIteration:
                newer_pending.set()  # Frames 2 and 3 replaced the pending job.
                completed = finished.wait(5.0)
                ros.is_shutdown.return_value = True
                if not completed:
                    raise RuntimeError("inference did not finish")
                return None

        receiver.wait_for_latest.side_effect = next_frame

        def decode(buffer, _mode):
            clock.advance(0.2)
            return Image(buffer.data)

        self.vision["cv2"].imdecode.side_effect = decode
        deps = {
            **self.modules, **ros_deps, "ultralytics": SimpleNamespace(YOLO=factory),
            "torch": SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: True)),
        }
        yolo = load_module("scripts/camera_object_detection_node.py", deps)
        yolo.time = clock
        yolo.LatestCameraReceiver = Mock(return_value=receiver)
        publish_times = []
        original_publisher = ros.Publisher.side_effect

        def publisher(topic, *args, **kwargs):
            pub = original_publisher(topic, *args, **kwargs)

            def publish(message):
                pub.messages.append(message)
                if topic == "/detection/traffic_light":
                    publish_times.append(clock.seconds)
                    if message.header.seq == 3:
                        finished.set()

            pub.publish.side_effect = publish
            return pub

        ros.Publisher.side_effect = publisher
        with patch.dict(sys.modules, deps), patch.object(yolo.os.path, "isfile", return_value=custom):
            yolo.main(custom_model_path="test.pt")
        self.assertTrue(finished.is_set(), "worker must publish newest pending frame")
        ros.logerr_throttle.assert_not_called()
        ros.Time.now.assert_not_called()
        yolo.LatestCameraReceiver.assert_called_once_with(yolo.IP, yolo.PORT, stamp_clock=ros.Time.now)
        self.assertEqual([call.kwargs["source"].data for call in base.predict.call_args_list], [b"\x01", b"\x03"])
        receiver.close.assert_called_once()
        return publishers, frames, publish_times

    def test_yolo_pending_replacement_and_two_inferences_preserve_frame_age(self):
        pubs, frames, published_at = self.run_yolo()
        traffic = pubs["/detection/traffic_light"].messages
        obstacles = pubs["/detection/obstacle"].messages
        self.assertEqual([msg.header.seq for msg in traffic], [1, 3])
        self.assertEqual([msg.header.seq for msg in obstacles], [1, 1, 3, 3])
        for msg in traffic + obstacles:
            self.assertEqual(msg.header.stamp, frames[msg.header.seq - 1].received_stamp)
        self.assertGreater(published_at[0] - traffic[0].header.stamp.to_sec(), 1.5)
        self.assertLess(traffic[0].header.stamp.to_sec(), traffic[1].header.stamp.to_sec())

    def test_yolo_no_custom_model_and_missing_stamps_stay_unknown(self):
        pubs, _, _ = self.run_yolo(custom=False, stamps=[None, Stamp(100.2), Stamp()])
        traffic = pubs["/detection/traffic_light"].messages
        self.assertEqual([msg.header.seq for msg in traffic], [1, 3])
        self.assertEqual([msg.header.stamp for msg in traffic], [Stamp(), Stamp()])
        self.assertTrue(all(not msg.objects for msg in traffic))


class TrafficLightObservationTest(unittest.TestCase):
    def setUp(self):
        self.modules, self.publishers = ros_modules()
        self.ros = self.modules["rospy"]
        self.node_module = load_module("scripts/traffic_light_stop_node.py", self.modules)
        self.clock = Clock(0.0)
        self.node_module.time = self.clock
        self.node = self.node_module.TrafficLightStopNode()
        self.states = self.publishers["/perception/traffic_light/state"].messages
        self.bools = self.publishers["/perception/traffic_light/stop_required"].messages

    def observe(self, objects, stamp=None, header=True):
        message = Message(objects=[SimpleNamespace(class_name=name, conf=conf) for name, conf in objects])
        if header:
            message.header = Header(seq=7, stamp=stamp, frame_id="camera_link")
        else:
            del message.header
        self.node.callback(message)
        self.ros.Time.now.assert_not_called()
        return self.states[-1]

    def test_unknown_during_and_after_legacy_clear_is_never_green(self):
        red = self.observe([("Red", 0.8)], Stamp(100.0))
        self.assertEqual((red.state, red.valid, red.confidence), ("RED", True, 0.8))
        self.clock.advance(0.1)
        unknown = self.observe([], Stamp(100.1))
        self.assertTrue(self.bools[-1].data)
        self.assertEqual((unknown.state, unknown.valid, unknown.confidence), ("UNKNOWN", False, 0.0))
        self.clock.advance(0.6)
        cleared = self.observe([], Stamp(100.7))
        self.assertFalse(self.bools[-1].data)
        self.assertEqual((cleared.state, cleared.valid), ("UNKNOWN", False))

    def test_green_priority_confidence_comes_only_from_green_evidence(self):
        self.observe([("Red", 0.99)], Stamp(100.0))
        green = self.observe([("Green_Left", 0.51), ("Green", 0.6), ("Red", 0.99), ("Yellow", 0.9)], Stamp(100.1))
        self.assertEqual((green.state, green.valid, green.confidence), ("GREEN", True, 0.6))
        self.assertFalse(self.bools[-1].data)

    def test_yellow_and_red_confidence_exclude_other_objects(self):
        yellow = self.observe([("Amber", 0.4), ("Red", 0.95), ("car", 0.99)], Stamp(1))
        self.assertEqual((yellow.state, yellow.valid, yellow.confidence), ("YELLOW", True, 0.4))
        red = self.observe([("Red", 0.5), ("car", 0.99)], Stamp(2))
        self.assertEqual((red.state, red.valid, red.confidence), ("RED", True, 0.5))
        self.assertTrue(self.bools[-1].data)

    def test_unrecognized_turn_classes_do_not_become_green_or_valid(self):
        for name in ("car", "traffic light", "Left", "Red_Left", "Red_Right"):
            with self.subTest(name=name):
                output = self.observe([(name, 0.99)], Stamp(100.0))
                self.assertEqual((output.state, output.valid, output.confidence), ("UNKNOWN", False, 0.0))
                self.assertFalse(self.bools[-1].data)
        red_with_turn = self.observe([("Red", 0.7), ("Left", 0.9)], Stamp(100.1))
        self.assertEqual(red_with_turn.state, "RED")
        self.assertFalse(self.bools[-1].data)  # Existing turn exception retained.

    def test_delayed_out_of_order_duplicate_and_zero_stamps_are_preserved(self):
        self.clock.seconds = 9000.0
        for stamp in (Stamp(100), Stamp(90), Stamp(90), Stamp()):
            with self.subTest(stamp=stamp.to_sec()):
                output = self.observe([("Green", 0.7)], stamp)
                self.assertEqual(output.header.stamp, stamp)
                self.assertEqual(output.header.seq, 7)
        no_header = self.observe([("Green", 0.7)], header=False)
        self.assertEqual(no_header.header.stamp, Stamp())

    def test_state_header_copy_and_startup_shutdown_do_not_fabricate_observations(self):
        self.assertEqual((self.states[0].state, self.states[0].valid, self.states[0].header.stamp), ("UNKNOWN", False, Stamp()))
        source = Header(seq=8, stamp=Stamp(12))
        output = self.node.state_message("GREEN", 0.7, True, source)
        self.assertIsNot(output.header, source)
        self.assertEqual(source.frame_id, "")
        self.assertEqual(output.header.frame_id, "front_camera")
        self.node.shutdown()
        self.assertEqual((self.states[-1].state, self.states[-1].valid, self.states[-1].header.stamp), ("UNKNOWN", False, Stamp()))
        self.ros.Time.now.assert_not_called()

    def test_invalid_scores_do_not_create_nonfinite_state_confidence(self):
        output = self.observe([("Green", float("nan")), ("Green", float("inf")), ("Green", -1.0), ("Red", 0.99)], Stamp(1))
        self.assertEqual((output.state, output.valid, output.confidence), ("GREEN", True, 0.0))


if __name__ == "__main__":
    unittest.main()
