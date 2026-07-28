#!/usr/bin/env python3
"""Standalone Front-camera validation tool.

Run this before changing ROI's controller.  It proves that port 1101,
MORAI packet reassembly and JPEG decoding work at the configured resolution.
"""

from __future__ import annotations

import argparse
import os
import time

from front_camera_perception import FrontCameraPerception
from front_camera_udp import FrontCameraUdpReceiver


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=1101)
    parser.add_argument("--save-dir", default="")
    parser.add_argument("--expected-width", type=int, default=1280)
    parser.add_argument("--expected-height", type=int, default=720)
    args = parser.parse_args()

    receiver = FrontCameraUdpReceiver(args.bind_ip, args.port)
    perception = FrontCameraPerception(resize_width=640, process_rate_hz=15.0)
    if args.save_dir:
        os.makedirs(os.path.expanduser(args.save_dir), exist_ok=True)

    frame_count = 0
    last_log = time.monotonic()
    print("Front camera listening on {}:{}".format(args.bind_ip, args.port))
    try:
        while True:
            frames = receiver.receive_available()
            for frame in frames:
                frame_count += 1
                observation = perception.process_jpeg(frame.jpeg, frame.received_monotonic)
                if args.save_dir and frame_count % 30 == 0:
                    path = os.path.join(
                        os.path.expanduser(args.save_dir),
                        "front_camera_{:06d}.jpg".format(frame_count),
                    )
                    with open(path, "wb") as output:
                        output.write(frame.jpeg)

                if observation is not None and time.monotonic() - last_log >= 1.0:
                    last_log = time.monotonic()
                    resolution = "{}x{}".format(observation.width, observation.height)
                    expected = "{}x{}".format(args.expected_width, args.expected_height)
                    print(
                        "frames={} fragments={} resolution={} expected={} "
                        "traffic={}({}) stop_line={} lane_offset_px={}".format(
                            frame_count,
                            frame.fragment_count,
                            resolution,
                            expected,
                            observation.traffic_state,
                            observation.traffic_score,
                            observation.stop_line_detected,
                            observation.lane_offset_px,
                        )
                    )
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\nStopping Front camera receiver")
    finally:
        receiver.close()


if __name__ == "__main__":
    main()

