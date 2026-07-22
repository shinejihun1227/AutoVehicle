#!/usr/bin/env python3
"""Standalone MORAI Front-camera monitor for manual-driving tests.

This process only receives and inspects camera packets. It never sends
CtrlCmd and never starts a driving controller.
"""

from __future__ import annotations

import argparse
import os
import time

import cv2
import numpy as np

from front_camera_perception import FrontCameraPerception
from front_camera_udp import FrontCameraUdpReceiver


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=1101)
    parser.add_argument("--save-dir", default="")
    parser.add_argument("--save-every", type=int, default=30)
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--expected-width", type=int, default=1280)
    parser.add_argument("--expected-height", type=int, default=720)
    args = parser.parse_args()

    receiver = FrontCameraUdpReceiver(args.bind_ip, args.port)
    perception = FrontCameraPerception(resize_width=640, process_rate_hz=15.0)
    save_dir = os.path.expanduser(args.save_dir)
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    frame_count = 0
    last_log = time.monotonic()
    print("Front camera monitor listening on {}:{}".format(args.bind_ip, args.port))
    print("No CtrlCmd is sent. Press Ctrl+C{}.".format(" or q" if args.display else ""))

    try:
        while True:
            for frame in receiver.receive_available():
                frame_count += 1
                observation = perception.process_jpeg(
                    frame.jpeg, frame.received_monotonic
                )

                if save_dir and frame_count % max(1, args.save_every) == 0:
                    path = os.path.join(
                        save_dir, "front_camera_{:06d}.jpg".format(frame_count)
                    )
                    with open(path, "wb") as output:
                        output.write(frame.jpeg)

                if args.display:
                    image = cv2.imdecode(
                        np.frombuffer(frame.jpeg, dtype=np.uint8),
                        cv2.IMREAD_COLOR,
                    )
                    if image is not None:
                        cv2.putText(
                            image,
                            "frames={} fragments={}".format(
                                frame_count, frame.fragment_count
                            ),
                            (20, 35),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 255, 0),
                            2,
                        )
                        cv2.imshow("MORAI Front Camera", image)
                        key = cv2.waitKey(1) & 0xFF
                        if key in (ord("q"), 27):
                            return

                now = time.monotonic()
                if now - last_log >= 1.0:
                    last_log = now
                    if observation is None:
                        print(
                            "frames={} fragments={} perception=waiting".format(
                                frame_count, frame.fragment_count
                            )
                        )
                    else:
                        print(
                            "frames={} fragments={} resolution={} expected={} "
                            "traffic={} stop_line={} lane_offset_px={}".format(
                                frame_count,
                                frame.fragment_count,
                                "{}x{}".format(
                                    observation.width, observation.height
                                ),
                                "{}x{}".format(
                                    args.expected_width, args.expected_height
                                ),
                                observation.traffic_state,
                                observation.stop_line_detected,
                                observation.lane_offset_px,
                            )
                        )
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\nStopping Front camera monitor")
    finally:
        receiver.close()
        if args.display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
