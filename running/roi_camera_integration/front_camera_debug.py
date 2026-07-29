#!/usr/bin/env python3
"""Standalone Front-camera validation tool.

Run this before changing ROI's controller.  It proves that port 1101,
MORAI packet reassembly and JPEG decoding work at the configured resolution.
"""

from __future__ import annotations

import argparse
import os
import time

from front_camera_perception import FrontCameraPerception, cv2
from front_camera_udp import FrontCameraUdpReceiver


def _diagnostic_summary(receiver, frame_count: int) -> str:
    assembler = receiver.assembler
    reasons = ",".join(
        "{}={}".format(name, count)
        for name, count in sorted(assembler.invalid_reason_counts.items())
    ) or "none"
    return (
        "datagrams={} frames={} invalid={} dropped={} pending={} "
        "invalid_reasons={} non_jpeg={} timeouts={} last_error={}"
    ).format(
        receiver.received_datagram_count,
        frame_count,
        assembler.invalid_packet_count,
        assembler.dropped_frame_count,
        assembler.pending_frame_count,
        reasons,
        assembler.non_jpeg_frame_count,
        assembler.expired_frame_count,
        assembler.last_error_reason or "none",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=1101)
    parser.add_argument("--save-dir", default="")
    parser.add_argument("--expected-width", type=int, default=1280)
    parser.add_argument("--expected-height", type=int, default=720)
    parser.add_argument("--log-period", type=float, default=1.0)
    parser.add_argument(
        "--display",
        action="store_true",
        help="show a live OpenCV window with the lane mask and centers",
    )
    args = parser.parse_args()

    if args.display and cv2 is None:
        raise RuntimeError("--display requires python3-opencv")

    receiver = FrontCameraUdpReceiver(args.bind_ip, args.port)
    perception = FrontCameraPerception(resize_width=640, process_rate_hz=15.0)
    if args.save_dir:
        os.makedirs(os.path.expanduser(args.save_dir), exist_ok=True)

    frame_count = 0
    last_log = time.monotonic()
    last_observation = None
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
                    if perception.last_debug_overlay is not None:
                        overlay_path = os.path.join(
                            os.path.expanduser(args.save_dir),
                            "front_camera_overlay_{:06d}.jpg".format(frame_count),
                        )
                        cv2.imwrite(overlay_path, perception.last_debug_overlay)

                if observation is not None:
                    last_observation = (frame, observation)
                    if args.display and perception.last_debug_overlay is not None:
                        cv2.imshow(
                            "MORAI Front Camera Lane Debug",
                            perception.last_debug_overlay,
                        )
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            raise KeyboardInterrupt

            now = time.monotonic()
            if now - last_log >= max(0.1, args.log_period):
                assembler = receiver.assembler
                diagnostic = _diagnostic_summary(receiver, frame_count)
                if last_observation is None:
                    print(diagnostic + " waiting_for_complete_frame=true")
                else:
                    frame, observation = last_observation
                    source_width = observation.source_width or observation.width
                    source_height = observation.source_height or observation.height
                    source_resolution = "{}x{}".format(source_width, source_height)
                    processing_resolution = "{}x{}".format(
                        observation.width, observation.height
                    )
                    expected = "{}x{}".format(args.expected_width, args.expected_height)
                    print(
                        "{} fragments={} source_resolution={} "
                        "processing_resolution={} expected={} "
                        "traffic={}({}) stop_line={} lane_offset_px={} "
                        "lane_confidence={:.2f} left_x={} right_x={}".format(
                            diagnostic,
                            frame.fragment_count,
                            source_resolution,
                            processing_resolution,
                            expected,
                            observation.traffic_state,
                            observation.traffic_score,
                            observation.stop_line_detected,
                            observation.lane_offset_px,
                            observation.lane_confidence,
                            observation.left_lane_x,
                            observation.right_lane_x,
                        )
                    )
                last_log = now
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\nStopping Front camera receiver")
    finally:
        receiver.close()
        if args.display and cv2 is not None:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
