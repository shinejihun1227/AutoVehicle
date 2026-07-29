#!/usr/bin/env python3
"""MORAI Front Camera Perception Main Pipeline Script."""

from __future__ import annotations

import argparse
import os
import time

try:
    import cv2
except ImportError:
    cv2 = None

from front_camera_perception import FrontCameraPerception
from front_camera_udp import FrontCameraUdpReceiver


def main() -> None:
    parser = argparse.ArgumentParser(description="MORAI Front Camera Advanced Perception Pipeline")
    parser.add_argument("--bind-ip", default="0.0.0.0", help="UDP Bind IP (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=1101, help="UDP Camera Port (default: 1101)")
    parser.add_argument("--save-dir", default="", help="Directory path to save debug images")
    parser.add_argument("--log-period", type=float, default=1.0, help="Diagnostic log interval in seconds")
    parser.add_argument("--display", action="store_true", help="Display live OpenCV visualization window")
    args = parser.parse_args()

    if args.display and cv2 is None:
        raise RuntimeError("--display requires python3-opencv")

    # Receiver 및 업그레이드된 Perception 생성
    receiver = FrontCameraUdpReceiver(bind_ip=args.bind_ip, port=args.port)
    perception = FrontCameraPerception(resize_width=640, process_rate_hz=15.0)

    if args.save_dir:
        os.makedirs(os.path.expanduser(args.save_dir), exist_ok=True)

    frame_count = 0
    last_log_time = time.monotonic()

    print(f"[*] MORAI Camera Receiver Listening on {args.bind_ip}:{args.port}")

    try:
        while True:
            # 1. UDP 소켓 버퍼 비우기 및 조립 완료 프레임 수신
            frames = receiver.receive_available(max_packets=64)

            for frame in frames:
                frame_count += 1

                # 2. 전처리 + BEV + Sobel + Hough + RANSAC 통합 인지 연산 수행
                observation = perception.process_jpeg(frame.jpeg, frame.received_monotonic)

                # 3. 주기적 이미지 저장 (--save-dir)
                if args.save_dir and frame_count % 30 == 0:
                    save_path = os.path.join(
                        os.path.expanduser(args.save_dir), f"frame_{frame_count:06d}.jpg"
                    )
                    with open(save_path, "wb") as f:
                        f.write(frame.jpeg)

                    if perception.last_debug_overlay is not None:
                        overlay_path = os.path.join(
                            os.path.expanduser(args.save_dir), f"overlay_{frame_count:06d}.jpg"
                        )
                        cv2.imwrite(overlay_path, perception.last_debug_overlay)

                # 4. 실시간 시각화 화면 창 (--display)
                if args.display and perception.last_debug_overlay is not None:
                    cv2.imshow("MORAI Front Camera Advanced Perception", perception.last_debug_overlay)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        raise KeyboardInterrupt

            # 5. 진단 및 상태 로그 주기적 출력
            now = time.monotonic()
            if now - last_log_time >= max(0.1, args.log_period):
                assembler = receiver.assembler
                print(
                    f"[STATUS] Datagrams={receiver.received_datagram_count} | "
                    f"Processed Frames={frame_count} | "
                    f"Dropped={assembler.dropped_frame_count} | "
                    f"Pending Chunks={assembler.pending_frame_count}"
                )
                last_log_time = now

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[*] Stopping Front Camera Receiver Pipeline.")
    finally:
        receiver.close()
        if args.display and cv2 is not None:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
