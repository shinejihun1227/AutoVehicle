#!/usr/bin/env python3
"""MORAI Front Camera Receiver & Perception Main Pipeline."""

from __future__ import annotations

import argparse
import os
import time

try:
    import cv2
except ImportError:
    cv2 = None

# 위에서 정의했던 모듈들 import
from front_camera_perception import FrontCameraPerception
from front_camera_udp import FrontCameraUdpReceiver


def main() -> None:
    parser = argparse.ArgumentParser(description="MORAI Front Camera Main Pipeline")
    parser.add_argument("--bind-ip", default="0.0.0.0", help="UDP Bind IP")
    parser.add_argument("--port", type=int, default=1101, help="UDP Camera Port")
    parser.add_argument("--save-dir", default="", help="Directory to save debug images")
    parser.add_argument("--log-period", type=float, default=1.0, help="Diagnostic log interval (sec)")
    parser.add_argument("--display", action="store_true", help="Show live OpenCV window")
    args = parser.parse_args()

    if args.display and cv2 is None:
        raise RuntimeError("--display requires opencv-python")

    # 1. Receiver 및 Perception 객체 생성
    receiver = FrontCameraUdpReceiver(bind_ip=args.bind_ip, port=args.port)
    perception = FrontCameraPerception(resize_width=640, process_rate_hz=15.0)

    if args.save_dir:
        os.makedirs(os.path.expanduser(args.save_dir), exist_ok=True)

    frame_count = 0
    last_log_time = time.monotonic()
    
    print(f"[*] Front Camera Receiver started on {args.bind_ip}:{args.port}")

    try:
        while True:
            # 2. 소켓에 쌓인 UDP 패킷들을 꺼내서 조립
            frames = receiver.receive_available(max_packets=64)
            
            for frame in frames:
                frame_count += 1
                
                # 3. 완벽히 조립된 JPEG 프레임을 Perception 모듈로 전달
                observation = perception.process_jpeg(frame.jpeg, frame.received_monotonic)

                # 주기적으로 디버그 이미지 저장 (--save-dir 사용 시)
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

                # 4. 실시간 시각화 (--display 사용 시)
                if args.display and perception.last_debug_overlay is not None:
                    cv2.imshow("MORAI Front Camera Perception", perception.last_debug_overlay)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        raise KeyboardInterrupt

            # 5. 진단용 로그 주기적 출력
            now = time.monotonic()
            if now - last_log_time >= max(0.1, args.log_period):
                assembler = receiver.assembler
                print(
                    f"[LOG] Recv Datagrams={receiver.received_datagram_count} | "
                    f"Frames={frame_count} | Drop={assembler.dropped_frame_count} | "
                    f"Pending={assembler.pending_frame_count}"
                )
                last_log_time = now

            # CPU 과점유 방지
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[*] Stopping pipeline...")
    finally:
        receiver.close()
        if args.display and cv2 is None:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
