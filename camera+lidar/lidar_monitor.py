#!/usr/bin/env python3
"""Standalone VLP16 UDP monitor for MORAI manual-driving tests.

The monitor parses VLP16 packets, reports packet/point rates, computes a
front-ROI nearest distance, and optionally saves point-cloud snapshots as
CSV. It does not publish ROS topics and does not send vehicle commands.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import socket
import struct
import time
from typing import Iterable, List, Tuple


VLP16_VERTICAL_ANGLES_DEG = [
    -15, 1, -13, 3, -11, 5, -9, 7,
    -7, 9, -5, 11, -3, 13, -1, 15,
]
Point = Tuple[float, float, float, float, int]


def parse_vlp16_packet(
    payload: bytes,
    min_distance_m: float,
    max_distance_m: float,
    azimuth_offset_deg: float,
    invert_y: bool,
    invert_z: bool,
) -> List[Point]:
    if len(payload) < 1200:
        return []

    vertical_angles = [
        math.radians(value) for value in VLP16_VERTICAL_ANGLES_DEG
    ]
    azimuth_offset = math.radians(azimuth_offset_deg)
    points: List[Point] = []

    # A VLP16 data packet has 12 blocks x 100 bytes. The remaining bytes
    # contain timestamp/factory data and are not needed for point decoding.
    for block_index in range(12):
        block_offset = block_index * 100
        flag = struct.unpack_from("<H", payload, block_offset)[0]
        if flag != 0xEEFF:
            continue

        azimuth_raw = struct.unpack_from(
            "<H", payload, block_offset + 2
        )[0]
        azimuth = math.radians(azimuth_raw / 100.0) + azimuth_offset

        for channel in range(32):
            data_offset = block_offset + 4 + channel * 3
            distance_raw = struct.unpack_from(
                "<H", payload, data_offset
            )[0]
            distance_m = distance_raw * 0.002
            if not min_distance_m <= distance_m <= max_distance_m:
                continue

            intensity = float(payload[data_offset + 2])
            ring = channel % 16
            vertical = vertical_angles[ring]
            horizontal_distance = distance_m * math.cos(vertical)
            x = horizontal_distance * math.cos(azimuth)
            y = horizontal_distance * math.sin(azimuth)
            z = distance_m * math.sin(vertical)
            if invert_y:
                y = -y
            if invert_z:
                z = -z
            points.append((x, y, z, intensity, ring))

    return points


def nearest_front_distance(
    points: Iterable[Point],
    x_min: float,
    x_max: float,
    y_abs_max: float,
    z_min: float,
    z_max: float,
) -> Tuple[float, int]:
    selected = [
        point
        for point in points
        if x_min <= point[0] <= x_max
        and abs(point[1]) <= y_abs_max
        and z_min <= point[2] <= z_max
    ]
    if not selected:
        return float("inf"), 0
    return min(math.hypot(point[0], point[1]) for point in selected), len(selected)


def save_csv(path: str, points: Iterable[Point]) -> None:
    with open(path, "w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(["x_m", "y_m", "z_m", "intensity", "ring"])
        writer.writerows(points)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=2001)
    parser.add_argument("--packets-per-scan", type=int, default=76)
    parser.add_argument("--save-dir", default="")
    parser.add_argument("--save-every-scans", type=int, default=1)
    parser.add_argument("--min-distance-m", type=float, default=0.2)
    parser.add_argument("--max-distance-m", type=float, default=120.0)
    parser.add_argument("--azimuth-offset-deg", type=float, default=0.0)
    parser.add_argument("--invert-y", action="store_true")
    parser.add_argument("--invert-z", action="store_true")
    parser.add_argument("--front-x-min-m", type=float, default=0.5)
    parser.add_argument("--front-x-max-m", type=float, default=30.0)
    parser.add_argument("--front-y-abs-max-m", type=float, default=3.0)
    parser.add_argument("--front-z-min-m", type=float, default=-1.5)
    parser.add_argument("--front-z-max-m", type=float, default=2.0)
    args = parser.parse_args()

    save_dir = os.path.expanduser(args.save_dir)
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind((args.bind_ip, args.port))
    sock.settimeout(0.5)

    packet_count = 0
    valid_packet_count = 0
    scan_count = 0
    scan_points: List[Point] = []
    source = ("?", 0)
    last_packet_length = 0
    last_status_log = time.monotonic()
    print("VLP16 LiDAR monitor listening on {}:{}".format(args.bind_ip, args.port))
    print("No ROS topics or CtrlCmd are used. Press Ctrl+C to stop.")

    try:
        while True:
            try:
                payload, source = sock.recvfrom(2048)
            except socket.timeout:
                now = time.monotonic()
                if now - last_status_log >= 2.0:
                    last_status_log = now
                    print(
                        "waiting for LiDAR packets: packets={} valid={} "
                        "last_bytes={} source={}:{}".format(
                            packet_count,
                            valid_packet_count,
                            last_packet_length,
                            source[0],
                            source[1],
                        )
                    )
                continue

            packet_count += 1
            last_packet_length = len(payload)
            points = parse_vlp16_packet(
                payload,
                args.min_distance_m,
                args.max_distance_m,
                args.azimuth_offset_deg,
                args.invert_y,
                args.invert_z,
            )
            if points:
                valid_packet_count += 1
                scan_points.extend(points)

            now = time.monotonic()
            if now - last_status_log >= 2.0:
                last_status_log = now
                print(
                    "LiDAR UDP status: packets={} valid={} last_bytes={} "
                    "source={}:{}".format(
                        packet_count,
                        valid_packet_count,
                        last_packet_length,
                        source[0],
                        source[1],
                    )
                )

            if packet_count % max(1, args.packets_per_scan) != 0:
                continue

            scan_count += 1
            nearest, roi_count = nearest_front_distance(
                scan_points,
                args.front_x_min_m,
                args.front_x_max_m,
                args.front_y_abs_max_m,
                args.front_z_min_m,
                args.front_z_max_m,
            )
            nearest_text = "none" if math.isinf(nearest) else "{:.2f} m".format(nearest)
            print(
                "scan={} packets={} points={} front_roi_points={} "
                "nearest_front={} source={}:{}".format(
                    scan_count,
                    packet_count,
                    len(scan_points),
                    roi_count,
                    nearest_text,
                    source[0],
                    source[1],
                )
            )

            if (
                save_dir
                and scan_count % max(1, args.save_every_scans) == 0
            ):
                path = os.path.join(
                    save_dir, "vlp16_scan_{:06d}.csv".format(scan_count)
                )
                save_csv(path, scan_points)

            scan_points = []
    except KeyboardInterrupt:
        print("\nStopping LiDAR monitor")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
