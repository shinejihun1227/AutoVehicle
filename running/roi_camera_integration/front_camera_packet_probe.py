#!/usr/bin/env python3
"""Print the raw layout of MORAI camera UDP datagrams.

This tool intentionally does not try to decode a frame. It is used when the
debug receiver reports datagrams but cannot assemble a JPEG.
"""

from __future__ import annotations

import argparse
import socket
import struct
from typing import Iterable


META_OFFSET = 3
DATA_OFFSET = 19


def _positions(packet: bytes, marker: bytes) -> str:
    positions = []
    start = 0
    while len(positions) < 8:
        position = packet.find(marker, start)
        if position < 0:
            break
        positions.append(position)
        start = position + 1
    if not positions:
        return "none"
    suffix = ",..." if packet.find(marker, start) >= 0 else ""
    return ",".join(str(position) for position in positions) + suffix


def _unpack(packet: bytes, endian: str) -> str:
    if len(packet) < DATA_OFFSET:
        return "unpack=short"
    sec, nsec, index, size = struct.unpack_from(endian + "4I", packet, META_OFFSET)
    declared_end = DATA_OFFSET + size
    return (
        "{}sec={} nsec={} index={} size={} declared_end={} length_delta={}"
    ).format(
        "little_" if endian == "<" else "big_",
        sec,
        nsec,
        index,
        size,
        declared_end,
        len(packet) - declared_end,
    )


def _format_packet(packet: bytes, sender: tuple[str, int]) -> Iterable[str]:
    yield "sender={} length={} head={!r} tail={!r}".format(
        sender,
        len(packet),
        packet[:3],
        packet[-2:] if len(packet) >= 2 else packet,
    )
    yield "head_hex={} tail_hex={} last32_hex={}".format(
        packet[:32].hex(),
        packet[-2:].hex() if len(packet) >= 2 else packet.hex(),
        packet[-32:].hex(),
    )
    yield _unpack(packet, "<")
    yield _unpack(packet, ">")
    yield (
        "markers jpeg_soi={} jpeg_eoi={} AI={} EI={}".format(
            _positions(packet, b"\xff\xd8"),
            _positions(packet, b"\xff\xd9"),
            _positions(packet, b"AI"),
            _positions(packet, b"EI"),
        )
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=1101)
    parser.add_argument("--count", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind_ip, args.port))
    sock.settimeout(args.timeout)
    print("Probing MORAI camera UDP on {}:{}".format(args.bind_ip, args.port))
    try:
        for number in range(max(1, args.count)):
            packet, sender = sock.recvfrom(65535)
            print("--- datagram {} ---".format(number + 1))
            for line in _format_packet(packet, sender):
                print(line)
    except socket.timeout:
        print("timeout: no datagram received within {:.1f}s".format(args.timeout))
    finally:
        sock.close()


if __name__ == "__main__":
    main()
