#!/usr/bin/env python3
"""MORAI Front Camera UDP receiver.

This module parses the MORAI camera UDP packet format used by the 24.R2
NetworkModule examples.  It deliberately uses the packet Size, Index and
AI/EI tail fields instead of searching only for JPEG SOI/EOI markers.

The module is independent of ROS so it can be imported by ROI's raw-UDP
controller.  A completed JPEG can be published as sensor_msgs/CompressedImage
by the caller.
"""

from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


MORAI_CAMERA_HEADER = b"MOR"
CAMERA_HEADER_SIZE = 3
CAMERA_META_SIZE = 16  # seconds, nanoseconds, index, payload size
CAMERA_DATA_OFFSET = CAMERA_HEADER_SIZE + CAMERA_META_SIZE
CAMERA_TAIL_SIZE = 2
JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"
TAIL_MORE = b"AI"
TAIL_END = b"EI"


@dataclass(frozen=True)
class CameraFrame:
    """A complete JPEG frame reconstructed from one or more UDP datagrams."""

    timestamp_sec: int
    timestamp_nsec: int
    jpeg: bytes
    fragment_count: int
    received_monotonic: float

    @property
    def timestamp(self) -> float:
        return self.timestamp_sec + self.timestamp_nsec / 1_000_000_000.0


@dataclass
class _FrameAssembly:
    timestamp_sec: int
    timestamp_nsec: int
    chunks: Dict[int, bytes] = field(default_factory=dict)
    last_index: Optional[int] = None
    first_seen_monotonic: float = 0.0


class MoraiCameraPacketError(ValueError):
    """Raised when a MORAI camera datagram is structurally invalid."""


class MoraiCameraFrameAssembler:
    """Reassemble MORAI camera JPEG fragments by timestamp and packet index."""

    def __init__(self, frame_timeout_sec: float = 0.25) -> None:
        self.frame_timeout_sec = float(frame_timeout_sec)
        self._frames: Dict[Tuple[int, int], _FrameAssembly] = {}
        self.invalid_packet_count = 0
        self.dropped_frame_count = 0

    def feed(
        self, packet: bytes, now_monotonic: Optional[float] = None
    ) -> Optional[CameraFrame]:
        """Consume one UDP datagram and return a frame only when complete."""

        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        self._drop_expired(now)

        try:
            sec, nsec, index, payload_size, payload, tail = self._parse_packet(packet)
        except MoraiCameraPacketError:
            self.invalid_packet_count += 1
            return None

        if index < 0:
            self.invalid_packet_count += 1
            return None

        key = (sec, nsec)
        assembly = self._frames.get(key)
        if assembly is None:
            assembly = _FrameAssembly(
                timestamp_sec=sec,
                timestamp_nsec=nsec,
                first_seen_monotonic=now,
            )
            self._frames[key] = assembly

        assembly.chunks[index] = payload
        if tail == TAIL_END:
            assembly.last_index = index
        elif tail != TAIL_MORE:
            self.invalid_packet_count += 1
            self._frames.pop(key, None)
            return None

        if assembly.last_index is None:
            return None

        last_index = assembly.last_index
        expected_indexes = range(last_index + 1)
        if any(fragment_index not in assembly.chunks for fragment_index in expected_indexes):
            return None

        jpeg = b"".join(assembly.chunks[fragment_index] for fragment_index in expected_indexes)
        self._frames.pop(key, None)

        if not jpeg.startswith(JPEG_SOI) or not jpeg.endswith(JPEG_EOI):
            self.dropped_frame_count += 1
            return None

        return CameraFrame(
            timestamp_sec=sec,
            timestamp_nsec=nsec,
            jpeg=jpeg,
            fragment_count=last_index + 1,
            received_monotonic=now,
        )

    @staticmethod
    def _parse_packet(
        packet: bytes,
    ) -> Tuple[int, int, int, int, bytes, bytes]:
        minimum_size = CAMERA_DATA_OFFSET + CAMERA_TAIL_SIZE
        if len(packet) < minimum_size:
            raise MoraiCameraPacketError("camera packet is shorter than the header")
        if packet[:CAMERA_HEADER_SIZE] != MORAI_CAMERA_HEADER:
            raise MoraiCameraPacketError("camera packet header is not MOR")

        # MORAI's ctypes example uses native 32-bit integers.  Ubuntu/AMD64
        # is little-endian, so use an explicit little-endian layout here.
        sec, nsec, index, payload_size = struct.unpack_from("<4i", packet, 3)
        if payload_size <= 0:
            raise MoraiCameraPacketError("camera JPEG payload size is not positive")

        data_start = CAMERA_DATA_OFFSET
        data_end = data_start + payload_size
        tail_end = data_end + CAMERA_TAIL_SIZE
        if tail_end > len(packet):
            raise MoraiCameraPacketError("camera payload size exceeds datagram length")

        payload = packet[data_start:data_end]
        tail = packet[data_end:tail_end]
        return sec, nsec, index, payload_size, payload, tail

    def _drop_expired(self, now_monotonic: float) -> None:
        expired = [
            key
            for key, assembly in self._frames.items()
            if now_monotonic - assembly.first_seen_monotonic > self.frame_timeout_sec
        ]
        for key in expired:
            self._frames.pop(key, None)
            self.dropped_frame_count += 1


class FrontCameraUdpReceiver:
    """Non-blocking Front-camera UDP socket suitable for ROI's selector loop."""

    def __init__(
        self,
        bind_ip: str = "0.0.0.0",
        port: int = 1101,
        receive_buffer_bytes: int = 4 * 1024 * 1024,
        frame_timeout_sec: float = 0.25,
    ) -> None:
        self.bind_ip = bind_ip
        self.port = int(port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, receive_buffer_bytes)
        self.socket.bind((self.bind_ip, self.port))
        self.socket.setblocking(False)
        self.assembler = MoraiCameraFrameAssembler(frame_timeout_sec)
        self.last_frame: Optional[CameraFrame] = None
        self.last_sender: Optional[Tuple[str, int]] = None

    def fileno(self) -> int:
        return self.socket.fileno()

    def feed_datagram(
        self,
        packet: bytes,
        sender: Optional[Tuple[str, int]] = None,
        now_monotonic: Optional[float] = None,
    ) -> Optional[CameraFrame]:
        frame = self.assembler.feed(packet, now_monotonic)
        if frame is not None:
            self.last_frame = frame
            self.last_sender = sender
        return frame

    def receive_available(self, max_packets: int = 64) -> List[CameraFrame]:
        """Drain currently available datagrams for a standalone debug loop."""

        frames: List[CameraFrame] = []
        for _ in range(max_packets):
            try:
                packet, sender = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            frame = self.feed_datagram(packet, sender)
            if frame is not None:
                frames.append(frame)
        return frames

    def close(self) -> None:
        self.socket.close()

