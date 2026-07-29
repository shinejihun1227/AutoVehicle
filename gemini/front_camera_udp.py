#!/usr/bin/env python3
"""MORAI Front Camera UDP Receiver & Assembler."""

from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

MORAI_CAMERA_HEADER = b"MOR"
CAMERA_HEADER_SIZE = 3
CAMERA_META_SIZE = 16
CAMERA_DATA_OFFSET = CAMERA_HEADER_SIZE + CAMERA_META_SIZE
CAMERA_TAIL_SIZE = 2
JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"
TAIL_MORE = b"AI"
TAIL_END = b"EI"


@dataclass(frozen=True)
class CameraFrame:
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
    """Raised when a MORAI camera datagram is invalid."""


class MoraiCameraFrameAssembler:
    """Reassemble MORAI UDP camera fragments."""

    def __init__(self, frame_timeout_sec: float = 0.25) -> None:
        self.frame_timeout_sec = float(frame_timeout_sec)
        self._frames: Dict[Tuple[int, int], _FrameAssembly] = {}
        self.invalid_packet_count = 0
        self.dropped_frame_count = 0
        self.expired_frame_count = 0
        self.non_jpeg_frame_count = 0
        self.invalid_reason_counts: Dict[str, int] = {}
        self.last_error_reason: Optional[str] = None

    @property
    def pending_frame_count(self) -> int:
        return len(self._frames)

    def feed(self, packet: bytes, now_monotonic: Optional[float] = None) -> Optional[CameraFrame]:
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        self._drop_expired(now)

        try:
            sec, nsec, index, payload_size, payload, tail = self._parse_packet(packet)
        except MoraiCameraPacketError:
            self._record_invalid("packet_parse_error")
            return None

        if index < 0:
            self._record_invalid("negative_index")
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
            self._record_invalid("invalid_tail")
            self._frames.pop(key, None)
            return None

        if assembly.last_index is None:
            return None

        last_index = assembly.last_index
        expected_indexes = range(last_index + 1)
        if any(idx not in assembly.chunks for idx in expected_indexes):
            return None

        jpeg = b"".join(assembly.chunks[idx] for idx in expected_indexes)
        self._frames.pop(key, None)

        if not jpeg.startswith(JPEG_SOI) or not jpeg.endswith(JPEG_EOI):
            self.dropped_frame_count += 1
            self.non_jpeg_frame_count += 1
            self.last_error_reason = "non_jpeg_frame"
            return None

        return CameraFrame(
            timestamp_sec=sec,
            timestamp_nsec=nsec,
            jpeg=jpeg,
            fragment_count=last_index + 1,
            received_monotonic=now,
        )

    @staticmethod
    def _parse_packet(packet: bytes) -> Tuple[int, int, int, int, bytes, bytes]:
        if len(packet) < CAMERA_DATA_OFFSET + CAMERA_TAIL_SIZE:
            raise MoraiCameraPacketError("Short packet")
        if packet[:CAMERA_HEADER_SIZE] != MORAI_CAMERA_HEADER:
            raise MoraiCameraPacketError("Invalid header")

        sec, nsec, index, payload_size = struct.unpack_from("<4i", packet, 3)
        if payload_size <= 0:
            raise MoraiCameraPacketError("Invalid payload size")

        data_start = CAMERA_DATA_OFFSET
        data_end = data_start + payload_size
        if data_end > len(packet):
            raise MoraiCameraPacketError("Payload exceeds packet size")

        tail_candidates = (
            (data_end, data_end + CAMERA_TAIL_SIZE, data_end),
            (data_end - CAMERA_TAIL_SIZE, data_end, data_end - CAMERA_TAIL_SIZE),
            (len(packet) - CAMERA_TAIL_SIZE, len(packet), data_end),
        )
        payload_end = None
        tail = None
        for t_start, t_end, cand_p_end in tail_candidates:
            if t_start < data_start or t_end > len(packet):
                continue
            candidate = packet[t_start:t_end]
            if candidate in (TAIL_MORE, TAIL_END):
                payload_end = cand_p_end
                tail = candidate
                break

        if payload_end is None or tail is None:
            raise MoraiCameraPacketError("Invalid tail")

        return sec, nsec, index, payload_size, packet[data_start:payload_end], tail

    def _drop_expired(self, now_monotonic: float) -> None:
        expired = [
            k for k, v in self._frames.items()
            if now_monotonic - v.first_seen_monotonic > self.frame_timeout_sec
        ]
        for k in expired:
            self._frames.pop(k, None)
            self.dropped_frame_count += 1
            self.expired_frame_count += 1

    def _record_invalid(self, reason: str) -> None:
        self.invalid_packet_count += 1
        self.invalid_reason_counts[reason] = self.invalid_reason_counts.get(reason, 0) + 1
        self.last_error_reason = reason


class FrontCameraUdpReceiver:
    """Non-blocking UDP receiver socket."""

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
        self.received_datagram_count = 0
        self.last_frame: Optional[CameraFrame] = None

    def feed_datagram(self, packet: bytes, now_monotonic: Optional[float] = None) -> Optional[CameraFrame]:
        self.received_datagram_count += 1
        frame = self.assembler.feed(packet, now_monotonic)
        if frame is not None:
            self.last_frame = frame
        return frame

    def receive_available(self, max_packets: int = 64) -> List[CameraFrame]:
        frames: List[CameraFrame] = []
        for _ in range(max_packets):
            try:
                packet, _ = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            frame = self.feed_datagram(packet)
            if frame is not None:
                frames.append(frame)
        return frames

    def close(self) -> None:
        self.socket.close()
