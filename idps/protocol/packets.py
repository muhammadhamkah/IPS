"""Binary UDP packets: common header, FRAME_OBS, HEALTH (docs/PROTOCOL.md sections 1-3).

All integers little-endian. Layouts are fixed and must match
protocol/idps_protocol.h; tests/test_protocol.py checks the golden vectors in
protocol/vectors/.
"""
from __future__ import annotations

import json
import struct
from dataclasses import dataclass, field

MAGIC = 0x53504449            # b"IDPS" read as little-endian u32
VERSION = 1
MSG_FRAME_OBS = 1
MSG_HEALTH = 2

FLAG_PART = 1 << 0
FLAG_LAST_PART = 1 << 1
FLAG_SIMULATED = 1 << 2
FLAG_UNCALIBRATED = 1 << 3

MFLAG_SATURATED = 1 << 0
MFLAG_EDGE = 1 << 1
MFLAG_ELONGATED = 1 << 2
MFLAG_ROI = 1 << 3

SYNC_FREE_RUN = 0
SYNC_PTP = 1
SYNC_HW_TRIGGER = 2
SYNC_HOLDOVER = 3

HEADER = struct.Struct("<IBBHIHH")             # 16 bytes
FRAME = struct.Struct("<QqIHBBHHHH")           # 32 bytes
MARKER = struct.Struct("<ffIHBB")              # 16 bytes
MAX_UDP_PAYLOAD = 1472                         # 1500-byte MTU minus IPv4 + UDP headers
MAX_MARKERS_PER_PACKET = (MAX_UDP_PAYLOAD - HEADER.size - FRAME.size) // MARKER.size   # 89

assert HEADER.size == 16 and FRAME.size == 32 and MARKER.size == 16


class ProtocolError(ValueError):
    pass


@dataclass
class Header:
    msg_type: int
    station_id: int
    seq: int
    payload_len: int
    flags: int = 0
    version: int = VERSION


@dataclass
class Marker:
    x: float                 # distorted image coordinates, pixels
    y: float
    sum: int                 # background-subtracted integrated intensity (DN)
    peak: int                # peak pixel value (DN)
    area: int                # pixels above the low threshold, saturating at 255
    mflags: int = 0


@dataclass
class FrameObs:
    station_id: int
    frame_id: int
    t_mid_ns: int            # mid-exposure, system time base
    exposure_ns: int
    markers: list[Marker] = field(default_factory=list)
    proc_us: int = 0
    sync_state: int = SYNC_FREE_RUN
    threshold: int = 0
    drops_since_last: int = 0
    n_markers_total: int | None = None
    part_index: int = 0
    flags: int = 0


def encode_frame(obs: FrameObs, seq_start: int) -> list[bytes]:
    """Encode a frame; frames with more than 89 markers are split into parts
    (each part repeats the fixed fields). Returns one bytes object per packet;
    the caller's next sequence number is seq_start + len(result)."""
    total = len(obs.markers)
    chunks = [obs.markers[i:i + MAX_MARKERS_PER_PACKET] for i in range(0, total, MAX_MARKERS_PER_PACKET)] or [[]]
    out = []
    for k, chunk in enumerate(chunks):
        flags = obs.flags
        if len(chunks) > 1:
            flags |= FLAG_PART | (FLAG_LAST_PART if k == len(chunks) - 1 else 0)
        body = FRAME.pack(obs.frame_id, obs.t_mid_ns, obs.exposure_ns, min(obs.proc_us, 0xFFFF), obs.sync_state,
                          k, len(chunk), min(total, 0xFFFF), obs.threshold, min(obs.drops_since_last, 0xFFFF))
        body += b"".join(MARKER.pack(m.x, m.y, int(m.sum) & 0xFFFFFFFF, int(m.peak) & 0xFFFF,
                                     min(int(m.area), 255), m.mflags) for m in chunk)
        hdr = HEADER.pack(MAGIC, VERSION, MSG_FRAME_OBS, obs.station_id, (seq_start + k) & 0xFFFFFFFF, len(body),
                          flags)
        out.append(hdr + body)
    return out


def encode_health(station_id: int, seq: int, payload: dict, flags: int = 0) -> bytes:
    body = json.dumps(payload, separators=(",", ":")).encode()
    if len(body) > MAX_UDP_PAYLOAD - HEADER.size:
        raise ProtocolError("health payload too large")
    return HEADER.pack(MAGIC, VERSION, MSG_HEALTH, station_id, seq & 0xFFFFFFFF, len(body), flags) + body


def decode_header(buf: bytes) -> Header:
    if len(buf) < HEADER.size:
        raise ProtocolError("short packet")
    magic, ver, mtype, sid, seq, plen, flags = HEADER.unpack_from(buf)
    if magic != MAGIC:
        raise ProtocolError("bad magic")
    if ver != VERSION:
        raise ProtocolError(f"unsupported version {ver}")
    if plen != len(buf) - HEADER.size:
        raise ProtocolError("payload length mismatch")
    return Header(mtype, sid, seq, plen, flags, ver)


def decode(buf: bytes):
    """Returns (Header, FrameObs | dict). Raises ProtocolError on malformed input."""
    h = decode_header(buf)
    if h.msg_type == MSG_FRAME_OBS:
        if h.payload_len < FRAME.size:
            raise ProtocolError("short frame")
        (frame_id, t_mid, exp, proc, sync, part, n, n_total, thr, drops) = FRAME.unpack_from(buf, HEADER.size)
        if h.payload_len != FRAME.size + n * MARKER.size:
            raise ProtocolError("marker count mismatch")
        off = HEADER.size + FRAME.size
        markers = [Marker(*MARKER.unpack_from(buf, off + i * MARKER.size)) for i in range(n)]
        return h, FrameObs(h.station_id, frame_id, t_mid, exp, markers, proc, sync, thr, drops, n_total, part, h.flags)
    if h.msg_type == MSG_HEALTH:
        try:
            return h, json.loads(buf[HEADER.size:].decode())
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ProtocolError(f"bad health payload: {exc}") from None
    raise ProtocolError(f"unknown msg_type {h.msg_type}")
