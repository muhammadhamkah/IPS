"""``.trj``: the per-drone show file stored on the drone's SD card (decision D22).

The PX4 module ``idps_show`` (``px4/idps_show``) reads it; ``idps-show export`` writes one
per drone ID next to the JSON file, and ``idps-show upload`` copies each drone's file to
``/fs/microsd/idps/show.trj`` on that drone. Little-endian, fixed size, so the module reads it
without parsing:

  header, 64 bytes
    0  char[8]  magic "IDPSTRJ1"
    8  u16      version (1)
   10  u16      header size (64)
   12  u16      drone ID (= PX4 MAV_SYS_ID; the module refuses a file for another drone)
   14  u16      flags (0)
   16  f32      sample rate (Hz)
   20  u32      sample count
   24  f32      show time of the first sample (s)
   28  f32[3]   home N, E, D (m)
   40  f32      yaw (rad), fixed for the whole show (0 = North)
   44  u32      CRC-32 of the sample block
   48  u8       batch: take-off batch (1..) from a charging pad, 0 = the show starts on the ground
   49  u8       batches in the show
   50  u16      gap between batches (ms)
   52  u16      climb speed (mm/s), 54 u16 transfer speed (mm/s): see idps.show.pads
   56  u16      hold timeout (s): no start this long after the last batch took off: land
   58  u16      pad facing (0.01°): the way its pad's front points, clockwise from North
   60  u32      CRC-32 of bytes 0..59

  samples, 40 bytes each, at the sample rate from the first sample's time
    f32[3] position N, E, D (m); f32[3] velocity (m/s); f32[3] acceleration (m/s²);
    u8[3] LED colour R, G, B; u8 reserved

CRC-32 is PX4's ``crc32part`` (IEEE polynomial, start 0, no final inversion), the same one
PX4's MAVLink FTP uses for "calculate file CRC", so the PC checks an upload with it too.
A 10-minute show at 20 Hz is about 470 kB.
"""
from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np

MAGIC = b"IDPSTRJ1"
VERSION = 1
HEADER = struct.Struct("<8sHHHHfIf3ffIBBHHHHH")      # 60 bytes, then the header CRC
HEADER_SIZE = HEADER.size + 4
SAMPLE = np.dtype([("pos", "<f4", 3), ("vel", "<f4", 3), ("acc", "<f4", 3), ("rgb", "u1", 3), ("pad", "u1")])
assert HEADER_SIZE == 64 and SAMPLE.itemsize == 40


def crc32(data: bytes, crc: int = 0) -> int:
    """PX4 crc32part: zlib's CRC-32 without the start/end inversion."""
    return zlib.crc32(data, crc ^ 0xFFFFFFFF) ^ 0xFFFFFFFF


@dataclass
class Trj:
    drone_id: int
    rate_hz: float
    t0: float
    home: np.ndarray
    yaw_rad: float
    pos: np.ndarray            # (n, 3) NED
    vel: np.ndarray
    acc: np.ndarray
    rgb: np.ndarray            # (n, 3) 0..255
    batch: int = 0             # take-off from a charging pad (idps.show.pads); 0: from the ground
    n_batches: int = 0
    batch_gap_s: float = 0.0
    climb_speed_mps: float = 0.0
    transfer_speed_mps: float = 0.0
    hold_timeout_s: float = 0.0
    pad_facing_deg: float = 0.0

    @property
    def duration(self) -> float:
        return (len(self.pos) - 1) / self.rate_hz


def to_bytes(t: Trj) -> bytes:
    n = len(t.pos)
    s = np.zeros(n, SAMPLE)
    s["pos"], s["vel"], s["acc"] = t.pos, t.vel, t.acc
    s["rgb"] = np.clip(np.rint(t.rgb), 0, 255).astype(np.uint8)
    body = s.tobytes()
    head = HEADER.pack(MAGIC, VERSION, HEADER_SIZE, int(t.drone_id), 0, float(t.rate_hz), n, float(t.t0),
                       *[float(x) for x in t.home], float(t.yaw_rad), crc32(body), int(t.batch), int(t.n_batches),
                       int(round(t.batch_gap_s * 1000)), int(round(t.climb_speed_mps * 1000)),
                       int(round(t.transfer_speed_mps * 1000)), int(round(t.hold_timeout_s)),
                       int(round((t.pad_facing_deg % 360.0) * 100)) % 36000)
    return head + struct.pack("<I", crc32(head)) + body


def from_bytes(b: bytes) -> Trj:
    if len(b) < HEADER_SIZE or b[:8] != MAGIC:
        raise ValueError("not an IDPS .trj file")
    head = b[:HEADER.size]
    (crc_h,) = struct.unpack_from("<I", b, HEADER.size)
    if crc32(head) != crc_h:
        raise ValueError(".trj header CRC mismatch")
    _, ver, hsize, did, _, rate, n, t0, hn, he, hd, yaw, crc_s, batch, nb, gap, climb, xfer, hold, facing = \
        HEADER.unpack(head)
    if ver != VERSION:
        raise ValueError(f".trj version {ver} not supported")
    body = b[hsize:hsize + n * SAMPLE.itemsize]
    if len(body) != n * SAMPLE.itemsize or crc32(body) != crc_s:
        raise ValueError(".trj samples truncated or corrupt")
    s = np.frombuffer(body, SAMPLE)
    return Trj(did, rate, t0, np.array([hn, he, hd]), yaw, s["pos"].astype(float), s["vel"].astype(float),
               s["acc"].astype(float), s["rgb"].astype(float), batch, nb, gap / 1000, climb / 1000, xfer / 1000,
               float(hold), facing / 100)


def write_trj(path, t: Trj) -> bytes:
    b = to_bytes(t)
    Path(path).write_bytes(b)
    return b


def read_trj(path) -> Trj:
    return from_bytes(Path(path).read_bytes())
