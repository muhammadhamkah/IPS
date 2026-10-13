"""Read per-drone show files in the "XTJZ" binary format (one file per drone, as uploaded to
show drones by third-party show conversion software).

The layout was worked out from sample files; fields not listed are not needed here.

    header, 140 bytes:
      0  "XTJZ"
      4  u16  header size (140)        6  u16  1
      8  u32  body size (file size - 140)
      plain files:  12 u32 drone ID, 16 u16 position rate (Hz), 18 u16 position samples
      signed files: 12..139 is an opaque 128-byte block (the ID then comes from the file name)
    body: sections, each [u32 length][u8 channel][u8 encoding][u16 8][data]
      channels 1, 2, 3: x, y, z in cm, NED (z negative = above the floor), int16
      channels 5, 6, 7: red, green, blue 0..255, 18 samples per position sample
      encoding 1: raw values (int16 for positions, uint8 for colours)
      encoding 2: run-length deltas from 0: [u8 repeats - 1][delta] with an int16 delta for
                  positions and an int8 delta for colours (byte arithmetic: a colour change of
                  more than ±127 wraps around)

A file may also be gzipped (".dat.gz").
"""
from __future__ import annotations

import gzip
import re
import struct
from dataclasses import dataclass

import numpy as np

MAGIC = b"XTJZ"
HEADER = 140
COLOURS_PER_SAMPLE = 18
DEFAULT_RATE_HZ = 20.0


@dataclass
class XtjzDrone:
    drone_id: int | None
    rate_hz: float
    pos_ned: np.ndarray              # (n, 3) metres
    rgb: np.ndarray                  # (m, 3) 0..255
    signed: bool                     # the header is the opaque (signed) kind

    @property
    def t(self) -> np.ndarray:
        return np.arange(len(self.pos_ned)) / self.rate_hz

    @property
    def t_rgb(self) -> np.ndarray:
        return np.arange(len(self.rgb)) / (self.rate_hz * len(self.rgb) / max(len(self.pos_ned), 1))


def is_xtjz(data: bytes) -> bool:
    if data[:2] == b"\x1f\x8b":
        data = gzip.decompress(data)
    return data[:4] == MAGIC


def _channel(enc: int, data: bytes, wide: bool) -> np.ndarray:
    if enc == 1:
        return np.frombuffer(data, "<i2" if wide else "u1").astype(np.int64)
    if enc != 2:
        raise ValueError(f"unknown channel encoding {enc}")
    step, fmt = (3, "<h") if wide else (2, "<b")
    if len(data) % step:
        raise ValueError("truncated run-length channel")
    runs = np.frombuffer(data, "u1")[0::step].astype(np.int64) + 1
    deltas = np.array([struct.unpack_from(fmt, data, i + 1)[0] for i in range(0, len(data), step)], np.int64)
    v = np.cumsum(np.repeat(deltas, runs))
    return v if wide else v % 256                         # colours are bytes: changes wrap around


def read_xtjz(data: bytes, name: str = "") -> XtjzDrone:
    if data[:2] == b"\x1f\x8b":
        data = gzip.decompress(data)
    if data[:4] != MAGIC or len(data) < HEADER:
        raise ValueError(f"{name}: not an XTJZ show file")
    hsize, _, body = struct.unpack_from("<HHI", data, 4)
    if hsize != HEADER or body != len(data) - HEADER:
        raise ValueError(f"{name}: XTJZ header and file size disagree")
    sec = {}
    off = HEADER
    while off + 8 <= len(data):
        n, ch, enc = struct.unpack_from("<IBB", data, off)
        if n == 0:
            break
        if off + 8 + n > len(data):
            raise ValueError(f"{name}: truncated section {ch}")
        sec[ch] = (enc, data[off + 8:off + 8 + n])
        off += 8 + n
    missing = [c for c in (1, 2, 3) if c not in sec]
    if missing:
        raise ValueError(f"{name}: no position channel(s) {missing}")
    pos = np.stack([_channel(*sec[c], True) for c in (1, 2, 3)], 1) / 100.0
    rgb = (np.stack([_channel(*sec[c], False) for c in (5, 6, 7)], 1) if all(c in sec for c in (5, 6, 7))
           else np.full((len(pos), 3), 255))
    did, rate = struct.unpack_from("<IH", data, 12)
    n_hdr = struct.unpack_from("<H", data, 18)[0]
    plain = 0 < did < 1000 and 1 <= rate <= 100 and n_hdr == len(pos)
    if not plain:                                          # signed header: ID from the name ("3.dat")
        m = re.search(r"(\d+)(?:\.\w+)*$", name)
        did, rate = (int(m.group(1)) if m else None), DEFAULT_RATE_HZ
    return XtjzDrone(did, float(rate), pos, np.clip(rgb, 0, 255), not plain)


def show_from_xtjz(files: dict[str, bytes], source: str):
    """{file name: bytes} -> Show. Positions are converted to the show frame IDPS uses by default
    (x East, y North, z up: Placement axes "ENU"), so the venue origin is the file's origin."""
    from idps.show.model import Show, track_from_samples
    drones = []
    for name, data in files.items():
        d = read_xtjz(data, name)
        enu = np.column_stack([d.pos_ned[:, 1], d.pos_ned[:, 0], -d.pos_ned[:, 2]])
        label = f"Drone {d.drone_id}" if d.drone_id is not None else name
        drones.append(track_from_samples(label, d.t, enu, lights=(d.t_rgb, d.rgb.astype(float)),
                                         meta={"file": name, "rate_hz": d.rate_hz, "signed": d.signed}))
    drones.sort(key=lambda tr: int(re.sub(r"\D", "", tr.name) or 0))
    return Show(drones, source, "xtjz", {})
