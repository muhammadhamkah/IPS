"""Read per-drone show files in the "XTJZ" binary format (one file per drone, as uploaded to
show drones by third-party show conversion software).

The layout was worked out from sample files; fields not listed are not needed here.

    header, 140 bytes:
      0  "XTJZ"
      4  u16  header size (140)        6  u16  1
      8  u32  body size (file size - 140)
      plain files:  12 u32 drone ID, 16 u16 20 (meaning unknown; not the rate), 18 u16 position samples,
                    23 u8 colour samples per second (90 or 180: 18 per position sample, so 5 or 10 positions
                    per second; a show's summary file gives the same: frames / duration)
      signed files: 12..139 is an opaque 128-byte block (the ID then comes from the file name)
    body: sections, each [u32 length][u8 channel][u8 encoding][u16 8][data]
      channels 1, 2, 3: x, y, z in cm, NED (z negative = above the floor), int16, 10 per second
                        (confirmed by the length of a show as flown)
      channels 5, 6, 7: red, green, blue 0..255, 18 samples per position sample (180 per second)
      encoding 0 or 1: raw values (int16 for positions, uint8 for colours)
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
RATE_HZ = 10.0                  # position samples per second


@dataclass
class XtjzDrone:
    drone_id: int | None
    rate_hz: float
    pos_ned: np.ndarray              # (n, 3) metres
    rgb: np.ndarray                  # (m, 3) 0..255
    signed: bool                     # the header is the opaque (signed) kind
    rate_known: bool = False         # the rate is the file's own or was given (not the default)

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
    if enc in (0, 1):
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


def read_xtjz(data: bytes, name: str = "", rate_hz: float | None = None) -> XtjzDrone:
    """rate_hz: position samples per second, if known (a show's summary); else a plain header's own
    (its colour rate / 18), else RATE_HZ."""
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
    did = struct.unpack_from("<I", data, 12)[0]
    n_hdr = struct.unpack_from("<H", data, 18)[0]
    plain = 0 < did < 1000 and n_hdr == len(pos)
    if not plain:                                          # signed header: ID from the name ("3.dat")
        m = re.search(r"(\d+)(?:\.\w+)*$", name)
        did = int(m.group(1)) if m else None
    own = None
    if plain and data[23] and data[23] % COLOURS_PER_SAMPLE == 0:      # colour samples per second
        own = data[23] / COLOURS_PER_SAMPLE
    return XtjzDrone(did, float(rate_hz or own or RATE_HZ), pos, np.clip(rgb, 0, 255), not plain,
                     rate_known=bool(rate_hz or own))


def rate_from_summary(text: str) -> float | None:
    """Position samples per second from a show's summary CSV (columns drone_id, duration "m:ss.s",
    frames, ...): frames / duration. None if it is not such a file."""
    lines = [ln for ln in text.splitlines() if ln.strip()]
    if len(lines) < 2:
        return None
    head = [h.strip().lower() for h in lines[0].split(",")]
    if "duration" not in head or "frames" not in head:
        return None
    row = lines[1].split(",")
    try:
        dur = row[head.index("duration")].strip()
        secs = sum(float(x) * 60 ** k for k, x in enumerate(reversed(dur.split(":"))))
        frames = float(row[head.index("frames")])
    except (ValueError, IndexError):
        return None
    return round(frames / secs, 3) if secs > 0 and frames > 1 else None


def show_from_xtjz(files: dict[str, bytes], source: str, rate_hz: float | None = None):
    """{file name: bytes} -> Show. Positions are converted to the show frame IDPS uses by default
    (x East, y North, z up: Placement axes "ENU"), so the venue origin is the file's origin.

    A download folder holds each drone several times (plain, run-length coded, gzipped): one
    file per drone ID is used, a plain one if there is one. The rate: a plain file's own (it also
    serves the signed ones), else rate_hz (the show's summary: frames / duration), else RATE_HZ."""
    from idps.show.model import Show, track_from_samples
    parsed = [(name, read_xtjz(data, name)) for name, data in files.items()]
    own = sorted({d.rate_hz for _, d in parsed if d.rate_known}) or ([float(rate_hz)] if rate_hz else [])
    best: dict = {}
    for name, d in parsed:
        key = d.drone_id if d.drone_id is not None else name
        if key not in best or (best[key][1].signed and not d.signed):
            best[key] = (name, d)
    drones = []
    for name, d in best.values():
        rate = d.rate_hz if d.rate_known or not own else own[0]
        t = np.arange(len(d.pos_ned)) / rate
        t_rgb = np.arange(len(d.rgb)) / (rate * len(d.rgb) / max(len(d.pos_ned), 1))
        enu = np.column_stack([d.pos_ned[:, 1], d.pos_ned[:, 0], -d.pos_ned[:, 2]])
        label = f"Drone {d.drone_id}" if d.drone_id is not None else name
        drones.append(track_from_samples(label, t, enu, lights=(t_rgb, d.rgb.astype(float)),
                                         meta={"file": name, "rate_hz": rate, "signed": d.signed}))
    drones.sort(key=lambda tr: int(re.sub(r"\D", "", tr.name) or 0))
    return Show(drones, source, "xtjz", {"rate_hz": drones[0].meta["rate_hz"] if drones else None,
                                         "rate_known": bool(own)})
