"""Synthetic raw camera frames: LED spots at known positions, as the OV9281 would see them.

For testing the station's dot detector (edge/src/detect.hpp) and for benchmarking it on a Pi
before a marker is in front of the camera. A spot is a Gaussian blur (the slight defocus,
docs/RISKS.md R7) integrated over each pixel, plus the sensor's black level, read noise and
photon (shot) noise, then clipped at the sensor's full scale.

Frame files (`write_frames` / `read_frames`, and the station's `--source file:`):
    b"IDPSFRM1", u32 width, u32 height, u32 bits (8 or 16 per pixel stored), u32 count,
    then count frames of width*height pixels (little-endian u16 when bits = 16).
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

FRM = struct.Struct("<8sIIII")
MAGIC = b"IDPSFRM1"


@dataclass
class Spot:
    x: float                  # true centre, pixels (0, 0 = centre of the top-left pixel)
    y: float
    flux: float               # total signal, DN (sum over the spot, before clipping)
    sigma: float = 1.2        # blur, pixels


@dataclass
class SensorNoise:
    bits: int = 8             # 8 (RAW8) or 10 (RAW10, stored in 16)
    black: float = 4.0        # black level, DN
    read_noise: float = 1.0   # DN rms
    dn_per_electron: float = 0.25   # shot noise: a signal of S DN carries S / dn_per_electron electrons

    @property
    def full_scale(self) -> int:
        return (1 << self.bits) - 1


def spot_flux_for_peak(peak: float, sigma: float) -> float:
    """Total flux whose Gaussian spot peaks at about `peak` DN on a pixel centre."""
    return peak * 2 * math.pi * sigma ** 2


def render(width: int, height: int, spots, noise: SensorNoise | None = None, rng=None,
           background=None) -> np.ndarray:
    """One frame: uint8 for 8-bit, uint16 for 10-bit. `background` (H, W) adds a fixed image."""
    noise = noise or SensorNoise()
    rng = rng if rng is not None else np.random.default_rng()
    img = np.zeros((height, width), np.float64)
    for s in spots:
        r = int(math.ceil(5 * s.sigma)) + 1
        x0, x1 = max(0, int(math.floor(s.x)) - r), min(width - 1, int(math.floor(s.x)) + r + 1)
        y0, y1 = max(0, int(math.floor(s.y)) - r), min(height - 1, int(math.floor(s.y)) + r + 1)
        if x0 > x1 or y0 > y1:
            continue
        img[y0:y1 + 1, x0:x1 + 1] += s.flux * np.outer(_pixel_weights(y0, y1, s.y, s.sigma),
                                                       _pixel_weights(x0, x1, s.x, s.sigma))
    if background is not None:
        img += background
    signal = np.maximum(img, 0.0)
    if noise.dn_per_electron > 0:
        signal = rng.poisson(signal / noise.dn_per_electron) * noise.dn_per_electron
    out = signal + noise.black + rng.normal(0.0, noise.read_noise, img.shape)
    out = np.clip(np.rint(out), 0, noise.full_scale)
    return out.astype(np.uint8 if noise.bits <= 8 else np.uint16)


def render_sparse(width: int, height: int, spots, noise: SensorNoise | None = None, rng=None) -> np.ndarray:
    """As render(), but noise only around the spots (the rest is the flat black level): about
    100x faster, for long runs where the dark pixels do not matter (they stay far below any
    detection threshold)."""
    noise = noise or SensorNoise()
    rng = rng if rng is not None else np.random.default_rng()
    out = np.full((height, width), int(round(noise.black)), np.uint8 if noise.bits <= 8 else np.uint16)
    boxes = []
    sig = {}
    for s in spots:
        r = int(math.ceil(4 * s.sigma)) + 1
        x0, x1 = max(0, int(math.floor(s.x)) - r), min(width - 1, int(math.floor(s.x)) + r + 1)
        y0, y1 = max(0, int(math.floor(s.y)) - r), min(height - 1, int(math.floor(s.y)) + r + 1)
        if x0 > x1 or y0 > y1:
            continue
        patch = s.flux * np.outer(_pixel_weights(y0, y1, s.y, s.sigma), _pixel_weights(x0, x1, s.x, s.sigma))
        boxes.append((x0, x1, y0, y1, patch))
    if not boxes:
        return out
    # overlapping spots add up: boxes that overlap (directly or through others) form one group,
    # rendered together over the group's bounding box
    groups = []
    for b in boxes:
        gx0, gx1, gy0, gy1, members = b[0], b[1], b[2], b[3], [b]
        rest = []
        for g in groups:
            if g[0] <= gx1 and gx0 <= g[1] and g[2] <= gy1 and gy0 <= g[3]:
                gx0, gx1, gy0, gy1 = min(gx0, g[0]), max(gx1, g[1]), min(gy0, g[2]), max(gy1, g[3])
                members += g[4]
            else:
                rest.append(g)
        groups = rest + [(gx0, gx1, gy0, gy1, members)]
    merged = True
    while merged:                       # a grown group can now overlap another: merge until none do
        merged = False
        for i in range(len(groups)):
            for j in range(i + 1, len(groups)):
                a, c = groups[i], groups[j]
                if a[0] <= c[1] and c[0] <= a[1] and a[2] <= c[3] and c[2] <= a[3]:
                    groups[i] = (min(a[0], c[0]), max(a[1], c[1]), min(a[2], c[2]), max(a[3], c[3]), a[4] + c[4])
                    del groups[j]
                    merged = True
                    break
            if merged:
                break
    for gx0, gx1, gy0, gy1, members in groups:
        acc = np.zeros((gy1 - gy0 + 1, gx1 - gx0 + 1))
        for x0, x1, y0, y1, patch in members:
            acc[y0 - gy0:y1 - gy0 + 1, x0 - gx0:x1 - gx0 + 1] += patch
        base = out[gy0:gy1 + 1, gx0:gx1 + 1].astype(np.float64) - noise.black
        sigl = np.maximum(acc + base, 0.0)
        if noise.dn_per_electron > 0:
            sigl = rng.poisson(sigl / noise.dn_per_electron) * noise.dn_per_electron
        v = sigl + noise.black + rng.normal(0.0, noise.read_noise, sigl.shape)
        out[gy0:gy1 + 1, gx0:gx1 + 1] = np.clip(np.rint(v), 0, noise.full_scale)
    return out


def _pixel_weights(i0: int, i1: int, c: float, sigma: float) -> np.ndarray:
    """Fraction of a 1-D Gaussian (centre c) falling on pixels i0..i1 (pixel i spans i-0.5..i+0.5)."""
    edges = np.arange(i0, i1 + 2) - 0.5
    cdf = 0.5 * (1 + _erf((edges - c) / (sigma * math.sqrt(2))))
    return np.diff(cdf)


def _erf(x):
    try:
        from scipy.special import erf
        return erf(x)
    except ImportError:                         # numpy has no erf: a vectorised math.erf
        return np.vectorize(math.erf)(x)


def write_frames(path, frames) -> Path:
    """Frames (each (H, W) uint8 or uint16, all the same) to a frame file for `--source file:`."""
    frames = [np.asarray(f) for f in frames]
    if not frames:
        raise ValueError("no frames")
    H, W = frames[0].shape
    bits = 8 if frames[0].dtype == np.uint8 else 16
    path = Path(path)
    with open(path, "wb") as f:
        f.write(FRM.pack(MAGIC, W, H, bits, len(frames)))
        for fr in frames:
            if fr.shape != (H, W):
                raise ValueError("frames differ in size")
            f.write(fr.astype("<u2" if bits == 16 else np.uint8).tobytes())
    return path


def read_frames(path) -> np.ndarray:
    """(count, H, W) array from a frame file."""
    data = Path(path).read_bytes()
    magic, W, H, bits, n = FRM.unpack_from(data)
    if magic != MAGIC:
        raise ValueError("not an IDPS frame file")
    dt = np.dtype("<u2") if bits == 16 else np.dtype(np.uint8)
    return np.frombuffer(data, dt, count=n * W * H, offset=FRM.size).reshape(n, H, W)
