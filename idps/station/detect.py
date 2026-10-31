"""LED dot detector, the reference version of edge/src/detect.hpp (the camera station's).

Same algorithm step for step, written for clarity; the station runs the C++ one, and the tests
check both give the same dots. Pixel (0, 0) is the centre of the top-left pixel, x right, y down.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

SATURATED, EDGE, ELONGATED = 1, 2, 4          # = idps.protocol.packets.MFLAG_*


@dataclass
class DetectParams:
    threshold: int = 60            # seed level (raw DN)
    threshold_low: int = 30        # a dot's pixels (raw DN)
    seed_step: int = 2             # scan every n-th row for seeds
    min_area: int = 1
    max_area: int = 400            # larger regions (a lamp, sunlight) are not markers
    saturation: int = 255          # clipped pixel value (255 for 8-bit, 1023 for 10-bit)
    elongation: float = 1.5        # sqrt(major / minor second moment) above this: elongated
    elongation_min_area: int = 6
    max_dots: int = 1000


@dataclass
class Dot:
    x: float
    y: float
    sum: float
    peak: int
    area: int
    flags: int


def detect(img: np.ndarray, p: DetectParams | None = None) -> list[Dot]:
    p = p or DetectParams()
    img = np.asarray(img)
    H, W = img.shape
    I = img.astype(np.int64)
    label = np.zeros((H, W), np.int64)            # 0 = not part of a dot yet; else the dot's number
    dots: list[Dot] = []
    step = max(1, p.seed_step)
    ys, xs = np.nonzero(I[::step] >= p.threshold)
    next_id = 1
    for sy, sx in zip(ys * step, xs):
        if label[sy, sx]:
            continue
        did = next_id
        next_id += 1
        region = _grow(I, label, sx, sy, p.threshold_low, did)
        d = _measure(I, label, region, did, p)
        if d is not None:
            dots.append(d)
            if len(dots) >= p.max_dots:
                break
    return dots


def _grow(I, label, sx, sy, lo, did):
    """The 8-connected pixels >= lo around the seed, labelled did. Returns [(x, y)]."""
    H, W = I.shape
    stack = [(sx, sy)]
    label[sy, sx] = did
    region = []
    while stack:
        x, y = stack.pop()
        region.append((x, y))
        for dy in (-1, 0, 1):
            yy = y + dy
            if not 0 <= yy < H:
                continue
            for dx in (-1, 0, 1):
                xx = x + dx
                if (dx or dy) and 0 <= xx < W and not label[yy, xx] and I[yy, xx] >= lo:
                    label[yy, xx] = did
                    stack.append((xx, yy))
    return region


def _measure(I, label, region, did, p: DetectParams) -> Dot | None:
    H, W = I.shape
    area = len(region)
    if area < p.min_area or area > p.max_area:
        return None
    rx = np.array([r[0] for r in region])
    ry = np.array([r[1] for r in region])
    x0, x1, y0, y1 = rx.min(), rx.max(), ry.min(), ry.max()
    peak = int(I[ry, rx].max())
    lo = p.threshold_low

    # background: the pixels below lo on the rectangle 2 px outside the bounding box
    ring = [(x, y0 - 2) for x in range(x0 - 2, x1 + 3)] + [(x, y1 + 2) for x in range(x0 - 2, x1 + 3)]
    ring += [(x0 - 2, y) for y in range(y0 - 1, y1 + 2)] + [(x1 + 2, y) for y in range(y0 - 1, y1 + 2)]
    vals = [I[y, x] for x, y in ring if 0 <= x < W and 0 <= y < H and I[y, x] < lo]
    bg = float(np.mean(vals)) if vals else 0.0

    # centroid over the box grown by 1 px: this dot's pixels and the dim pixels around it
    wx0, wx1, wy0, wy1 = max(0, x0 - 1), min(W - 1, x1 + 1), max(0, y0 - 1), min(H - 1, y1 + 1)
    win = I[wy0:wy1 + 1, wx0:wx1 + 1]
    keep = (win < lo) | (label[wy0:wy1 + 1, wx0:wx1 + 1] == did)
    w = np.where(keep, win - bg, 0.0)
    w = np.maximum(w, 0.0)
    s = float(w.sum())
    if s <= 0:
        return None
    gy, gx = np.mgrid[wy0:wy1 + 1, wx0:wx1 + 1]
    cx, cy = float((w * gx).sum() / s), float((w * gy).sum() / s)

    flags = 0
    if area >= p.elongation_min_area:
        wr = I[ry, rx] - bg
        ok = wr > 0
        if ok.any():
            wr, dx, dy = wr[ok], rx[ok] - cx, ry[ok] - cy
            mw = wr.sum()
            mxx, myy, mxy = (wr * dx * dx).sum() / mw, (wr * dy * dy).sum() / mw, (wr * dx * dy).sum() / mw
            tr = mxx + myy
            d = np.sqrt(max(0.0, (mxx - myy) ** 2 / 4 + mxy ** 2))
            l1, l2 = tr / 2 + d, max(tr / 2 - d, 1e-6)
            if np.sqrt(l1 / l2) > p.elongation:
                flags |= ELONGATED
    if peak >= p.saturation:
        flags |= SATURATED
    if x0 == 0 or y0 == 0 or x1 == W - 1 or y1 == H - 1:
        flags |= EDGE
    return Dot(cx, cy, s, peak, area, flags)
