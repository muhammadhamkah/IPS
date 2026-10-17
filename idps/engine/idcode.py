"""Reading a marker's ID from its blink code (docs/PROTOCOL.md section 6.2).

Each marker LED repeats a 1 s code at 30 chips/s: a sync pattern then its 8-bit ID and a
4-bit check, Manchester-coded, switching between HIGH and LOW brightness. The cameras see
it as the dot's brightness going up and down. The decoder:

1. takes the track's brightness samples from every camera, with each camera's slow trend
   (range, viewing angle) removed, so HIGH sits above 0 and LOW below;
2. tries chip-clock phases across one chip and keeps the one that splits the samples into
   the cleanest HIGH / LOW chips (samples near a chip edge are left out);
3. looks for the sync pattern in the chip string, decodes every complete code it finds and
   keeps those whose check bits are right.
"""
from __future__ import annotations

import numpy as np

from idps.protocol import markercode as mc


def decode(t, level, chip_rate_hz: float = mc.DEFAULT_CHIP_RATE_HZ, phases: int = 24, edge: float = 0.15):
    """(marker ID, number of valid codes, contrast) or (None, 0, contrast)."""
    t = np.asarray(t, float)
    level = np.asarray(level, float)
    if len(t) < 30:
        return None, 0, 0.0
    T = 1.0 / chip_rate_hz
    best = None
    for k in range(phases):
        ph = k * T / phases
        x = (t - t[0] + T - ph) / T                      # +T keeps chip indices non-negative
        idx = np.floor(x).astype(int)
        frac = x - idx
        keep = (frac > edge) & (frac < 1 - edge)
        if keep.sum() < 20:
            continue
        n = idx[keep].max() + 1
        s = np.bincount(idx[keep], level[keep], minlength=n)
        c = np.bincount(idx[keep], minlength=n)
        have = c > 0
        mean = np.where(have, s / np.maximum(c, 1), 0.0)
        contrast = float(np.mean(np.abs(mean[have])))
        if best is None or contrast > best[0]:
            best = (contrast, mean, have)
    if best is None:
        return None, 0, 0.0
    contrast, mean, have = best
    chips = (mean > 0).astype(int)
    votes: dict[int, int] = {}
    L = mc.CHIPS_PER_FRAME
    for start in range(0, len(chips) - L + 1):
        if not have[start:start + L].all():
            continue
        mid = mc.decode_chips(chips[start:start + L])
        if mid is not None:
            votes[mid] = votes.get(mid, 0) + 1
    if not votes:
        return None, 0, contrast
    mid = max(votes, key=votes.get)
    return mid, votes[mid], contrast
