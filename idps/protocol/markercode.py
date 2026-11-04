"""Optical marker ID code, Mode A (docs/PROTOCOL.md section 6.2).

Frame = SYNC (6 chips: H H H L L L) + Manchester(ID 8 bits + CRC-4), 30 chips.
Manchester: bit 1 -> (H, L), bit 0 -> (L, H). Data never contains three equal
chips in a row, so the H H H L L L sync is unambiguous. CRC-4 uses the
primitive polynomial x^4 + x + 1; the 12-bit codewords form a shortened
Hamming code with minimum distance 3, so every one of the 256 IDs is usable.
"""
from __future__ import annotations

import numpy as np

SYNC = (1, 1, 1, 0, 0, 0)
ID_BITS = 8
CRC_BITS = 4
CHIPS_PER_FRAME = len(SYNC) + 2 * (ID_BITS + CRC_BITS)     # 30
DEFAULT_CHIP_RATE_HZ = 30.0                                 # F/4 at 120 fps


def crc4(value: int, nbits: int = ID_BITS) -> int:
    """CRC-4 (x^4 + x + 1), MSB first, zero initial value."""
    reg = 0
    for i in reversed(range(nbits)):
        bit = (value >> i) & 1
        top = (reg >> 3) & 1
        reg = (reg << 1) & 0xF
        if bit ^ top:
            reg ^= 0x3
    return reg


def codeword_bits(marker_id: int) -> list[int]:
    if not 0 <= marker_id < 2 ** ID_BITS:
        raise ValueError(f"marker id must be 0..{2 ** ID_BITS - 1}")
    word = (marker_id << CRC_BITS) | crc4(marker_id)
    return [(word >> i) & 1 for i in reversed(range(ID_BITS + CRC_BITS))]


def chips(marker_id: int) -> np.ndarray:
    """One frame of chips (1 = HIGH, 0 = LOW), length CHIPS_PER_FRAME."""
    out = list(SYNC)
    for b in codeword_bits(marker_id):
        out += [1, 0] if b else [0, 1]
    return np.array(out, np.int8)


def decode_chips(seq) -> int | None:
    """Decode one aligned chip frame (starting at SYNC). None if invalid."""
    seq = list(seq)
    if len(seq) != CHIPS_PER_FRAME or tuple(seq[:len(SYNC)]) != SYNC:
        return None
    bits = []
    for a, b in zip(seq[len(SYNC)::2], seq[len(SYNC) + 1::2]):
        if (a, b) == (1, 0):
            bits.append(1)
        elif (a, b) == (0, 1):
            bits.append(0)
        else:
            return None
    word = int("".join(map(str, bits)), 2)
    mid, crc = word >> CRC_BITS, word & 0xF
    return mid if crc4(mid) == crc else None


class MarkerLed:
    """Brightness of one free-running coded LED over time.

    The LED's own clock runs at chip_rate * (1 + ppm * 1e-6) and starts at a
    random phase; HIGH/LOW are relative drive levels."""

    def __init__(self, marker_id: int, chip_rate_hz: float = DEFAULT_CHIP_RATE_HZ, clock_ppm: float = 0.0,
                 phase_s: float = 0.0, high: float = 1.0, low: float = 0.4):
        self.marker_id = marker_id
        self.pattern = chips(marker_id).astype(float)
        self.rate = chip_rate_hz * (1.0 + clock_ppm * 1e-6)
        self.phase = phase_s
        self.high, self.low = high, low

    def level(self, t) -> np.ndarray:
        """Instantaneous drive level at true time(s) t."""
        idx = np.floor((np.asarray(t) + self.phase) * self.rate).astype(np.int64) % CHIPS_PER_FRAME
        return np.where(self.pattern[idx] > 0, self.high, self.low)

    def mean_level(self, t_start, t_end) -> np.ndarray:
        """Average level over an exposure window (exact for a piecewise-constant signal)."""
        t_start, t_end = np.asarray(t_start, float), np.asarray(t_end, float)
        a = (t_start + self.phase) * self.rate
        b = (t_end + self.phase) * self.rate
        ia, ib = np.floor(a).astype(np.int64), np.floor(b).astype(np.int64)
        la = np.where(self.pattern[ia % CHIPS_PER_FRAME] > 0, self.high, self.low)
        lb = np.where(self.pattern[ib % CHIPS_PER_FRAME] > 0, self.high, self.low)
        same = ia == ib
        # exposures are far shorter than a chip, so at most one transition occurs
        frac_a = np.where(same, 1.0, (ib - a) / np.maximum(b - a, 1e-12))
        return np.where(same, la, la * frac_a + lb * (1 - frac_a))
