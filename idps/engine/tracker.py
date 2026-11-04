"""Marker tracks: a constant-velocity Kalman filter per marker, and the track's life cycle.

tentative (just born) -> confirmed (seen in enough epochs) -> coasting (missed lately)
-> deleted (missed too long). Only confirmed tracks whose blink code has been read (so the
engine knows which drone it is) are output.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

TENTATIVE, CONFIRMED = "tentative", "confirmed"


class KalmanCV:
    """State [x y z vx vy vz]; white-noise acceleration with spectral density q (m²/s³)."""

    def __init__(self, X, t: float, cov_pos, q: float = 30.0, vel_sigma: float = 3.0):
        self.x = np.concatenate([np.asarray(X, float), np.zeros(3)])
        self.P = np.zeros((6, 6))
        self.P[:3, :3] = cov_pos
        self.P[3:, 3:] = np.eye(3) * vel_sigma ** 2
        self.t = t
        self.q = q

    def predicted(self, t: float):
        dt = t - self.t
        return self.x[:3] + self.x[3:] * dt, self.x[3:]

    def predict(self, t: float):
        dt = t - self.t
        if dt <= 0:
            return
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        q = self.q
        Q = np.zeros((6, 6))
        Q[:3, :3] = np.eye(3) * q * dt ** 3 / 3
        Q[:3, 3:] = Q[3:, :3] = np.eye(3) * q * dt ** 2 / 2
        Q[3:, 3:] = np.eye(3) * q * dt
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.t = t

    def update(self, z, R) -> float:
        """Position measurement; returns the squared Mahalanobis distance of the innovation."""
        H = np.zeros((3, 6))
        H[:, :3] = np.eye(3)
        y = np.asarray(z, float) - self.x[:3]
        S = self.P[:3, :3] + R
        Si = np.linalg.inv(S)
        K = self.P[:, :3] @ Si
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
        return float(y @ Si @ y)

    @property
    def pos(self):
        return self.x[:3]

    @property
    def vel(self):
        return self.x[3:]


@dataclass
class Track:
    tid: int
    kf: KalmanCV
    born: float
    state: str = TENTATIVE
    hits: int = 1
    epochs: int = 1
    last_update: float = 0.0
    n_cams: int = 0
    rejects: int = 0                 # consecutive measurements rejected as outliers
    updated_epoch: int = -1
    marker_id: int | None = None
    id_votes: dict = field(default_factory=dict)
    next_decode: float = 0.0
    # brightness samples for the blink-code decoder: (time s, station, log brightness)
    samples: deque = field(default_factory=lambda: deque(maxlen=6000))

    def add_brightness(self, t: float, station: int, value: float):
        self.samples.append((t, station, float(np.log(max(value, 1.0)))))

    def levels(self, since: float):
        """(times, levels) with each station's slow brightness trend removed (centred on 0):
        the trend follows range and viewing angle, the remaining step is the blink. The trend
        is a centred 1 s moving mean: one whole code (exactly half HIGH, half LOW), so the
        blink averages out of it exactly."""
        rows = [r for r in self.samples if r[0] >= since - 0.5]
        if not rows:
            return np.zeros(0), np.zeros(0)
        a = np.array(rows)
        t, st, lv = a[:, 0], a[:, 1], a[:, 2]
        out = np.full_like(lv, np.nan)
        for s in np.unique(st):
            m = np.nonzero(st == s)[0]
            tt, vv = t[m], lv[m]
            c = np.concatenate([[0.0], np.cumsum(vv)])
            j0 = np.searchsorted(tt, tt - 0.5, "left")
            j1 = np.searchsorted(tt, tt + 0.5, "right")
            full = (tt - 0.5 >= tt[0]) & (tt + 0.5 <= tt[-1])          # only where the window is complete
            mean = (c[j1] - c[j0]) / np.maximum(j1 - j0, 1)
            out[m] = np.where(full, vv - mean, np.nan)
        keep = np.isfinite(out) & (t >= since)
        order = np.argsort(t[keep], kind="stable")
        return t[keep][order], out[keep][order]
