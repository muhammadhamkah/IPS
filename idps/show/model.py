"""Show model: one trajectory (and light programme) per drone.

Trajectories follow Skybrush's representation: a list of keypoints
``[t, [x, y, z], [control points...]]``; each pair of consecutive keypoints is a Bézier
segment whose interior control points are listed with the later keypoint (none = a straight
line, one = quadratic, two = cubic, ...). Times are seconds, positions metres, in the
*show frame* (the axes the show was designed in, e.g. Blender's X, Y and Z up).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import comb

import numpy as np


@dataclass
class Segment:
    t0: float
    t1: float
    points: np.ndarray            # (n + 1, 3) Bézier control polygon, start ... end

    def _diff(self, order):
        P, n = self.points, len(self.points) - 1
        for o in range(order):
            if len(P) < 2:
                return np.zeros((1, 3))
            P = (n - o) * (P[1:] - P[:-1])
        return P

    def eval(self, t, order=0):
        """Position (order 0), velocity (1) or acceleration (2) at times t (seconds)."""
        T = max(self.t1 - self.t0, 1e-9)
        u = (np.asarray(t, float) - self.t0) / T
        n = len(self.points) - 1
        if order > n:
            return np.zeros((len(np.atleast_1d(t)), 3))
        D = self._diff(order)
        nd = len(D) - 1
        uu = np.clip(np.atleast_1d(u), 0.0, 1.0)[:, None]
        k = np.arange(nd + 1)[None, :]
        B = np.array([comb(nd, i) for i in range(nd + 1)])[None, :] * uu ** k * (1 - uu) ** (nd - k)
        return (B @ D) / T ** order


@dataclass
class DroneTrack:
    name: str
    segments: list[Segment]
    home: np.ndarray | None = None
    takeoff_time: float = 0.0
    landing_time: float | None = None
    lights: object = None          # (times (K,), rgb (K, 3) 0..255) when known, else raw data
    meta: dict = field(default_factory=dict)

    @property
    def t_start(self) -> float:
        return self.segments[0].t0 if self.segments else 0.0

    @property
    def t_end(self) -> float:
        return self.segments[-1].t1 if self.segments else 0.0

    def _seg_index(self, t):
        starts = np.array([s.t0 for s in self.segments])
        return np.clip(np.searchsorted(starts, t, side="right") - 1, 0, len(self.segments) - 1)

    def sample(self, t, order=0) -> np.ndarray:
        """Position / velocity / acceleration at times t; holds the first / last position outside."""
        t = np.atleast_1d(np.asarray(t, float))
        out = np.zeros((len(t), 3))
        if not self.segments:
            return out
        idx = self._seg_index(t)
        for k in np.unique(idx):
            sel = idx == k
            out[sel] = self.segments[k].eval(t[sel], order)
        if order:
            out[(t < self.t_start) | (t > self.t_end)] = 0.0
        return out

    def colour(self, t) -> np.ndarray:
        """LED colour (0..255) at times t, when the light programme is known as samples."""
        t = np.atleast_1d(np.asarray(t, float))
        if not (isinstance(self.lights, tuple) and len(self.lights) == 2):
            return np.full((len(t), 3), 255.0)
        lt, rgb = self.lights
        k = np.clip(np.searchsorted(lt, t, side="right") - 1, 0, len(lt) - 1)
        return rgb[k]


@dataclass
class Show:
    drones: list[DroneTrack]
    source: str = ""
    fmt: str = ""
    meta: dict = field(default_factory=dict)

    @property
    def duration(self) -> float:
        return max((d.t_end for d in self.drones), default=0.0)

    @property
    def t_start(self) -> float:
        return min((d.t_start for d in self.drones), default=0.0)


def track_from_points(name: str, points, **kw) -> DroneTrack:
    """Build a track from Skybrush-style keypoints [[t, [x,y,z], [cps...]], ...]."""
    segs = []
    prev_t, prev_p = None, None
    for t, p, cps in points:
        t, p = float(t), np.asarray(p, float)[:3]
        if prev_t is not None:
            if t <= prev_t:
                raise ValueError(f"{name}: keypoint times must increase (t = {t})")
            ctrl = [np.asarray(c, float)[:3] for c in (cps or [])]
            segs.append(Segment(prev_t, t, np.array([prev_p, *ctrl, p])))
        elif cps:
            raise ValueError(f"{name}: the first keypoint cannot have control points")
        prev_t, prev_p = t, p
    if not segs and prev_t is not None:                     # a single point: hold it
        segs.append(Segment(prev_t, prev_t + 1e-3, np.array([prev_p, prev_p])))
    return DroneTrack(name, segs, **kw)


def track_from_samples(name: str, t, pos, smooth: bool = True, **kw) -> DroneTrack:
    """Track through sampled positions (a CSV export, a per-drone show file).

    smooth: a Catmull-Rom spline through the samples (cubic Bezier segments, velocity continuous),
    so velocity and acceleration are available, e.g. as the drone's feed-forward. Otherwise
    straight lines between samples, whose acceleration is zero inside every segment."""
    t = np.asarray(t, float)
    pos = np.asarray(pos, float)
    if smooth and len(t) > 2:
        m = np.empty_like(pos)
        m[1:-1] = (pos[2:] - pos[:-2]) / (t[2:] - t[:-2])[:, None]
        m[0] = (pos[1] - pos[0]) / (t[1] - t[0])
        m[-1] = (pos[-1] - pos[-2]) / (t[-1] - t[-2])
        segs = []
        for k in range(len(t) - 1):
            h = (t[k + 1] - t[k]) / 3
            segs.append(Segment(t[k], t[k + 1], np.array([pos[k], pos[k] + m[k] * h, pos[k + 1] - m[k + 1] * h,
                                                          pos[k + 1]])))
        return DroneTrack(name, segs, **kw)
    segs = [Segment(t[k], t[k + 1], np.array([pos[k], pos[k + 1]])) for k in range(len(t) - 1)]
    if not segs and len(t):
        segs.append(Segment(t[0], t[0] + 1e-3, np.array([pos[0], pos[0]])))
    return DroneTrack(name, segs, **kw)
