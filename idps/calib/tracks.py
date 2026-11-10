"""From a recording of one LED moved through the room to matched observations.

Each station reports the dots it sees, frame by frame, each at its own exposure time. This module
keeps the frames where a station sees exactly one dot (after removing its fixed reflections),
brings all stations to common instants by interpolating each one's dot between its own frames
(the cameras do not expose together, and the LED moves between their exposures), and finds the
moments the LED rested still (the reference points)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from idps.engine.ingest import Ingest


@dataclass
class DotTrack:
    station_id: int
    t: np.ndarray               # (N,) seconds from the recording's start, increasing
    xy: np.ndarray              # (N, 2) distorted pixels
    frames: int = 0             # frames received
    fixed_spots: int = 0        # fixed reflections removed


@dataclass
class Observations:
    stations: list              # station ids, the order of axis 1 below
    t: np.ndarray               # (P,) seconds
    xy: np.ndarray              # (P, C, 2) pixels, NaN where a station did not see the LED
    rest: list                  # [(first index, last index)] in P of each rest point, in time order
    period_s: float = 1 / 120

    @property
    def seen(self) -> np.ndarray:
        return ~np.isnan(self.xy[..., 0])


def frames_from_packets(packets) -> dict[int, list]:
    """packets: iterable of (time ns, bytes) (idps.sim.runner.read_raw, or a MemorySink's list).
    Returns each station's frames, in time order."""
    ing = Ingest()
    out: dict[int, list] = {}
    for _, pkt in packets:
        for f in ing.feed(pkt):
            out.setdefault(f.station_id, []).append(f)
    for fr in out.values():
        fr.sort(key=lambda f: f.t_ns)
    return out


def single_dot_tracks(frames: dict[int, list], spot_share: float = 0.5) -> tuple[dict[int, DotTrack], int]:
    """Each station's frames with exactly one dot that is not a fixed reflection (a spot, to 2 px,
    lit in more than ``spot_share`` of that station's frames). Returns the tracks and the time
    origin (ns)."""
    t0 = min(fr[0].t_ns for fr in frames.values() if fr)
    tracks = {}
    for sid, fr in sorted(frames.items()):
        counts: dict[tuple, int] = {}
        for f in fr:
            for key in {(int(x) // 2, int(y) // 2) for x, y in f.xy}:
                counts[key] = counts.get(key, 0) + 1
        fixed = {k for k, n in counts.items() if n > spot_share * len(fr)}

        def is_fixed(x, y):
            kx, ky = int(x) // 2, int(y) // 2          # (a spot on a bin edge falls in either bin)
            return any((kx + a, ky + b) in fixed for a in (-1, 0, 1) for b in (-1, 0, 1)
                       if abs(x - (2 * (kx + a) + 1)) < 2.5 and abs(y - (2 * (ky + b) + 1)) < 2.5)

        t, xy = [], []
        for f in fr:
            keep = [i for i in range(f.n) if not is_fixed(f.xy[i, 0], f.xy[i, 1])] if fixed else list(range(f.n))
            if len(keep) == 1:
                t.append((f.t_ns - t0) * 1e-9)
                xy.append(f.xy[keep[0]])
        tracks[sid] = DotTrack(sid, np.array(t), np.array(xy).reshape(-1, 2), len(fr), len(fixed))
    return tracks, t0


def _interp(tr: DotTrack, tq: np.ndarray, max_gap_s: float, max_step_px: float) -> np.ndarray:
    out = np.full((len(tq), 2), np.nan)
    if len(tr.t) < 2:
        return out
    j = np.searchsorted(tr.t, tq)                     # tr.t[j-1] <= tq < tr.t[j]
    ok = (j > 0) & (j < len(tr.t))
    j0, j1 = np.clip(j - 1, 0, len(tr.t) - 1), np.clip(j, 0, len(tr.t) - 1)
    gap = tr.t[j1] - tr.t[j0]
    step = np.linalg.norm(tr.xy[j1] - tr.xy[j0], axis=1)
    ok &= (gap > 0) & (gap <= max_gap_s) & (step <= max_step_px)
    w = np.where(ok, (tq - tr.t[j0]) / np.where(gap > 0, gap, 1.0), 0.0)
    val = tr.xy[j0] + w[:, None] * (tr.xy[j1] - tr.xy[j0])
    out[ok] = val[ok]
    return out


def observations(tracks: dict[int, DotTrack], rate_hz: float = 30.0, period_s: float | None = None,
                 max_step_px: float = 12.0, rest_s: float = 1.5, rest_px: float = 0.6,
                 max_points: int = 2500) -> Observations:
    """Common instants at ``rate_hz`` where at least two stations see the LED. A station's dot is
    interpolated between two consecutive frames of its own (not across a missed frame, nor across
    a jump of more than ``max_step_px``: a reflection or a second light). Rest points: the LED
    within ``rest_px`` of one spot in every station for ``rest_s``; each becomes one observation
    (the mean), and the rest are thinned to ``max_points``."""
    sids = sorted(tracks)
    if period_s is None:
        d = np.concatenate([np.diff(tr.t) for tr in tracks.values() if len(tr.t) > 1] or [np.array([1 / 120])])
        period_s = float(np.median(d))
    t_end = max((tr.t[-1] for tr in tracks.values() if len(tr.t)), default=0.0)
    tq = np.arange(0.0, t_end, 1.0 / rate_hz)
    xy = np.stack([_interp(tracks[s], tq, 1.6 * period_s, max_step_px) for s in sids], axis=1)   # (P, C, 2)
    seen = ~np.isnan(xy[..., 0])
    keep = seen.sum(1) >= 2
    tq, xy, seen = tq[keep], xy[keep], seen[keep]

    # rest points: runs where every station that sees the LED keeps it within rest_px of the run's start
    def still(i, j):
        # (the blinking LED drops below some far cameras' threshold on its dim half: compare the
        # cameras that see it at both instants, two at least)
        both = seen[i] & seen[j]
        return both.sum() >= 2 and np.abs(xy[j, both] - xy[i, both]).max() < rest_px

    rest, i, n = [], 0, len(tq)
    while i < n:
        j = i + 1
        while j < n and tq[j] - tq[j - 1] < 3.0 / rate_hz and still(i, j):
            j += 1
        if tq[j - 1] - tq[i] >= rest_s:
            rest.append((i, j - 1))
        i = j if tq[j - 1] - tq[i] >= rest_s else i + 1

    # one observation per rest point (its mean), then the moving ones, thinned evenly
    in_rest = np.zeros(n, bool)
    for a, b in rest:
        in_rest[a:b + 1] = True
    moving = np.nonzero(~in_rest)[0]
    if len(moving) > max_points:
        moving = moving[np.linspace(0, len(moving) - 1, max_points).astype(int)]
    def mean_xy(a, b):
        seg, m = xy[a:b + 1], seen[a:b + 1]
        cnt = m.sum(0)
        out = np.where(m[..., None], seg, 0.0).sum(0) / np.maximum(cnt, 1)[:, None]
        out[cnt < max(3, 0.2 * len(seg))] = np.nan        # a camera that caught it only now and then
        return out

    rows = [(tq[a:b + 1].mean(), mean_xy(a, b), True) for a, b in rest]
    rows += [(tq[k], xy[k], False) for k in moving]
    rows.sort(key=lambda r: r[0])
    t_out = np.array([r[0] for r in rows])
    xy_out = np.array([r[1] for r in rows]).reshape(-1, len(sids), 2)
    rest_idx = [(k, k) for k, r in enumerate(rows) if r[2]]
    return Observations(sids, t_out, xy_out, rest_idx, period_s)
