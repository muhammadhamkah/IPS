"""A PX4 SITL show flight (px4/sitl/run_loop.sh) read back for the dashboard's PX4 flight tab.

A run folder holds the exported show (show/manifest.json and one .trj per drone) and the
idps_show module's own tracking log: how far each drone was from its file at its own show time
(track.csv in the camera loop, truth.track.csv with positions inside PX4). That log is all the
replay needs: where the drone was = its file at that time + the logged distance.

    run = load_run("out/loop")
    run.drones[0].worst_mm, run.plan[0, k], run.actual[0, k], run.error_mm[0, k]

The file may still be growing (the flight is running): load it again for the latest state.
"""
from __future__ import annotations

import json
import warnings
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from idps.core.frames import Frame

REPLAY_HZ = 20.0
TRACK_FILES = ("track.csv", "truth.track.csv")
GAP_S = 0.25                    # no tracking sample this close: no position shown
SCORE_FROM_S = 1.0              # as px4/sitl/score_track.py: leave out the first second (on the pad)


@dataclass
class DroneFlight:
    drone_id: int
    samples: int
    median_mm: float
    p95_mm: float
    worst_mm: float
    worst_at_s: float           # show time of the worst moment


@dataclass
class Px4Run:
    folder: Path
    frame: Frame
    drone_ids: list
    t: np.ndarray               # (T,) show time, s
    plan: np.ndarray            # (D, T, 3) where the file has each drone, NED m
    actual: np.ndarray          # (D, T, 3) where the drone was, NED m (NaN: no sample)
    error_mm: np.ndarray        # (D, T) distance from the file (NaN: no sample)
    drones: list                # DroneFlight, per drone
    last_t: float               # show time of the newest sample (NaN: none yet)
    mode: str                   # "cameras" | "vision" | "" (no tracking log yet)

    @property
    def worst_mm(self) -> float:
        v = [d.worst_mm for d in self.drones if np.isfinite(d.worst_mm)]
        return max(v) if v else float("nan")

    def world(self, ned: np.ndarray) -> np.ndarray:
        """NED points (..., 3) -> the dashboard's world frame (drawing x, y, z up)."""
        ned = np.asarray(ned, float)
        return np.stack([self.frame.from_ned(p) for p in ned.reshape(-1, 3)]).reshape(ned.shape)


def _plan_at(trj, t: np.ndarray) -> np.ndarray:
    k = np.clip((t - trj.t0) * trj.rate_hz, 0, len(trj.pos) - 1)
    i = np.arange(len(trj.pos))
    return np.stack([np.interp(k, i, trj.pos[:, j]) for j in range(3)], 1)


def track_file(folder) -> Path | None:
    """The tracking log of a run folder, the newest if both kinds are there."""
    found = [Path(folder) / f for f in TRACK_FILES if (Path(folder) / f).exists()]
    return max(found, key=lambda p: p.stat().st_mtime) if found else None


def load_run(folder) -> Px4Run:
    from idps.show.trj import read_trj
    folder = Path(folder)
    man_path = folder / "show" / "manifest.json"
    if not man_path.exists():
        raise FileNotFoundError(f"{folder} is not a PX4 flight folder (no show/manifest.json)")
    man = json.loads(man_path.read_text())
    frame = Frame.from_dict(man["venue"]["origin"])
    rows = sorted(man["drones"], key=lambda d: d["drone_id"])
    trjs = [read_trj(man_path.parent / d["trj"]) for d in rows]
    ids = [d["drone_id"] for d in rows]
    t_start = min(tr.t0 for tr in trjs)
    t_end = max(tr.t0 + tr.duration for tr in trjs)
    t = np.arange(t_start, t_end + 1e-9, 1.0 / REPLAY_HZ)
    plan = np.stack([_plan_at(tr, t) for tr in trjs])

    D = len(ids)
    actual = np.full((D, len(t), 3), np.nan)
    err = np.full((D, len(t)), np.nan)
    tf = track_file(folder)
    T = np.empty((0, 5))
    if tf is not None:
        try:
            with warnings.catch_warnings():               # empty until the show starts
                warnings.simplefilter("ignore", UserWarning)
                T = np.loadtxt(tf, delimiter=",", skiprows=1, ndmin=2).reshape(-1, 5)
        except ValueError:                       # a half-written last line while the flight runs
            lines = tf.read_text().splitlines()[1:-1]
            T = np.array([[float(x) for x in ln.split(",")] for ln in lines if ln.count(",") == 4]).reshape(-1, 5)
    drones = []
    for i, (did, tr) in enumerate(zip(ids, trjs)):
        R = T[T[:, 0] == did] if len(T) else T
        R = R[np.argsort(R[:, 1])] if len(R) else R
        if len(R) >= 2:
            pos = _plan_at(tr, R[:, 1]) + R[:, 2:5]
            j = np.clip(np.searchsorted(R[:, 1], t), 1, len(R) - 1)
            near = np.minimum(np.abs(R[j, 1] - t), np.abs(R[j - 1, 1] - t)) < GAP_S
            near &= (t >= R[0, 1]) & (t <= R[-1, 1])
            for a in range(3):
                actual[i, :, a] = np.where(near, np.interp(t, R[:, 1], pos[:, a]), np.nan)
            e = 1000 * np.linalg.norm(R[:, 2:5], axis=1)
            err[i] = np.where(near, np.interp(t, R[:, 1], e), np.nan)
        S = R[R[:, 1] >= tr.t0 + SCORE_FROM_S] if len(R) else R
        if len(S):
            e = 1000 * np.linalg.norm(S[:, 2:5], axis=1)
            drones.append(DroneFlight(did, len(S), float(np.median(e)), float(np.percentile(e, 95)),
                                      float(e.max()), float(S[e.argmax(), 1])))
        else:
            drones.append(DroneFlight(did, 0, *[float("nan")] * 4))
    last = float(T[:, 1].max()) if len(T) else float("nan")
    mode = "" if tf is None else ("cameras" if tf.name == "track.csv" else "vision")
    return Px4Run(folder, frame, ids, t, plan, actual, err, drones, last, mode)
