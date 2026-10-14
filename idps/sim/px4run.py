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
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from idps.core.frames import Frame

REPLAY_HZ = 50.0                # replay steps (the tracking log has 100 per second)
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
    last_s: float = float("nan")  # show time of its newest sample (data that stops early: lost link)


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
        return self.frame.from_ned(np.asarray(ned, float))


def _plan_at(trj, t: np.ndarray) -> np.ndarray:
    k = np.clip((t - trj.t0) * trj.rate_hz, 0, len(trj.pos) - 1)
    i = np.arange(len(trj.pos))
    return np.stack([np.interp(k, i, trj.pos[:, j]) for j in range(3)], 1)


def track_file(folder) -> Path | None:
    """The tracking log of a run folder, the newest if both kinds are there."""
    found = [Path(folder) / f for f in TRACK_FILES if (Path(folder) / f).exists()]
    return max(found, key=lambda p: p.stat().st_mtime) if found else None


_TRACK_CACHE: dict = {}          # path -> (bytes read, rows so far): a growing log is read only from where we stopped


def read_track(path) -> np.ndarray:
    """The tracking log's rows (drone_id, show_t, dn, de, dd), reading only what was added since the
    last call (the flight is running and the log grows); a half-written last line waits for the next."""
    path = Path(path)
    key = str(path.resolve())
    size = path.stat().st_size
    offset, rows = _TRACK_CACHE.get(key, (0, np.empty((0, 5))))
    if size < offset:                                    # a new file under the same name
        offset, rows = 0, np.empty((0, 5))
    if size > offset:
        with open(path, "rb") as f:
            f.seek(offset)
            chunk = f.read(size - offset)
        end = chunk.rfind(b"\n") + 1                     # complete lines only
        text = chunk[:end].decode(errors="replace")
        if offset == 0:
            text = text.split("\n", 1)[1] if "\n" in text else ""   # the header
        new = [ln.split(",") for ln in text.splitlines() if ln.count(",") == 4]
        if new:
            try:
                rows = np.vstack([rows, np.array(new, float)])
            except ValueError:                           # a garbled line: parse one by one
                ok = []
                for ln in new:
                    try:
                        ok.append([float(x) for x in ln])
                    except ValueError:
                        pass
                rows = np.vstack([rows, np.array(ok, float).reshape(-1, 5)])
        offset += end
        _TRACK_CACHE[key] = (offset, rows)
    return rows


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
    T = read_track(tf) if tf is not None else np.empty((0, 5))
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
                                      float(e.max()), float(S[e.argmax(), 1]), float(R[-1, 1])))
        else:
            drones.append(DroneFlight(did, 0, *[float("nan")] * 4))
    last = float(T[:, 1].max()) if len(T) else float("nan")
    mode = "" if tf is None else ("cameras" if tf.name == "track.csv" else "vision")
    return Px4Run(folder, frame, ids, t, plan, actual, err, drones, last, mode)
