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
TRACK_HEADER = "drone_id,show_t,dn,de,dd,phase,pc_s,pn,pe,pd\n"
PHASES = {0: "show", 1: "take-off", 2: "hold", 3: "landing"}     # idps_show's TrackPhase
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
    takeoff_mm: float = float("nan")    # worst off its take-off path (from charging pads)
    hold_mm: float = float("nan")       # ... its spot in the grid while it waited for the start
    landing_mm: float = float("nan")    # ... its landing path


@dataclass
class Px4Run:
    folder: Path
    frame: Frame
    drone_ids: list
    t: np.ndarray               # (T,) show time, s (negative: before the show, taking off and holding)
    plan: np.ndarray            # (D, T, 3) where each drone should be, NED m
    actual: np.ndarray          # (D, T, 3) where the drone was, NED m (NaN: no sample)
    error_mm: np.ndarray        # (D, T) distance from where it should be (NaN: no sample)
    drones: list                # DroneFlight, per drone
    last_t: float               # show time of the newest sample (NaN: none yet)
    mode: str                   # "cameras" | "vision" | "" (no tracking log yet)
    phase: np.ndarray = None    # (D, T) PHASES key at each step (-1: no sample)
    show_span: tuple = (0.0, 0.0)   # the show file's start and end, show time

    @property
    def worst_mm(self) -> float:
        v = [d.worst_mm for d in self.drones if np.isfinite(d.worst_mm)]
        return max(v) if v else float("nan")

    def world(self, ned: np.ndarray) -> np.ndarray:
        """NED points (..., 3) -> the dashboard's world frame (drawing x, y, z up)."""
        return self.frame.from_ned(np.asarray(ned, float))


def track_line(drone_id: int, d) -> str:
    """One tracking-log row from idps_show's DEBUG_FLOAT_ARRAY "track" data: the distance from where
    the drone should be (dn, de, dd), the show time (NaN before the start is known), the phase,
    the PC's clock and where it should be (venue NED)."""
    return (f"{drone_id},{d[3]:.4f},{d[0]:.5f},{d[1]:.5f},{d[2]:.5f},{int(d[4])},"
            f"{d[5] + d[6]:.4f},{d[7]:.4f},{d[8]:.4f},{d[9]:.4f}\n")


def _plan_at(trj, t: np.ndarray) -> np.ndarray:
    k = np.clip((t - trj.t0) * trj.rate_hz, 0, len(trj.pos) - 1)
    i = np.arange(len(trj.pos))
    return np.stack([np.interp(k, i, trj.pos[:, j]) for j in range(3)], 1)


def track_file(folder) -> Path | None:
    """The tracking log of a run folder, the newest if both kinds are there."""
    found = [Path(folder) / f for f in TRACK_FILES if (Path(folder) / f).exists()]
    return max(found, key=lambda p: p.stat().st_mtime) if found else None


_TRACK_CACHE: dict = {}          # path -> (bytes read, rows so far): a growing log is read only from where we stopped


NCOL = 10


def _rows(lines) -> np.ndarray:
    """Tracking-log lines -> (N, 10): an older log's rows (drone_id, show_t, dn, de, dd) are all show,
    with no PC time or plan."""
    out = []
    for ln in lines:
        try:
            v = [float(x) for x in ln]
        except ValueError:
            continue
        if len(v) == 5:
            v += [0.0, np.nan, np.nan, np.nan, np.nan]
        if len(v) == NCOL:
            out.append(v)
    return np.array(out, float).reshape(-1, NCOL)


def read_track(path) -> np.ndarray:
    """The tracking log's rows (drone_id, show_t, dn, de, dd, phase, pc_s, pn, pe, pd), reading only
    what was added since the last call (the flight is running and the log grows); a half-written
    last line waits for the next."""
    path = Path(path)
    key = str(path.resolve())
    size = path.stat().st_size
    offset, rows = _TRACK_CACHE.get(key, (0, np.empty((0, NCOL))))
    if size < offset:                                    # a new file under the same name
        offset, rows = 0, np.empty((0, NCOL))
    if size > offset:
        with open(path, "rb") as f:
            f.seek(offset)
            chunk = f.read(size - offset)
        end = chunk.rfind(b"\n") + 1                     # complete lines only
        text = chunk[:end].decode(errors="replace")
        if offset == 0:
            text = text.split("\n", 1)[1] if "\n" in text else ""   # the header
        new = _rows(ln.split(",") for ln in text.splitlines())
        if len(new):
            rows = np.vstack([rows, new])
        offset += end
        _TRACK_CACHE[key] = (offset, rows)
    return rows


def phase_times(T: np.ndarray) -> np.ndarray:
    """Show time of every tracking-log row: its own while the show runs, else from the PC's clock
    (the drones' clocks are synced to it) and the show's start on that clock (from the show rows).
    No show rows (the hold timed out): seconds from the first row."""
    t = T[:, 1].copy()
    if T.shape[1] < NCOL or not len(T):
        return t
    pc = T[:, 6]
    show = (T[:, 5] == 0) & np.isfinite(T[:, 1]) & (pc > 0)
    if show.any():
        zero = float(np.median(pc[show] - T[show, 1]))
    else:
        ok = pc > 0
        zero = float(pc[ok].min()) if ok.any() else 0.0
    return np.where(np.isfinite(t) & (T[:, 5] == 0), t, pc - zero)


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
    homes = [np.asarray(d.get("home_ned", tr.home), float) for d, tr in zip(rows, trjs)]
    ids = [d["drone_id"] for d in rows]
    show_start = min(tr.t0 for tr in trjs)
    show_end = max(tr.t0 + tr.duration for tr in trjs)

    tf = track_file(folder)
    T = read_track(tf) if tf is not None else np.empty((0, NCOL))
    T = T[np.isfinite(phase_times(T))] if len(T) else T
    tt = phase_times(T) if len(T) else np.empty(0)
    pads = bool(len(T) and (T[:, 5] > 0).any())
    t_lo = min(show_start, float(tt.min())) if len(tt) else show_start
    t_hi = max(show_end, float(tt.max())) if len(tt) else show_end
    t = np.arange(t_lo, t_hi + 1e-9, 1.0 / REPLAY_HZ)

    D = len(ids)
    plan = np.stack([_plan_at(tr, t) for tr in trjs])
    actual = np.full((D, len(t), 3), np.nan)
    err = np.full((D, len(t)), np.nan)
    phase = np.full((D, len(t)), -1, int)
    drones = []
    for i, (did, tr) in enumerate(zip(ids, trjs)):
        mine = T[:, 0] == did if len(T) else np.zeros(0, bool)
        R, rt = T[mine], tt[mine]
        order = np.argsort(rt, kind="stable")
        R, rt = R[order], rt[order]
        if pads:                                         # on its pad before its take-off and after landing
            off = (t < show_start) | (t > show_end)
            plan[i, off] = homes[i]
        if len(R) >= 2:
            want = np.where(np.isfinite(R[:, 7:10]), R[:, 7:10], _plan_at(tr, rt))   # older logs: no plan
            pos = want + R[:, 2:5]
            j = np.clip(np.searchsorted(rt, t), 1, len(rt) - 1)
            near = np.minimum(np.abs(rt[j] - t), np.abs(rt[j - 1] - t)) < GAP_S
            near &= (t >= rt[0]) & (t <= rt[-1])
            for a in range(3):
                actual[i, :, a] = np.where(near, np.interp(t, rt, pos[:, a]), np.nan)
                plan[i, :, a] = np.where(near, np.interp(t, rt, want[:, a]), plan[i, :, a])
            e = 1000 * np.linalg.norm(R[:, 2:5], axis=1)
            err[i] = np.where(near, np.interp(t, rt, e), np.nan)
            nearest = np.where(np.abs(rt[j] - t) < np.abs(rt[j - 1] - t), j, j - 1)
            phase[i] = np.where(near, R[nearest, 5].astype(int), -1)
        e_all = 1000 * np.linalg.norm(R[:, 2:5], axis=1) if len(R) else np.empty(0)
        S = (R[:, 5] == 0) & (R[:, 1] >= tr.t0 + (0.0 if pads else SCORE_FROM_S)) if len(R) else np.zeros(0, bool)
        worst_of = [float(e_all[R[:, 5] == p].max()) if len(R) and (R[:, 5] == p).any() else float("nan")
                    for p in (1, 2, 3)]
        last = float(rt[-1]) if len(rt) else float("nan")
        if S.any():
            e = e_all[S]
            drones.append(DroneFlight(did, int(S.sum()), float(np.median(e)), float(np.percentile(e, 95)),
                                      float(e.max()), float(R[S, 1][e.argmax()]), last, *worst_of))
        else:
            drones.append(DroneFlight(did, 0, *[float("nan")] * 4, last, *worst_of))
    last = float(tt.max()) if len(tt) else float("nan")
    mode = "" if tf is None else ("cameras" if tf.name == "track.csv" else "vision")
    return Px4Run(folder, frame, ids, t, plan, actual, err, drones, last, mode, phase, (show_start, show_end))
