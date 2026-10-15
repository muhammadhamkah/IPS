"""Why did drones leave their files? For each moment in a camera-loop run (px4/sitl/run_loop.sh
cameras) when a drone was over the limit, what the simulators and the engine did just before:

* a PX4 simulator that stood still: its clock fell behind the PC's (clock.csv, from the truth link);
* positions that reached the drones late or not at all: their age when sent and gaps between them
  (positions.csv, from idps-engine live: a busy PC, cameras or engine held up);
* neither: the drone's own flight control.

    python px4/sitl/explain_run.py out/loop [--limit-mm 30]
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np

LOOK_BACK_S = 1.0          # what happened up to this long before a drone started to drift
ONSET_MM = 12.0            # an excursion starts where the error last rose past this
CLOCK_STALL_MS = 20.0      # a simulator clock falling this much further behind: it stood still
LATE_MS = 30.0             # a position this much older than the median when sent: held up
GAP_MS = 60.0              # no position for a drone this long


def _excursions(track: np.ndarray, limit_mm: float):
    """[(onset show s, peak show s, peak mm, [drone ids])], drones over the limit within 1 s merged."""
    ev = []
    for did in np.unique(track[:, 0]).astype(int):
        R = track[track[:, 0] == did]
        R = R[np.argsort(R[:, 1])]
        t, e = R[:, 1], 1000 * np.linalg.norm(R[:, 2:5], axis=1)
        over = np.flatnonzero(e > limit_mm)
        if not len(over):
            continue
        groups = np.split(over, np.flatnonzero(np.diff(t[over]) > 1.0) + 1)
        for g in groups:
            k = g[np.argmax(e[g])]
            j = k
            while j > 0 and e[j] > ONSET_MM:
                j -= 1
            ev.append([t[j], t[k], e[k], {did}])
    ev.sort(key=lambda x: x[0])
    merged = []
    for x in ev:
        if merged and x[0] - merged[-1][0] < 1.0:
            m = merged[-1]
            if x[2] > m[2]:
                m[1], m[2] = x[1], x[2]
            m[0] = min(m[0], x[0])
            m[3] |= x[3]
        else:
            merged.append(x)
    return [(a, b, c, sorted(d)) for a, b, c, d in merged]


def _clock_stall_ms(clock: np.ndarray, did: int, show0: float, a: float, b: float) -> float:
    """How much further this drone's simulated clock fell behind the PC's between show times a and
    b (ms): the lower envelope of (PC arrival - PX4 time), so delivery delays do not count."""
    C = clock[clock[:, 0] == did]
    if len(C) < 10:
        return float("nan")
    st = C[:, 1] - show0
    off = C[:, 1] - C[:, 2]
    w = (st >= a - 0.3) & (st <= b)
    if w.sum() < 5:
        return float("nan")
    bins = np.floor(st[w] / 0.1)
    env = np.array([off[w][bins == x].min() for x in np.unique(bins)])
    return float(1000 * (np.max(env) - env[0]))


def explain(folder, limit_mm: float = 30.0) -> list[str]:
    folder = Path(folder)
    track = np.loadtxt(folder / "track.csv", delimiter=",", skiprows=1, ndmin=2)
    m = re.search(r"PC clock \(time.monotonic\) ([\d.]+)", (folder / "start.txt").read_text())
    if not len(track) or m is None:
        return ["explain: no flight in this run"]
    show0 = float(m.group(1))
    loop = json.loads((folder / "loop.json").read_text())
    clock = (np.loadtxt(folder / "clock.csv", delimiter=",", skiprows=1, ndmin=2)
             if (folder / "clock.csv").exists() else np.empty((0, 3)))
    pos = None
    if (folder / "positions.csv").exists():
        with open(folder / "positions.csv") as f:
            head = f.readline().strip().split(",")
        if "age_ms" in head:
            P = np.loadtxt(folder / "positions.csv", delimiter=",", skiprows=1, ndmin=2,
                           usecols=(0, 1, head.index("age_ms")))
            sent_show = loop["t0_monotonic_s"] + (P[:, 0] - loop["epoch_ns"]) * 1e-9 + P[:, 2] / 1000 - show0
            pos = (P[:, 1].astype(int), sent_show, P[:, 2])
    events = _excursions(track, limit_mm)
    if not events:
        return [f"explain: every drone stayed within {limit_mm:g} mm"]
    typical = float(np.median(pos[2])) if pos is not None and len(pos[2]) else float("nan")
    out = [f"== why drones left their files (over {limit_mm:g} mm)"]
    for onset, peak_t, peak, drones in events:
        a, b = onset - LOOK_BACK_S, peak_t
        stalls = {d: _clock_stall_ms(clock, d, show0, a, onset + 0.2) for d in drones}
        stalled = {d: v for d, v in stalls.items() if np.isfinite(v) and v >= CLOCK_STALL_MS}
        late, gaps = {}, {}
        if pos is not None:
            for d in drones:
                w = (pos[0] == d) & (pos[1] >= a) & (pos[1] <= onset + 0.2)
                if w.sum():
                    late[d] = float(pos[2][w].max())
                    s = np.sort(pos[1][w])
                    gaps[d] = float(1000 * np.diff(s).max()) if len(s) > 1 else float("nan")
        held = {d for d in drones if late.get(d, 0) > typical + LATE_MS or gaps.get(d, 0) > GAP_MS}
        who = ", ".join(str(d) for d in drones)
        out.append(f"{onset:6.1f} s: drone(s) {who} off by up to {peak:.0f} mm (at {peak_t:.1f} s)")
        if stalled:
            out.append("   PX4 simulator stood still: " + ", ".join(f"drone {d} {v:.0f} ms" for d, v in stalled.items())
                       + " (a busy or swapping PC)")
        if held:
            out.append("   positions late or missing: " + ", ".join(
                f"drone {d} up to {late.get(d, float('nan')):.0f} ms old, gap {gaps.get(d, float('nan')):.0f} ms"
                for d in sorted(held)) + f" (typical {typical:.0f} ms; cameras or engine held up)")
        if not stalled and not held:
            known = [d for d in drones if np.isfinite(stalls[d])]
            out.append("   no stall seen in the simulators or the positions: the drone's own flight"
                       + ("" if known and pos is not None else " (clock.csv or ages missing: older run?)"))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("folder")
    ap.add_argument("--limit-mm", type=float, default=30.0)
    a = ap.parse_args(argv)
    print("\n".join(explain(a.folder, a.limit_mm)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
