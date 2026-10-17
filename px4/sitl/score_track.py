"""Score a SITL flight by each drone's distance from where it should be (idps_show's "track"
output: track.csv from idps-sim px4, truth.track.csv from the stand-in): from its show file at its
own show time, and, from charging pads, from its take-off path, its spot in the grid while it holds
and its landing path.

    python px4/sitl/score_track.py out/loop/track.csv [--from 1.0]
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from idps.sim.px4run import PHASES, phase_times  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("track")
    ap.add_argument("--from", dest="t0", type=float, default=None,
                    help="show time to score the show from (s; default 1.0 from the ground, all of it from pads)")
    a = ap.parse_args()
    T = np.genfromtxt(a.track, delimiter=",", skip_header=1, ndmin=2)
    if T.size == 0:
        raise SystemExit("no tracking data: did the show start?")
    phase = T[:, 5].astype(int) if T.shape[1] >= 10 else np.zeros(len(T), int)
    pads = bool((phase > 0).any())
    t0 = a.t0 if a.t0 is not None else (-np.inf if pads else 1.0)
    t = phase_times(T)                                   # show time of every row, before and after it too
    e_all = 1000 * np.linalg.norm(T[:, 2:5], axis=1)
    show = phase == 0
    if not show.any():
        print("the show did not start")
    worst = 0.0
    for did in np.unique(T[:, 0]).astype(int):
        mine = T[:, 0] == did
        sel = mine & show & (T[:, 1] >= t0)
        line = f"drone {did}: "
        if sel.any():
            e = e_all[sel]
            worst = max(worst, e.max())
            line += (f"median {np.median(e):6.1f} mm   p95 {np.percentile(e, 95):6.1f} mm   "
                     f"worst {e.max():7.1f} mm at {T[sel, 1][e.argmax()]:5.1f} s")
        if pads:
            parts = []
            for p in (1, 2, 3):
                s = mine & (phase == p)
                if s.any():
                    k = np.flatnonzero(s)[e_all[s].argmax()]
                    parts.append(f"{PHASES[p]} {e_all[k]:5.1f} mm at {t[k]:6.1f} s")
            line += ("   |   " if sel.any() else "") + ", ".join(parts)
        print(line)
    if show.any():
        print(f"worst over all drones: {worst:.1f} mm")
    if pads:
        for p in (1, 2, 3):
            s = phase == p
            if s.any():
                k = np.flatnonzero(s)[e_all[s].argmax()]
                print(f"{PHASES[p]}: worst {e_all[k]:.1f} mm (drone {int(T[k, 0])} at {t[k]:.1f} s show time)")


if __name__ == "__main__":
    main()
