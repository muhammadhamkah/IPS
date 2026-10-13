"""Score a SITL flight by each drone's distance from its show file at its own show time
(idps_show's "track" output: track.csv from idps-sim px4, truth.track.csv from the stand-in).

    python px4/sitl/score_track.py out/loop/track.csv [--from 1.0]
"""
import argparse

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("track")
    ap.add_argument("--from", dest="t0", type=float, default=1.0, help="show time to score from (s)")
    a = ap.parse_args()
    T = np.loadtxt(a.track, delimiter=",", skiprows=1, ndmin=2)
    if len(T) == 0:
        raise SystemExit("no tracking data: did the show start?")
    worst = 0.0
    for did in np.unique(T[:, 0]).astype(int):
        R = T[(T[:, 0] == did) & (T[:, 1] >= a.t0)]
        e = 1000 * np.linalg.norm(R[:, 2:5], axis=1)
        worst = max(worst, e.max())
        print(f"drone {did}: median {np.median(e):6.1f} mm   p95 {np.percentile(e, 95):6.1f} mm   "
              f"worst {e.max():7.1f} mm at {R[e.argmax(), 1]:5.1f} s")
    print(f"worst over all drones: {worst:.1f} mm")


if __name__ == "__main__":
    main()
