"""Score a closed-loop run (idps-sim px4 + idps-engine live + idps-show start).

    python px4/sitl/score_loop.py out/loop out/show/manifest.json --positions out/loop/positions.csv \
        --start 12345.678

* Engine: each position the engine sent against the marker the cameras filmed (truth.csv),
  on the ground and in flight.
* Flight: each drone's true centre against its show file, from show time 0 (--start: the "show
  time 0" that idps-show start prints) to the end of the show, and while the show has it above
  0.5 m (leaving out take-off and landing).
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from idps.core.frames import Frame  # noqa: E402
from idps.show.trj import read_trj  # noqa: E402


def stats(e):
    e = np.asarray(e) * 1000
    return f"median {np.median(e):5.1f}  p95 {np.percentile(e, 95):5.1f}  max {e.max():6.1f} mm  ({len(e)})" if len(e) else "-"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("loop", help="idps-sim px4 output folder (truth.csv, loop.json)")
    ap.add_argument("manifest")
    ap.add_argument("--positions", help="idps-engine live -o CSV (default LOOP/positions.csv)")
    ap.add_argument("--start", type=float, help="show time 0 on the PC clock (s), as idps-show start prints it")
    a = ap.parse_args()
    loop = Path(a.loop)
    man = json.loads(Path(a.manifest).read_text())
    frame = Frame.from_dict(man["venue"]["origin"])
    info = json.loads((loop / "loop.json").read_text())
    rows = [r for r in csv.reader(open(loop / "truth.csv")) if r and not r[0].startswith("#")][1:]
    truth = {}
    for r in rows:
        truth.setdefault(int(r[1]), []).append([float(x) for x in (r[0], *r[3:9])])
    truth = {k: np.array(v) for k, v in truth.items()}       # t_s, centre xyz, marker xyz (world)

    pos = np.loadtxt(a.positions or loop / "positions.csv", delimiter=",", skiprows=1, usecols=(0, 1, 2, 3, 4))
    print("Engine positions vs the filmed marker (NED):")
    for did, T in sorted(truth.items()):
        P = pos[pos[:, 1] == did]
        if not len(P):
            print(f"  drone {did}: no positions")
            continue
        t = (P[:, 0] - info["epoch_ns"]) * 1e-9
        ok = (t >= T[0, 0]) & (t <= T[-1, 0])
        t, P = t[ok], P[ok]
        m = np.stack([np.interp(t, T[:, 0], T[:, 4 + k]) for k in range(3)], 1)
        err = np.linalg.norm(P[:, 2:5] - frame.ned(m), axis=1)
        fly = -frame.ned(m)[:, 2] > 0.3
        print(f"  drone {did}: on the ground {stats(err[~fly])}\n           in flight     {stats(err[fly])}")

    if a.start is None:
        return
    print("Flight: true centre vs the show file (from show time 0 to the end):")
    for d in sorted(man["drones"], key=lambda d: d["drone_id"]):
        did = d["drone_id"]
        if did not in truth:
            continue
        trj = read_trj(Path(a.manifest).parent / d["trj"])
        T = truth[did]
        tau = info["t0_monotonic_s"] + T[:, 0] - a.start
        ok = (tau >= 0) & (tau <= trj.duration)
        k = (tau[ok] - trj.t0) * trj.rate_hz
        plan = np.stack([np.interp(k, np.arange(len(trj.pos)), trj.pos[:, j]) for j in range(3)], 1)
        true = frame.ned(T[ok, 1:4])
        e = true - plan
        up = -plan[:, 2] > 0.5                          # leaving out take-off and landing
        print(f"  drone {did}: whole show   3D {stats(np.linalg.norm(e, axis=1))}\n"
              f"           above 0.5 m  3D {stats(np.linalg.norm(e[up], axis=1))}, "
              f"horizontal {stats(np.linalg.norm(e[up, :2], axis=1))}")


if __name__ == "__main__":
    main()
