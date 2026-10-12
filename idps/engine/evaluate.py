"""Compare engine output with the simulator's ground truth (truth.csv).

For each marker: position error (median, p95, max) against the true marker position at the
output's timestamp, the share of the flight time it was output (availability), how long it
took to be identified, and wrong-ID outputs.
"""
from __future__ import annotations

import csv
from dataclasses import dataclass

import numpy as np


def read_truth(path):
    """{marker_id: (t_ns array, marker xyz (N, 3))} from a truth.csv."""
    epoch = None
    rows: dict[int, list] = {}
    with open(path) as f:
        first = f.readline()
        if "epoch_ns" in first:
            epoch = int(first.rsplit("=", 1)[1])
        r = csv.DictReader(f)
        for row in r:
            rows.setdefault(int(row["marker_id"]), []).append(
                (float(row["t_s"]), float(row["marker_x"]), float(row["marker_y"]), float(row["marker_z"])))
    out = {}
    for mid, v in rows.items():
        a = np.array(v)
        out[mid] = ((epoch or 0) + np.round(a[:, 0] * 1e9).astype(np.int64), a[:, 1:4])
    return out


@dataclass
class MarkerScore:
    marker_id: int
    outputs: int
    median_mm: float
    p95_mm: float
    max_mm: float
    availability: float           # share of 1/120 s slots (after identification) with an output
    first_id_s: float             # time from the start of the recording to the first output
    wrong: int                    # outputs further than 0.3 m from the truth (a wrong match or ID)

    def line(self) -> str:
        return (f"marker {self.marker_id:3d}: {self.outputs:6d} outputs, error median {self.median_mm:5.2f} mm, "
                f"p95 {self.p95_mm:5.2f} mm, max {self.max_mm:6.1f} mm, available {self.availability * 100:5.1f} %, "
                f"identified after {self.first_id_s:4.2f} s, wrong {self.wrong}")


def score(outputs, truth, epoch_hz: float = 120.0) -> tuple[list[MarkerScore], int]:
    """outputs: iterable of engine Output. Returns (per-marker scores, outputs with an ID that
    is not in the truth at all)."""
    by: dict[int, list] = {}
    unknown = 0
    for o in outputs:
        if o.marker_id not in truth:
            unknown += 1
            continue
        by.setdefault(o.marker_id, []).append((o.t_ns, *o.pos))
    t_start = min(tt[0][0] for tt in truth.values())
    res = []
    for mid, (tt, X) in sorted(truth.items()):
        rows = np.array(by.get(mid, []), float).reshape(-1, 4)
        if not len(rows):
            res.append(MarkerScore(mid, 0, np.nan, np.nan, np.nan, 0.0, np.nan, 0))
            continue
        t = rows[:, 0]
        ref = np.column_stack([np.interp(t, tt, X[:, k]) for k in range(3)])
        err = np.linalg.norm(rows[:, 1:] - ref, axis=1)
        good = err < 0.3
        e = err[good] * 1000 if good.any() else np.array([np.nan])
        first = t.min()
        slots = max(1.0, (tt[-1] - first) * 1e-9 * epoch_hz)
        res.append(MarkerScore(mid, len(rows), float(np.median(e)), float(np.percentile(e, 95)), float(e.max()),
                               min(1.0, len(rows) / slots), (first - t_start) * 1e-9, int((~good).sum())))
    return res, unknown
