"""Export a checked show as one file per drone, for PX4 drones.

Each drone gets ``slot_NN.json`` ("idps-traj/1"): its trajectory sampled in the venue's
**NED** frame (D18) with position, velocity and acceleration (feed-forward for PX4's
position controller), a fixed yaw of 0 (North, decision D7), and its LED colour. The drone's
companion computer loads it and plays it through PX4 offboard mode (idps.show.runner).
``manifest.json`` lists the slots, their home spots (where each drone is placed before the
show), the show length and the check result.

Slots, not drones: the show defines *slots* (Drone 1, Drone 2, ...). At the venue each
physical drone is put on a slot's home spot; IDPS knows which drone stands there from its
marker ID, so any drone can fly any slot.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from idps.show.check import ShowReport
from idps.show.model import Show
from idps.show.placement import Placement

FORMAT = "idps-traj/1"
COLUMNS = ["t", "n", "e", "d", "vn", "ve", "vd", "an", "ae", "ad", "r", "g", "b"]


def drone_samples(track, placement: Placement, rate_hz: float, t0: float, t1: float) -> np.ndarray:
    t = np.arange(t0, t1 + 1e-9, 1.0 / rate_hz)
    P = placement.to_ned(track.sample(t))
    V = placement.vec_to_ned(track.sample(t, 1))
    A = placement.vec_to_ned(track.sample(t, 2))
    C = track.colour(t)
    return np.column_stack([t, P, V, A, C])


def export_px4(show: Show, out_dir, placement: Placement | None = None, rate_hz: float = 20.0,
               report: ShowReport | None = None, venue: dict | None = None) -> Path:
    """Write slot_NN.json per drone plus manifest.json. Returns the manifest path."""
    pl = placement or Placement()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    t0, t1 = show.t_start, show.duration
    slots = []
    for k, d in enumerate(show.drones, 1):
        S = drone_samples(d, pl, rate_hz, t0, t1)
        home = S[0, 1:4]
        doc = {
            "format": FORMAT, "slot": k, "name": d.name, "frame": "NED",
            "frame_note": "metres from the venue origin (calibration square corner): N along its long arm, "
                          "E along its short arm, D down. Time in seconds from the show start.",
            "rate_hz": rate_hz, "yaw_deg": 0.0, "home_ned": [round(float(x), 3) for x in home],
            "takeoff_s": float(d.takeoff_time), "end_s": float(t1),
            "columns": COLUMNS,
            "samples": [[round(float(x), 4) for x in row] for row in S],
        }
        text = json.dumps(doc, separators=(",", ":"))
        name = f"slot_{k:02d}.json"
        (out / name).write_text(text)
        slots.append({"slot": k, "name": d.name, "file": name, "home_ned": doc["home_ned"],
                      "sha256": hashlib.sha256(text.encode()).hexdigest()})
    manifest = {
        "format": "idps-show/1", "source": show.source, "duration_s": float(t1 - t0), "rate_hz": rate_hz,
        "placement": {"axes": pl.axes, "rotate_deg": pl.rotate_deg, "offset_ned": list(pl.offset_ned)},
        "venue": venue or {}, "slots": slots,
        "check": None if report is None else {"ok": report.ok, "summary": report.summary(),
                                              "errors": sum(f.severity == "error" for f in report.findings),
                                              "warnings": sum(f.severity == "warning" for f in report.findings)},
    }
    path = out / "manifest.json"
    path.write_text(json.dumps(manifest, indent=1))
    return path
