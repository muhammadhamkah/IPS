"""Export a checked show as one file per drone, for PX4 drones.

Each drone gets ``drone_NNN.json`` ("idps-traj/1"), named by its **drone ID**: its trajectory sampled in the venue's
**NED** frame (D18) with position, velocity and acceleration (feed-forward for PX4's
position controller), a fixed yaw of 0 (North, decision D7), and its LED colour. The PC plays
every file to its drone through PX4 offboard mode over Wi-Fi (idps.show.fleet, D20).
``manifest.json`` lists the drones, their home spots (where each drone is placed before the
show), the show length and the check result.

Drone IDs (D21): every physical drone has a fixed ID, and the same number is its PX4
``MAV_SYS_ID`` and its IR marker ID. The show's "Drone 7" is flown by the drone with ID 7,
which must stand on Drone 7's home spot; IDPS checks that before take-off from the marker IDs
(idps.show.fleet.preflight).
"""
from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

import numpy as np

from idps.show.check import ShowReport
from idps.show.model import Show
from idps.show.placement import Placement
from idps.show.trj import Trj, crc32, write_trj

FORMAT = "idps-traj/1"
COLUMNS = ["t", "n", "e", "d", "vn", "ve", "vd", "an", "ae", "ad", "r", "g", "b"]


MAX_DRONE_ID = 255                       # PX4 MAV_SYS_ID 1..255; marker IDs 0..255


def drone_ids(show: Show) -> list[int]:
    """Drone ID for each show drone: the number in its name ("Drone 7" -> 7), or 1..N in
    order when the names carry no numbers. Duplicates and IDs outside 1..255 are errors."""
    nums = [re.findall(r"\d+", d.name) for d in show.drones]
    if all(nums):
        ids = [int(n[-1]) for n in nums]
    else:
        ids = list(range(1, len(show.drones) + 1))
    dup = sorted({i for i in ids if ids.count(i) > 1})
    if dup:
        raise ValueError(f"two show drones have the same drone ID: {dup}; rename them in Blender")
    bad = [i for i in ids if not 1 <= i <= MAX_DRONE_ID]
    if bad:
        raise ValueError(f"drone IDs must be 1..{MAX_DRONE_ID} (PX4 MAV_SYS_ID): {bad}")
    return ids


def drone_samples(track, placement: Placement, rate_hz: float, t0: float, t1: float) -> np.ndarray:
    t = np.arange(t0, t1 + 1e-9, 1.0 / rate_hz)
    P = placement.to_ned(track.sample(t))
    V = placement.vec_to_ned(track.sample(t, 1))
    A = placement.vec_to_ned(track.sample(t, 2))
    C = track.colour(t)
    return np.column_stack([t, P, V, A, C])


def export_px4(show: Show, out_dir, placement: Placement | None = None, rate_hz: float = 20.0,
               report: ShowReport | None = None, venue: dict | None = None, pads=None) -> Path:
    """Write drone_NNN.json and drone_NNN.trj per drone (by drone ID) plus manifest.json.
    Returns the manifest path."""
    pl = placement or Placement()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    t0, t1 = show.t_start, show.duration
    drones = []
    ids = drone_ids(show)
    nb = pads.n_batches(ids) if pads is not None else 0
    up = down = wait = {}
    if pads is not None:                    # each batch's long legs: as long as its longest (a formation)
        from idps.show.pads import batch_leg_times
        ends = [drone_samples(d, pl, rate_hz, t0, t1) for d in show.drones]
        up, down, wait = batch_leg_times(pads, ids, [S[0, 1:4] for S in ends], [S[-1, 1:4] for S in ends])
    for k, d in zip(ids, show.drones):
        S = drone_samples(d, pl, rate_hz, t0, t1)
        # with charging pads the drone's home is its pad slot (the show starts in the air)
        home = pads.home_ned(k) if pads is not None else S[0, 1:4]
        seq = dict(batch=pads.batch(k), n_batches=nb, batch_gap_s=pads.batch_gap_s,
                   climb_speed_mps=pads.climb_speed_mps, transfer_speed_mps=pads.transfer_speed_mps,
                   hold_timeout_s=pads.hold_timeout_s, pad_facing_deg=pads.facing_deg,
                   pad_clear_m=pads.clear_m(k), up_leg_s=up[k], down_leg_s=down[k],
                   land_wait_s=wait[k]) if pads is not None else {}
        doc = {
            "format": FORMAT, "drone_id": k, "name": d.name, "frame": "NED",
            "frame_note": "metres from the venue origin (calibration square corner): N along its long arm, "
                          "E along its short arm, D down. Time in seconds from the show start.",
            "rate_hz": rate_hz, "yaw_deg": 0.0, "home_ned": [round(float(x), 3) for x in home],
            "takeoff_s": float(d.takeoff_time), "end_s": float(t1),
            "columns": COLUMNS,
            "samples": [[round(float(x), 4) for x in row] for row in S],
        }
        text = json.dumps(doc, separators=(",", ":"))
        name = f"drone_{k:03d}.json"
        (out / name).write_text(text)
        trj_name = f"drone_{k:03d}.trj"                   # what goes on the drone's SD card (D22)
        b = write_trj(out / trj_name, Trj(k, rate_hz, float(S[0, 0]), home, 0.0, S[:, 1:4], S[:, 4:7],
                                         S[:, 7:10], S[:, 10:13], **seq))
        extra = {"pad": pads.pad_slot(k)[0], "slot": pads.pad_slot(k)[1], "batch": pads.batch(k)} if pads else {}
        drones.append({"drone_id": k, "name": d.name, "file": name, "home_ned": doc["home_ned"], **extra,
                       "sha256": hashlib.sha256(text.encode()).hexdigest(),
                       "trj": trj_name, "trj_bytes": len(b), "trj_crc32": crc32(b)})
    manifest = {
        "format": "idps-show/1", "source": show.source, "duration_s": float(t1 - t0), "rate_hz": rate_hz,
        "placement": {"axes": pl.axes, "rotate_deg": pl.rotate_deg, "offset_ned": list(pl.offset_ned)},
        "venue": venue or {}, "drones": drones,
        "pads": pads.to_dict() if pads is not None else None,
        "check": None if report is None else {"ok": report.ok, "summary": report.summary(),
                                              "errors": sum(f.severity == "error" for f in report.findings),
                                              "warnings": sum(f.severity == "warning" for f in report.findings)},
    }
    path = out / "manifest.json"
    path.write_text(json.dumps(manifest, indent=1))
    return path
