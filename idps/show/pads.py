"""Charging pads, batched take-off to the air grid, and the landing back to the pads.

Each pad holds 4 drones in a 2 x 2 square (``slot_spacing_m`` apart, centre to centre). Drone
ID = 4 x (pad - 1) + slot. The slot is the drone's batch: batch 1 is slot 1 of every pad, and so
on. The show (designed in Blender) starts and ends in the air: its first frame is the initial
grid, its last the final grid, and batch k flies at layer k's height (``layers_m``) in both.

On the launch command every drone runs, on the PC's clock, with batch k starting
``batch_gap_s`` x (k - 1) after the launch time:

  climb straight up from its pad slot to its layer, fly level to its place in the initial grid,
  hold there for the show start. No start ``hold_timeout_s`` after the last batch took off: land.

After the show (or that timeout), batch k leaves the grid ``batch_gap_s`` x (n_batches - k)
later (last up, first down: every drone flies as long): level to above its pad slot, straight
down to ``land_hover_m`` above it, then PX4 lands it.

Each leg is a minimum-jerk move whose length gives its duration: peak speed = 1.875 x mean, so
T = max(MIN_LEG_S, 1.875 x distance / speed). The PX4 module (idps_show) computes the very same
legs from the numbers in the drone's .trj header, which the export writes from this layout.

    pads:                     # pad centres, venue N and E (m)
      - [-3.2, -2.0]
      - [-3.2, -1.0]
    slot_spacing_m: 0.2
    layers_m: [1, 2, 3, 4]    # batch 1..4
    batch_gap_s: 5
    hold_timeout_s: 60
    climb_speed_mps: 0.8
    transfer_speed_mps: 1.0
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

SLOTS = 4
MIN_LEG_S = 1.5
LAND_HOVER_M = 0.3               # the last leg ends this high above the pad; PX4 lands from there
# slot -> (N, E) offset in units of half the slot spacing: 1 front-left, 2 front-right, 3 back-left, 4 back-right
SLOT_UNITS = {1: (1, -1), 2: (1, 1), 3: (-1, -1), 4: (-1, 1)}


@dataclass
class PadLayout:
    pads_ne: np.ndarray                                  # (P, 2) pad centres, venue N, E
    slot_spacing_m: float = 0.2
    layers_m: tuple = (1.0, 2.0, 3.0, 4.0)
    batch_gap_s: float = 5.0
    hold_timeout_s: float = 60.0
    climb_speed_mps: float = 0.8
    transfer_speed_mps: float = 1.0
    source: str = ""

    def pad_slot(self, drone_id: int) -> tuple[int, int]:
        """(pad 1.., slot 1..4) of a drone ID."""
        return (drone_id - 1) // SLOTS + 1, (drone_id - 1) % SLOTS + 1

    def batch(self, drone_id: int) -> int:
        return self.pad_slot(drone_id)[1]

    def layer_m(self, drone_id: int) -> float:
        return float(self.layers_m[self.batch(drone_id) - 1])

    def home_ned(self, drone_id: int) -> np.ndarray:
        """The drone's spot on its pad (D = 0: the floor)."""
        pad, slot = self.pad_slot(drone_id)
        if pad > len(self.pads_ne):
            raise ValueError(f"drone {drone_id} needs pad {pad}, the layout has {len(self.pads_ne)}")
        un, ue = SLOT_UNITS[slot]
        h = self.slot_spacing_m / 2
        return np.array([self.pads_ne[pad - 1][0] + un * h, self.pads_ne[pad - 1][1] + ue * h, 0.0])

    def n_batches(self, drone_ids) -> int:
        return max(self.batch(d) for d in drone_ids)

    def to_dict(self) -> dict:
        return {"pads": [[float(x) for x in p] for p in self.pads_ne], "slot_spacing_m": self.slot_spacing_m,
                "layers_m": [float(x) for x in self.layers_m], "batch_gap_s": self.batch_gap_s,
                "hold_timeout_s": self.hold_timeout_s, "climb_speed_mps": self.climb_speed_mps,
                "transfer_speed_mps": self.transfer_speed_mps}


def load_pads(path) -> PadLayout:
    import yaml
    d = yaml.safe_load(Path(path).read_text())
    return PadLayout(np.array(d["pads"], float).reshape(-1, 2), float(d.get("slot_spacing_m", 0.2)),
                     tuple(float(x) for x in d.get("layers_m", (1, 2, 3, 4))), float(d.get("batch_gap_s", 5.0)),
                     float(d.get("hold_timeout_s", 60.0)), float(d.get("climb_speed_mps", 0.8)),
                     float(d.get("transfer_speed_mps", 1.0)), str(path))


def leg_time(dist: float, speed: float) -> float:
    return max(MIN_LEG_S, 1.875 * abs(dist) / speed)


def min_jerk(u):
    u = np.clip(u, 0.0, 1.0)
    return u ** 3 * (10 - 15 * u + 6 * u * u)


@dataclass
class Legs:
    """A drone's moves: waypoints and leg durations, starting at t0 (s, PC clock / show time)."""
    t0: float
    points: list                                         # NED waypoints, len = legs + 1
    durations: list = field(default_factory=list)

    @property
    def t_end(self) -> float:
        return self.t0 + sum(self.durations)

    def at(self, t: np.ndarray) -> np.ndarray:
        """Positions (len(t), 3) at times t; before t0 the first point, after the end the last."""
        t = np.atleast_1d(np.asarray(t, float))
        out = np.repeat(np.asarray(self.points[0], float)[None], len(t), 0)
        start = self.t0
        for p0, p1, T in zip(self.points, self.points[1:], self.durations):
            p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
            s = min_jerk((t - start) / T)[:, None]
            here = t >= start
            out[here] = (p0 + (p1 - p0) * s)[here]
            start += T
        return out


def launch_legs(lay: PadLayout, drone_id: int, grid_ned, t_launch: float = 0.0) -> Legs:
    """Pad -> up to the layer -> level to the grid place."""
    home = lay.home_ned(drone_id)
    g = np.asarray(grid_ned, float)
    top = np.array([home[0], home[1], g[2]])
    t0 = t_launch + lay.batch_gap_s * (lay.batch(drone_id) - 1)
    return Legs(t0, [home, top, g], [leg_time(top[2] - home[2], lay.climb_speed_mps),
                                     leg_time(np.linalg.norm(g - top), lay.transfer_speed_mps)])


def landing_legs(lay: PadLayout, drone_id: int, from_ned, t_leave0: float, n_batches: int) -> Legs:
    """Grid place -> level to above the pad -> down to LAND_HOVER_M above it (PX4 lands from there).
    t_leave0: when the first batch to land (the last to take off) leaves the grid."""
    home = lay.home_ned(drone_id)
    p = np.asarray(from_ned, float)
    above = np.array([home[0], home[1], p[2]])
    low = np.array([home[0], home[1], home[2] - LAND_HOVER_M])
    t0 = t_leave0 + lay.batch_gap_s * (n_batches - lay.batch(drone_id))
    return Legs(t0, [p, above, low], [leg_time(np.linalg.norm(above - p), lay.transfer_speed_mps),
                                      leg_time(low[2] - above[2], lay.climb_speed_mps)])


def downwash_pairs(P: np.ndarray, active: np.ndarray, radius_m: float, depth_m: float):
    """(i, j, k): drone i hovers above drone j at sample k, within radius_m sideways and depth_m below
    (NED: i's D is smaller). P (N, T, 3), active (N, T)."""
    N = len(P)
    hits = []
    for i in range(N):
        for j in range(N):
            if i == j:
                continue
            dh = np.linalg.norm(P[i, :, :2] - P[j, :, :2], axis=1)
            dz = P[j, :, 2] - P[i, :, 2]                     # > 0: i is above j
            bad = active[i] & active[j] & (dh < radius_m) & (dz > 0.05) & (dz < depth_m)
            for k in np.flatnonzero(bad)[:1]:
                hits.append((i, j, int(k)))
    return hits


TRANSIT_MIN_SEP_M = 0.5          # between flying drones on the way to and from the pads
DOWNWASH_RADIUS_M = 0.4          # no flying drone this close sideways above another ...
DOWNWASH_DEPTH_M = 3.0           # ... that is less than this far below it


def check_sequence(lay: PadLayout, ids, first: np.ndarray, last: np.ndarray, dt: float = 0.05,
                   min_sep_m: float = TRANSIT_MIN_SEP_M, layer_tol_m: float = 0.1):
    """Take-off, hold and landing for drones `ids` whose show starts at `first` (N, 3 NED) and ends at
    `last`. Returns (problems, stats): problems = [(kind, drones (ids), t, ned, message)], all errors."""
    ids = list(ids)
    problems = []
    nb = lay.n_batches(ids)
    for k, did in enumerate(ids):
        pad, slot = lay.pad_slot(did)
        if pad > len(lay.pads_ne):
            problems.append(("pads", (did,), 0.0, (0, 0, 0), f"drone {did} needs pad {pad}: the layout has "
                             f"{len(lay.pads_ne)} pads ({SLOTS} drones each)."))
            continue
        for what, P in (("starts", first[k]), ("ends", last[k])):
            h = -float(P[2])
            if abs(h - lay.layer_m(did)) > layer_tol_m:
                problems.append(("layer", (did,), 0.0, tuple(P), f"drone {did} (batch {slot}) {what} the show at "
                                 f"{h:.2f} m; its batch flies at {lay.layer_m(did):g} m."))
    if problems:
        return problems, {}

    stats = {}
    phases = [("take-off", [launch_legs(lay, d, first[k]) for k, d in enumerate(ids)], "launch")]
    t_land = 0.0
    phases.append(("landing after the show", [landing_legs(lay, d, last[k], t_land, nb) for k, d in enumerate(ids)],
                   "land"))
    phases.append(("landing after a hold timeout", [landing_legs(lay, d, first[k], t_land, nb)
                                                    for k, d in enumerate(ids)], "land"))
    for name, legs, kind in phases:
        t_end = max(L.t_end for L in legs)
        t = np.arange(0.0, t_end + 1.0, dt)
        P = np.stack([L.at(t) for L in legs])                              # (N, T, 3)
        if kind == "launch":
            active = np.stack([t >= L.t0 for L in legs])                   # flying from its take-off on
        else:
            active = np.stack([t <= L.t_end for L in legs])                # flying until it is down
        stats[name] = t_end
        N = len(ids)
        worst = np.inf
        for i in range(N):
            for j in range(i + 1, N):
                both = active[i] & active[j]
                if not both.any():
                    continue
                d = np.linalg.norm(P[i] - P[j], axis=1)
                d = np.where(both, d, np.inf)
                k = int(d.argmin())
                worst = min(worst, float(d[k]))
                if d[k] < min_sep_m:
                    problems.append(("transit", (ids[i], ids[j]), float(t[k]), tuple(np.round(P[i, k], 2)),
                                     f"{name}: drones {ids[i]} and {ids[j]} come within {d[k]:.2f} m "
                                     f"(minimum {min_sep_m:g} m) at {t[k]:.1f} s."))
        stats[name + " closest"] = worst
        for i, j, k in downwash_pairs(P, active, DOWNWASH_RADIUS_M, DOWNWASH_DEPTH_M):
            problems.append(("downwash", (ids[i], ids[j]), float(t[k]), tuple(np.round(P[i, k], 2)),
                             f"{name}: drone {ids[i]} is above drone {ids[j]} ({P[j, k, 2] - P[i, k, 2]:.1f} m, "
                             f"within {DOWNWASH_RADIUS_M:g} m sideways) at {t[k]:.1f} s: its downwash."))
    return problems, stats


def smallest_safe_gap(lay: PadLayout, ids, first: np.ndarray, last: np.ndarray, step_s: float = 0.5,
                      max_s: float = 60.0) -> float | None:
    """The shortest batch gap (s) with which take-off and landing pass (same speeds); None: none up to max_s."""
    import dataclasses
    gap = step_s
    while gap <= max_s + 1e-9:
        if not check_sequence(dataclasses.replace(lay, batch_gap_s=gap), ids, first, last)[0]:
            return gap
        gap += step_s
    return None
