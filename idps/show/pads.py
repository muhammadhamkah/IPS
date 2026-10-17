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
down onto it, then PX4 lands and disarms it.

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
LAND_HOVER_M = 0.0               # the last leg ends on the pad (at zero speed); PX4 then only disarms
TOUCHDOWN_S = 0.5                # ... which takes this long: still flying for the checks
PAD_REACH_M = 0.4                # a drone's downwash reach sideways (the checks' radius)
CLEAR_M = 0.5                    # after the climb, first this far forward (the pad's front) at the layer,
                                 # over the slots already gone, before the long leg to the grid; landing
                                 # comes in there and steps back over the slot
# slot -> (front, right) offset in units of half the slot spacing: 1 front-left, 2 front-right, 3 back-left,
# 4 back-right. A pad's front faces where its drones fly (facing_deg, from North): batches 1-2 leave first and
# 3-4 fly forward over their empty slots, never over a drone still on the pad.
SLOT_UNITS = {1: (1, -1), 2: (1, 1), 3: (-1, -1), 4: (-1, 1)}
PAD_SIDE_M = 0.45                # a pad as drawn: its 4 slots and half a drone around them
MIN_PAD_GAP_M = 0.5              # pads stand 0.5-1 m apart, centre to centre
MARKER_Z_M = 0.08                # a drone's marker standing on its pad
# The venue (site) file's "pads": centres in the drawing frame like the trusses, plus the
# sequence settings. idps-show reads it with --pads venue.yaml, as a pads file.
SITE_DEFAULTS = {"centres": [], "slot_spacing_m": 0.2, "layers_m": [1.0, 2.0, 3.0, 4.0], "batch_gap_s": 5.0,
                 "hold_timeout_s": 60.0, "climb_speed_mps": 1.5, "transfer_speed_mps": 1.5, "facing": "auto"}


def slot_offset_ne(slot: int, spacing_m: float, facing_deg: float = 0.0) -> np.ndarray:
    """A slot's (N, E) offset from its pad's centre."""
    f, r = SLOT_UNITS[slot]
    a = np.radians(facing_deg)
    front, right = np.array([np.cos(a), np.sin(a)]), np.array([-np.sin(a), np.cos(a)])
    return (f * front + r * right) * spacing_m / 2


@dataclass
class PadLayout:
    pads_ne: np.ndarray                                  # (P, 2) pad centres, venue N, E
    slot_spacing_m: float = 0.2
    layers_m: tuple = (1.0, 2.0, 3.0, 4.0)
    batch_gap_s: float = 5.0
    hold_timeout_s: float = 60.0
    climb_speed_mps: float = 0.8
    transfer_speed_mps: float = 1.0
    facing_deg: float = 0.0                              # the pads' front, degrees from North (clockwise)
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
        dn, de = slot_offset_ne(slot, self.slot_spacing_m, self.facing_deg)
        return np.array([self.pads_ne[pad - 1][0] + dn, self.pads_ne[pad - 1][1] + de, 0.0])

    def clear_m(self, drone_id: int) -> float:
        """How far the drone steps forward (the pad's front) after its climb, before the long leg: CLEAR_M,
        or past any other pad standing ahead of its own within reach (a drone flying on over it would
        still be above it when that pad's next batch climbs)."""
        pad = self.pad_slot(drone_id)[0] - 1
        a = np.radians(self.facing_deg)
        fwd, right = np.array([np.cos(a), np.sin(a)]), np.array([-np.sin(a), np.cos(a)])
        reach = PAD_REACH_M + self.slot_spacing_m
        out = CLEAR_M
        for q, c in enumerate(self.pads_ne):
            rel = c - self.pads_ne[pad]
            ahead, side = float(rel @ fwd), abs(float(rel @ right))
            if q != pad and ahead > 0 and side < reach:
                out = max(out, ahead + reach)
        return round(out, 2)

    def n_batches(self, drone_ids) -> int:
        return max(self.batch(d) for d in drone_ids)

    def to_dict(self) -> dict:
        return {"pads": [[float(x) for x in p] for p in self.pads_ne], "slot_spacing_m": self.slot_spacing_m,
                "layers_m": [float(x) for x in self.layers_m], "batch_gap_s": self.batch_gap_s,
                "hold_timeout_s": self.hold_timeout_s, "climb_speed_mps": self.climb_speed_mps,
                "transfer_speed_mps": self.transfer_speed_mps, "facing_deg": self.facing_deg}


def pads_from_site(d: dict, source: str = "") -> PadLayout | None:
    """The charging pads of a venue (site) dict, in its NED frame; None if it has none."""
    from idps.core.frames import Frame
    p = {**SITE_DEFAULTS, **(d.get("pads") or {})}
    if not p["centres"]:
        return None
    f = Frame.from_dict(d.get("origin"))
    ne = np.round(np.array([f.ne(c) for c in p["centres"]], float).reshape(-1, 2), 6) + 0.0   # (no -0.0)
    return PadLayout(ne, float(p["slot_spacing_m"]), tuple(float(x) for x in p["layers_m"]), float(p["batch_gap_s"]),
                     float(p["hold_timeout_s"]), float(p["climb_speed_mps"]), float(p["transfer_speed_mps"]),
                     site_facing_deg(d), source)


FACINGS = {"north": 0.0, "east": 90.0, "south": 180.0, "west": 270.0}


def site_facing_deg(d: dict) -> float:
    """The pads' front, degrees from North: "auto" (the default) faces the middle of the performance
    area from the pads' middle, where their drones fly; or north / east / south / west, or degrees."""
    from idps.core.frames import Frame
    facing = (d.get("pads") or {}).get("facing", "auto")
    if isinstance(facing, (int, float)):
        return float(facing) % 360.0
    if str(facing).lower() in FACINGS:
        return FACINGS[str(facing).lower()]
    f = Frame.from_dict(d.get("origin"))
    vol = d.get("volume") or {}
    if "polygon" in vol and len(vol["polygon"]) >= 3:
        poly = np.array(vol["polygon"], float)
    elif "min" in vol:
        poly = np.array([vol["min"][:2], vol["max"][:2]], float)
    else:
        return 0.0
    pads = f.ne(np.array(d["pads"]["centres"], float)).reshape(-1, 2)
    to_area = f.ne(poly).mean(0) - pads.mean(0)
    if np.hypot(*to_area) < 1e-6:
        to_area = np.array([1.0, 0.0])
    face = to_area
    if len(pads) >= 2:
        # pads in a row: face across the row, towards the area, so they stand side by side and no
        # pad's drones fly over another pad while its next batch climbs
        _, sv, vt = np.linalg.svd(pads - pads.mean(0))
        if sv[1] < PAD_SIDE_M:                                   # (nearly) one row
            normal = np.array([-vt[0][1], vt[0][0]])
            face = normal if normal @ to_area >= 0 else -normal
    return round(float(np.degrees(np.arctan2(face[1], face[0]))), 3) % 360.0 + 0.0


def coverage_points(lay: PadLayout, frame, step_m: float = 0.5) -> np.ndarray:
    """Where the cameras must see the drones on and over the pads, in the drawing frame (x, y,
    z up): each slot's marker on the pad (the start checks every drone is on its own spot) and
    its climb, every step_m up to its batch's layer."""
    out = []
    for pad in range(len(lay.pads_ne)):
        for slot in range(1, SLOTS + 1):
            did = pad * SLOTS + slot
            top = lay.layer_m(did) if slot <= len(lay.layers_m) else 0.0
            zs = np.concatenate([[MARKER_Z_M], np.arange(step_m, top + 1e-9, step_m)])
            ned = np.column_stack([np.repeat(lay.home_ned(did)[None, :2], len(zs), 0), -zs])
            out.append(frame.from_ned(ned))
    return np.concatenate(out) if out else np.zeros((0, 3))


def too_close(centres, gap_m: float = MIN_PAD_GAP_M) -> list[tuple[int, int, float]]:
    """Pairs of pads (indices) closer than gap_m, centre to centre, with their distance."""
    c = np.asarray(centres, float).reshape(-1, 2)
    out = []
    for i in range(len(c)):
        for j in range(i + 1, len(c)):
            dist = float(np.linalg.norm(c[i] - c[j]))
            if dist < gap_m - 1e-9:
                out.append((i, j, dist))
    return out


def load_pads(path) -> PadLayout:
    """A pads file, or a venue (site) file with charging pads placed on its plan."""
    import yaml
    d = yaml.safe_load(Path(path).read_text())
    if isinstance(d.get("pads"), dict):
        lay = pads_from_site(d, str(path))
        if lay is None:
            raise ValueError(f"{path}: no charging pads placed (Planner: Charging pad tool, C)")
        return lay
    return PadLayout(np.array(d["pads"], float).reshape(-1, 2), float(d.get("slot_spacing_m", 0.2)),
                     tuple(float(x) for x in d.get("layers_m", (1, 2, 3, 4))), float(d.get("batch_gap_s", 5.0)),
                     float(d.get("hold_timeout_s", 60.0)), float(d.get("climb_speed_mps", 0.8)),
                     float(d.get("transfer_speed_mps", 1.0)), float(d.get("facing_deg", 0.0)), str(path))




def leg_time(dist: float, speed: float) -> float:
    """A minimum-jerk leg: peak speed 1.875 x mean (and peak acceleration 5.77 x dist / T²)."""
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


def _forward(lay: PadLayout) -> np.ndarray:
    a = np.radians(lay.facing_deg)
    return np.array([np.cos(a), np.sin(a), 0.0])


def batch_leg_times(lay: PadLayout, ids, first, last) -> tuple[dict, dict, dict]:
    """Each drone's long level leg on the way up (clear point -> its grid place) and down (grid ->
    in front of its slot), as long as the longest in its batch: a batch moves as one formation, so
    drones from pads side by side (0.5 m apart) never close in on each other on their way. The
    landing leg covers the final grid and the initial one (after a hold timeout). And how long each
    batch waits in front of its slots before stepping back over them: until the batch before it
    (on the slots beside them) is down. Written into each drone's .trj (the PX4 module flies these)."""
    up, down = {}, {}
    for k, did in enumerate(ids):
        home, f = lay.home_ned(did), _forward(lay) * lay.clear_m(did)
        for grid, out in ((first[k], up), (first[k], down), (last[k], down)):
            g = np.asarray(grid, float)
            p = np.array([home[0], home[1], g[2]]) + f
            out[did] = max(out.get(did, 0.0), leg_time(np.linalg.norm(g - p), lay.transfer_speed_mps))
    batches = sorted({lay.batch(d) for d in ids})
    for times in (up, down):
        for b in batches:
            mine = [d for d in ids if lay.batch(d) == b]
            longest = max(times[d] for d in mine)
            for d in mine:
                times[d] = round(longest, 3)
    wait, done_after = {}, None                       # landing: the last batch up comes down first
    nb = lay.n_batches(ids)
    for b in sorted(batches, reverse=True):
        mine = [d for d in ids if lay.batch(d) == b]
        arrive = lay.batch_gap_s * (nb - b) + down[mine[0]]
        w = max(0.0, done_after - arrive) if done_after is not None else 0.0
        step = max(leg_time(lay.clear_m(d), lay.transfer_speed_mps) for d in mine)
        descend = leg_time(lay.layer_m(mine[0]), lay.climb_speed_mps)
        done_after = arrive + w + step + descend + TOUCHDOWN_S
        for d in mine:
            wait[d] = round(w, 3)
    return up, down, wait


def launch_legs(lay: PadLayout, drone_id: int, grid_ned, t_launch: float = 0.0, long_s: float | None = None) -> Legs:
    """Pad -> up to the layer -> forward clear of the pads -> level to the grid place (in long_s, its
    batch's time, if given). The short step clears the pad (and the slot beside it, whose batch climbs
    next) at once; a long leg starts slowly."""
    home = lay.home_ned(drone_id)
    g = np.asarray(grid_ned, float)
    top = np.array([home[0], home[1], g[2]])
    clear = top + _forward(lay) * lay.clear_m(drone_id)
    t0 = t_launch + lay.batch_gap_s * (lay.batch(drone_id) - 1)
    return Legs(t0, [home, top, clear, g], [leg_time(top[2] - home[2], lay.climb_speed_mps),
                                            leg_time(lay.clear_m(drone_id), lay.transfer_speed_mps),
                                            long_s or leg_time(np.linalg.norm(g - clear), lay.transfer_speed_mps)])


def landing_legs(lay: PadLayout, drone_id: int, from_ned, t_leave0: float, n_batches: int,
                 long_s: float | None = None, wait_s: float = 0.0) -> Legs:
    """Grid place -> level to in front of its slot (in long_s, its batch's time, if given) -> wait
    there wait_s (until the batch beside it is down) -> back over the slot -> down onto it (PX4 lands
    and disarms). t_leave0: when the first batch to land (the last to take off) leaves the grid."""
    home = lay.home_ned(drone_id)
    p = np.asarray(from_ned, float)
    above = np.array([home[0], home[1], p[2]])
    front = above + _forward(lay) * lay.clear_m(drone_id)
    low = np.array([home[0], home[1], home[2] - LAND_HOVER_M])
    t0 = t_leave0 + lay.batch_gap_s * (n_batches - lay.batch(drone_id))
    pts = [p, front] + ([front] if wait_s > 0 else []) + [above, low]
    Ts = [long_s or leg_time(np.linalg.norm(front - p), lay.transfer_speed_mps)] + ([wait_s] if wait_s > 0 else []) \
        + [leg_time(lay.clear_m(drone_id), lay.transfer_speed_mps), leg_time(low[2] - above[2], lay.climb_speed_mps)]
    return Legs(t0, pts, Ts)


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
    up, down, wait = batch_leg_times(lay, ids, first, last)
    phases = [("take-off", [launch_legs(lay, d, first[k], long_s=up[d]) for k, d in enumerate(ids)], "launch")]
    t_land = 0.0
    phases.append(("landing after the show", [landing_legs(lay, d, last[k], t_land, nb, long_s=down[d], wait_s=wait[d])
                                              for k, d in enumerate(ids)], "land"))
    phases.append(("landing after a hold timeout", [landing_legs(lay, d, first[k], t_land, nb, long_s=down[d], wait_s=wait[d])
                                                    for k, d in enumerate(ids)], "land"))
    for name, legs, kind in phases:
        t_end = max(L.t_end for L in legs)
        t = np.arange(0.0, t_end + 1.0, dt)
        P = np.stack([L.at(t) for L in legs])                              # (N, T, 3)
        if kind == "launch":
            active = np.stack([t >= L.t0 for L in legs])                   # flying from its take-off on
        else:
            active = np.stack([t <= L.t_end + TOUCHDOWN_S for L in legs])  # flying until it is down
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
                if d[k] < min_sep_m - 1e-3:                                 # (pads exactly min_sep_m apart)
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
