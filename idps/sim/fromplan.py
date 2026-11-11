"""Simulate a planned layout end to end: the planner's cameras film drones, the positioning
engine works out where they are, and the result is scored against the truth.

This is the full pipeline (pixels, wire packets, engine), not the per-drone visibility model
the live monitor uses. The dashboard's Simulate tab runs it on the layout being planned.

    res = simulate_plan(site, cameras, SimSettings(drones="formation", n_drones=12, duration_s=30))
"""
from __future__ import annotations

import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from idps.plan.model import PlannedCamera
from idps.plan.site import Site
from idps.sim import trajectory as traj
from idps.sim.scenario import CameraModel, MarkerModel, NetworkModel, Scenario, SimDrone, TimingModel

REPLAY_HZ = 30.0
SHOW_LEAD_S = 5.0          # drones stand on their spots this long before the show starts (to be identified)


class Cancelled(Exception):
    pass


@dataclass
class SimSettings:
    drones: str = "formation"          # "formation" | "random" | "show"
    n_drones: int = 12
    duration_s: float = 30.0           # ignored for a show (its own length + SHOW_LEAD_S)
    show_path: str | None = None       # .skyc or Skybrush CSV zip, for drones="show"
    seed: int = 1
    speed_mps: float = 2.5
    calib_rot_mrad: float = 0.05       # how well the cameras are calibrated (1σ); 0.05 mrad is the
    calib_pos_mm: float = 1.0          # target for a hall (ARCHITECTURE P12)
    clock_profile: str | None = "pi5_ptp_hw"
    network_loss: float = 0.001
    reorder_ms: float = 8.0


@dataclass
class DroneResult:
    drone_id: int
    median_mm: float
    p95_mm: float
    max_mm: float
    available: float                   # share of the time after it was first identified
    identified_s: float                # seconds from the start until its first position
    wrong: int                         # positions further than 0.3 m from the truth


@dataclass
class SimResult:
    settings: SimSettings
    cameras: list                      # PlannedCamera, as filmed
    stations: list                     # idps.sim.camera.SimStation (true poses), same order
    drone_ids: list
    t: np.ndarray                      # (F,) replay times, s
    centre: np.ndarray                 # (D, F, 3) true drone centres, world frame
    marker: np.ndarray                 # (D, F, 3) true marker positions
    led: np.ndarray                    # (D, F) LED drive level (the blink code)
    engine: np.ndarray                 # (D, F, 3) engine position (NaN: none)
    error_mm: np.ndarray               # (D, F) engine vs true marker (NaN: none)
    n_cams: np.ndarray                 # (D, F) cameras the engine used (0: none)
    drones: list                       # DroneResult
    unknown_ids: int                   # positions with an ID that is not one of the drones
    frames: int = 0
    packets: int = 0
    sim_s: float = 0.0                 # wall time to film
    engine_s: float = 0.0              # wall time for the engine
    show_start_s: float | None = None  # replay time when the show starts (drones="show")
    notes: list = field(default_factory=list)
    takeoff_s: float | None = None     # from charging pads: replay time when every drone is up and the flight starts

    @property
    def worst_p95_mm(self) -> float:
        v = [d.p95_mm for d in self.drones if np.isfinite(d.p95_mm)]
        return max(v) if v else float("nan")

    @property
    def identified(self) -> int:
        return sum(np.isfinite(d.identified_s) for d in self.drones)


class _ShowTrajectory(traj.Trajectory):
    """A show drone's path in the simulator's world frame; show time 0 at `start_s`."""

    def __init__(self, track, placement, frame, t0: float, start_s: float):
        self.track, self.pl, self.frame, self.t0, self.start = track, placement, frame, t0, start_s

    def position(self, t):
        t = np.atleast_1d(np.asarray(t, float))
        ned = self.pl.to_ned(self.track.sample(t - self.start + self.t0))
        return self.frame.from_ned(ned)


PAD_WAIT_S = 5.0            # on the pads this long before the first batch takes off (to be identified)
DRONE_BODY_RADIUS_M = 0.06  # the show drone (60 g, 65 mm frame + props), as in the camera loop's scenario: the
                            # default 0.2 m is a 1 kg quad, and 4 of those on a pad would hide each other


class _FromPad(traj.Trajectory):
    """A drone that starts on its charging pad slot: it stands there, takes off with its batch
    (straight up, forward clear of the pad, then level to where its flight begins:
    idps.show.pads.launch_legs), holds until every drone is up (t_join), then flies `inner`
    from its start."""

    def __init__(self, inner: traj.Trajectory, legs, frame, t_join: float):
        self.inner, self.legs, self.frame, self.t_join = inner, legs, frame, float(t_join)

    def position(self, t):
        t = np.asarray(t, float)
        flat = np.atleast_1d(t).ravel()
        out = np.asarray(self.inner.position(np.maximum(flat - self.t_join, 0.0)), float).reshape(-1, 3).copy()
        up = flat < self.t_join
        if up.any():
            out[up] = self.frame.from_ned(self.legs.at(flat[up]))
        return out.reshape(t.shape + (3,))


def from_pads(site: Site, drones: list, starts_at_0: bool = True):
    """Put simulated drones on the venue's charging pads: drone k on pad (k - 1) // 4 + 1, slot
    (k - 1) % 4 + 1, taking off in the 4 batches (rows of pads one behind the other: staggered)
    to where its flight begins. drones: [(id, trajectory)], ids 1..n. Returns ([(id, trajectory)],
    time when every drone is up and the flight starts, notes)."""
    from idps.show.pads import SLOTS, launch_legs, sequence_plan
    lay = site.pads
    room = SLOTS * len(lay.pads_ne)
    notes = []
    if len(drones) > room:
        notes.append(f"{len(drones)} drones need {-(-len(drones) // SLOTS)} charging pads; the venue has "
                     f"{len(lay.pads_ne)}: {room} drones fly (4 per pad).")
        drones = drones[:room]
    ids = [did for did, _ in drones]
    first = np.array([site.frame.ned(np.asarray(tr.position(np.array(0.0)), float).reshape(1, 3))[0]
                      for _, tr in drones])
    plan = sequence_plan(lay, ids, first, first)
    legs = [launch_legs(lay, did, first[k], t_launch=PAD_WAIT_S, delay_s=plan.delay[did], stagger_s=plan.max_delay)
            for k, did in enumerate(ids)]
    t_join = max(L.t_end for L in legs) + 1.0
    return [(did, _FromPad(tr, L, site.frame, t_join)) for (did, tr), L in zip(drones, legs)], t_join, notes


def build_scenario(site: Site, cameras: list[PlannedCamera], s: SimSettings) -> tuple[Scenario, float | None]:
    """A Scenario for the planned cameras, with formation, random or show drones. With charging
    pads in the venue every drone starts on its pad slot and takes off from it (from_pads).
    Returns (scenario, replay time when the show starts or None); the scenario's extras carry
    "takeoff_s" (when every drone is up) and "notes"."""
    from idps.monitor.simsource import SimShow
    rng = np.random.default_rng(s.seed)
    show_start = None
    if s.drones == "show":
        if not s.show_path:
            raise ValueError("choose a show file")
        from idps.show.export import drone_ids
        from idps.show.placement import Placement
        from idps.show.readers import load_show
        show = load_show(s.show_path)
        ids = drone_ids(show)
        pl = Placement()
        t0 = show.t_start
        show_start = SHOW_LEAD_S
        duration = SHOW_LEAD_S + (show.duration - t0) + 2.0
        routes = [(did, _ShowTrajectory(tr, pl, site.frame, t0, show_start)) for did, tr in zip(ids, show.drones)]
    else:
        sim = SimShow(site, cameras, n_drones=s.n_drones, seed=s.seed, speed_mps=s.speed_mps,
                      formation=s.drones == "formation")
        routes = list(sim.drones)
        duration = s.duration_s
    extras = {}
    if getattr(site, "pads", None) is not None and len(site.pads.pads_ne) \
            and [did for did, _ in routes] == list(range(1, len(routes) + 1)):
        # every flight starts on the charging pads: take-off in the 4 batches, then the flight
        if s.drones == "show":                         # (the show's own lead: it begins when all are up)
            routes = [(did, _ShowTrajectory(tr.track, tr.pl, tr.frame, tr.t0, 0.0)) for did, tr in routes]
        routes, t_join, notes = from_pads(site, routes)
        duration = t_join + (duration - (SHOW_LEAD_S if s.drones == "show" else 0.0))
        if s.drones == "show":
            show_start = t_join
        extras = {"takeoff_s": t_join, "notes": notes}
    drones = [SimDrone(did, did, route) for did, route in routes]
    model = CameraModel(site.sensor, peak_at_10m=marker_brightness(site, cameras))
    timing = TimingModel(sync="ptp", jitter_sigma_us=5.0, clock_profile=s.clock_profile)
    lo, hi = site.volume_min.copy(), site.volume_max.copy()
    sc = Scenario(name="planned layout", duration_s=float(duration), seed=int(rng.integers(1 << 30)),
                  cameras=list(cameras), station_ids=list(range(1, len(cameras) + 1)), camera_model=model,
                  timing=timing, network=NetworkModel(loss=s.network_loss),
                  markers=MarkerModel(body_radius_m=DRONE_BODY_RADIUS_M), drones=drones,
                  calib_rot_mrad=s.calib_rot_mrad, calib_pos_mm=s.calib_pos_mm, bounds=(lo, hi), extras=extras)
    return sc, show_start


def marker_brightness(site: Site, cameras: list[PlannedCamera]) -> float:
    """Peak (DN) of a marker seen from 10 m, matched to the venue: a marker at the far end of a
    typical camera's range (90th percentile of camera-to-volume distances) peaks at about 160 DN,
    well above the detection threshold. LED current and exposure are set per venue the same way."""
    v = site.volume
    lo, hi = v.bbox_min, v.bbox_max
    pts = np.array([[x, y, z] for x, y in v.polygon for z in (lo[2], hi[2])] + [[*v.polygon.mean(0), (lo[2] + hi[2]) / 2]])
    P = np.array([c.position_m for c in cameras])
    far = np.percentile(np.linalg.norm(P[:, None] - pts[None], axis=2), 90)
    return float(np.clip(160.0 * (far / 10.0) ** 2, 60.0, 6000.0))


def simulate_plan(site: Site, cameras: list[PlannedCamera], s: SimSettings | None = None,
                  progress=None, cancel=None) -> SimResult:
    """Film, run the engine, score. progress(text) is called now and then; cancel() -> True stops
    (raises Cancelled)."""
    from idps.engine.calib import load_calibration
    from idps.engine.engine import Engine, EngineConfig
    from idps.engine.evaluate import score
    from idps.engine.ingest import Ingest
    from idps.sim.runner import MemorySink, Simulation, write_calibration
    s = s or SimSettings()
    say = progress or (lambda _: None)
    stop = cancel or (lambda: False)
    if not cameras:
        raise ValueError("there are no cameras to simulate: plan the layout first")
    sc, show_start = build_scenario(site, cameras, s)
    if not sc.drones:
        raise ValueError("no drones to fly")
    say(f"Setting up {len(cameras)} cameras and {len(sc.drones)} drones…")
    sim = Simulation(sc)
    D, dur = len(sc.drones), sc.duration_s

    def filmed(t, st):
        if stop():
            raise Cancelled()
        say(f"Filming: {t:.0f} of {dur:.0f} s · {st.frames} frames · {st.true_detections / max(1, st.frames):.1f} "
            "markers per frame")

    t0 = time.perf_counter()
    sink = MemorySink()
    stats = sim.run(sink, progress=filmed)
    sim_s = time.perf_counter() - t0

    # truth at the engine's slot rate, straight from the simulator
    tt = np.arange(0.0, dur, 1 / 120.0)
    M = np.empty((D, len(tt), 3))
    for k, t in enumerate(tt):
        M[:, k] = sim.state(t)[1]
    truth = {d.marker_id: (sim.epoch_ns + np.round(tt * 1e9).astype(np.int64), M[i]) for i, d in enumerate(sc.drones)}

    with tempfile.TemporaryDirectory() as tmp:
        cal = Path(tmp) / "calibration.yaml"
        write_calibration(sim, cal)
        eng = Engine(load_calibration(cal, "estimate"), EngineConfig(reorder_s=s.reorder_ms / 1000),
                     bounds=(sc.bounds[0] - [1, 1, 0.5], sc.bounds[1] + [1, 1, 0.5]), frame=site.frame)
    ing = Ingest()
    outs = []
    t0 = time.perf_counter()
    for k, (_, pkt) in enumerate(sink.packets):
        for f in ing.feed(pkt):
            outs += eng.feed(f)
        if k % 20000 == 0:
            if stop():
                raise Cancelled()
            say(f"Positioning engine: {100 * k / max(1, len(sink.packets)):.0f} %")
    outs += eng.flush()
    engine_s = time.perf_counter() - t0
    scores, unknown = score(outs, truth)

    # replay at REPLAY_HZ
    tr = np.arange(0.0, dur, 1 / REPLAY_HZ)
    C = np.empty((D, len(tr), 3))
    Mr = np.empty((D, len(tr), 3))
    for k, t in enumerate(tr):
        c, m, _, _ = sim.state(t)
        C[:, k], Mr[:, k] = c, m
    led = np.stack([[led.level(t) for t in tr] for led in sim.leds]).astype(float)
    E = np.full((D, len(tr), 3), np.nan)
    err = np.full((D, len(tr)), np.nan)
    ncam = np.zeros((D, len(tr)), int)
    index = {d.marker_id: i for i, d in enumerate(sc.drones)}
    for o in outs:
        i = index.get(o.marker_id)
        if i is None:
            continue
        t = (o.t_ns - sim.epoch_ns) * 1e-9
        k = int(round(t * REPLAY_HZ))
        if 0 <= k < len(tr) and abs(t - tr[k]) <= 0.5 / REPLAY_HZ:
            E[i, k] = o.pos
            ncam[i, k] = o.n_cams
            ref = np.array([np.interp(o.t_ns, truth[o.marker_id][0], truth[o.marker_id][1][:, j]) for j in range(3)])
            err[i, k] = 1000 * np.linalg.norm(o.pos - ref)
    drones = [DroneResult(sc.drones[index[r.marker_id]].id, r.median_mm, r.p95_mm, r.max_mm, r.availability,
                          r.first_id_s, r.wrong) for r in scores]
    return SimResult(s, list(cameras), list(sim.stations), [d.id for d in sc.drones], tr, C, Mr, led, E, err, ncam,
                     drones, unknown, stats.frames, stats.packets, sim_s, engine_s, show_start,
                     list(sc.extras.get("notes", [])), sc.extras.get("takeoff_s"))


def marker_in_view(station, marker: np.ndarray) -> np.ndarray:
    """Pixel position (N, 2) of each marker in a station's image, NaN where it is outside it."""
    u, v, z = station.project(np.atleast_2d(marker))
    ok = (z > 0.3) & (u >= 0) & (v >= 0) & (u <= station.W - 1) & (v <= station.H - 1)
    return np.where(ok[:, None], np.stack([u, v], 1), np.nan)
