"""Simulation scenario (YAML): which cameras, which drones, and how imperfect
the world is. See examples/scenarios/*.yaml."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from idps.core.camera import SENSORS, Sensor
from idps.plan.model import Plan, PlannedCamera
from idps.sim import trajectory as traj


@dataclass
class CameraModel:
    sensor: Sensor
    fps: float = 120.0
    exposure_us: float = 500.0
    distortion: tuple = (-0.05, 0.01, 0.0, 0.0, 0.0)     # k1, k2, p1, p2, k3 (Brown-Conrady)
    psf_sigma_px: float = 1.2         # blob size (deliberate slight defocus)
    centroid_floor_px: float = 0.03   # centroid noise at high SNR
    shot_noise_k: float = 1.0         # extra centroid noise = k / sqrt(sum)
    pixel_locking: bool = True        # bias towards pixel centres for small blobs
    saturation_dn: int = 1023         # 10-bit sensor
    threshold_dn: int = 20
    peak_at_10m: float = 600.0        # HIGH marker, on-axis, 10 m, at exposure_ref_us
    exposure_ref_us: float = 500.0
    false_positive_rate: float = 0.02 # random blobs per frame
    static_reflections: int = 1       # fixed spurious blobs per camera (stationary, to be masked)
    frame_drop_prob: float = 1e-4
    sensor_clock_ppm: float = 20.0    # free-run frame-rate error (1-sigma)
    proc_latency_ms: float = 3.0      # exposure end -> packet sent


@dataclass
class TimingModel:
    sync: str = "ptp"                 # free_run | ptp | hw_trigger
    offset_sigma_us: float = 20.0     # residual clock offset per station after sync
    jitter_sigma_us: float = 10.0     # per-frame timestamp jitter
    drift_ppm: float = 0.0            # residual clock drift per station (1-sigma)
    trigger_jitter_us: float = 2.0    # hw_trigger: capture-time spread across stations
    # Simulated clock synchronisation (idps.sim.clocksync). When set, each station's
    # timestamps follow a simulated disciplined clock instead of the fixed offset/drift above:
    # pi5_ptp_hw (chosen) | pi5_ptp_sw | pi5_ntp | pi5_wifi_ntp | pi3b_ptp_sw (comparison) | free_run
    clock_profile: str | None = None
    clock_start: str = "synced"       # "synced": a running system | "power_on": include start-up


@dataclass
class NetworkModel:
    loss: float = 0.001
    latency_ms: float = 2.0
    jitter_ms: float = 1.0


@dataclass
class MarkerModel:
    chip_rate_hz: float = 30.0
    high: float = 1.0
    low: float = 0.4
    clock_ppm: float = 50.0           # LED controller clock error (1-sigma)
    emission_floor: float = 0.55      # diffuser: brightness = floor + (1-floor) * max(0, cos(angle from drone up))
    self_occlusion_cos: float = -0.3  # camera this far below the drone's horizontal plane is blocked by the body
    body_radius_m: float = 0.2        # other drones block rays within this radius


@dataclass
class SimDrone:
    id: int
    marker_id: int
    trajectory: traj.Trajectory
    yaw_deg: float = 0.0
    marker_offset_m: tuple = (0.0, 0.0, 0.08)    # marker position in the body frame


@dataclass
class Scenario:
    name: str
    duration_s: float
    seed: int
    cameras: list[PlannedCamera]
    station_ids: list[int]
    camera_model: CameraModel
    timing: TimingModel
    network: NetworkModel
    markers: MarkerModel
    drones: list[SimDrone]
    calib_rot_mrad: float = 0.05
    calib_pos_mm: float = 2.0
    bounds: tuple | None = None
    source_path: str | None = None
    extras: dict = field(default_factory=dict)


def _update(dc, d: dict | None):
    for k, v in (d or {}).items():
        if not hasattr(dc, k):
            raise ValueError(f"unknown setting {k!r} for {type(dc).__name__}")
        setattr(dc, k, tuple(v) if isinstance(getattr(dc, k), tuple) else v)
    return dc


def _swarm(cfg: dict, rng, first_id: int, first_marker: int, bounds) -> list[SimDrone]:
    n = int(cfg["count"])
    center = np.asarray(cfg.get("center", [0, 0, 3]), float)
    spacing = float(cfg.get("spacing", 2.0))
    motion = cfg.get("motion", "circle")
    speed = float(cfg.get("speed", 2.0))
    cols = int(math.ceil(math.sqrt(n)))
    out = []
    for k in range(n):
        r, c = divmod(k, cols)
        home = center + np.array([(c - (cols - 1) / 2) * spacing, (r - (cols - 1) / 2) * spacing, 0.0])
        if motion == "hover":
            t = traj.Hover(home)
        elif motion == "circle":
            t = traj.Circle(home, float(cfg.get("radius", spacing / 3)), speed, phase_deg=360.0 * k / n)
        elif motion == "random":
            t = traj.from_config({"type": "random", "speed": speed, "waypoints": 8}, rng, bounds)
        else:
            raise ValueError(f"unknown swarm motion {motion!r}")
        out.append(SimDrone(first_id + k, (first_marker + k) % 256, t, float(cfg.get("yaw_deg", 0.0)),
                            tuple(cfg.get("marker_offset_m", (0.0, 0.0, 0.08)))))
    return out


def _find(p, base: Path) -> Path | None:
    """Resolve a path written relative to the scenario, the repo, or the cwd."""
    if not p:
        return None
    p = Path(p)
    for c in (p, base / p, *(parent / p for parent in base.parents)):
        if c.exists():
            return c
    return None


def load_scenario(path: str | Path) -> Scenario:
    path = Path(path)
    with open(path) as f:
        d = yaml.safe_load(f)
    base = path.parent
    plan = Plan.load(base / d["plan"])
    rng = np.random.default_rng(int(d.get("seed", 1)))

    cm_cfg = dict(d.get("camera_model", {}) or {})
    sensor = SENSORS[cm_cfg.pop("sensor", "OV9281")]
    cam_model = _update(CameraModel(sensor), cm_cfg)

    bounds = None
    site_path = _find(d.get("site") or plan.site_file, base)
    if "bounds" in d:
        bounds = (np.asarray(d["bounds"]["min"], float), np.asarray(d["bounds"]["max"], float))
    elif site_path is not None:
        from idps.plan.site import load_site
        site = load_site(site_path)
        lo, hi = site.volume_min.copy(), site.volume_max.copy()
        lo[2] = max(lo[2], 0.5)
        bounds = (lo + [0.5, 0.5, 0], hi - [0.5, 0.5, 0.5])

    drones: list[SimDrone] = []
    for k, dd in enumerate(d.get("drones", []) or []):
        drones.append(SimDrone(int(dd.get("id", k + 1)), int(dd["marker_id"]),
                               traj.from_config(dd["trajectory"], rng, bounds), float(dd.get("yaw_deg", 0.0)),
                               tuple(dd.get("marker_offset_m", (0.0, 0.0, 0.08)))))
    if d.get("swarm"):
        sw = d["swarm"]
        drones += _swarm(sw, rng, first_id=len(drones) + 1, first_marker=int(sw.get("first_marker_id", 1)),
                         bounds=bounds)
    ids = [x.marker_id for x in drones]
    if len(set(ids)) != len(ids):
        raise ValueError("marker ids must be unique")

    cal = d.get("calibration_error", {}) or {}
    return Scenario(
        name=str(d.get("name", path.stem)), duration_s=float(d.get("duration_s", 10.0)), seed=int(d.get("seed", 1)),
        cameras=plan.cameras, station_ids=list(range(1, len(plan.cameras) + 1)), camera_model=cam_model,
        timing=_update(TimingModel(), d.get("timing")), network=_update(NetworkModel(), d.get("network")),
        markers=_update(MarkerModel(), d.get("markers")), drones=drones,
        calib_rot_mrad=float(cal.get("rot_mrad", 0.05)), calib_pos_mm=float(cal.get("pos_mm", 2.0)),
        bounds=bounds, source_path=str(path))
