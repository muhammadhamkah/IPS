"""Simulated show feeding the brain until the positioning engine exists.

Drones fly random waypoints inside the venue's polygon volume. Every tick, each
station's detection of each drone's marker is modelled with the same physics
as the full simulator (field of view, range/brightness, the drone's own body,
other drones in the way, scenery boxes), but per drone rather than per pixel,
so a whole show runs faster than real time. Faults can be injected live: a
station going offline, a camera knocked off its calibration, or an obstruction
hung in front of the cameras covering a region.
"""
from __future__ import annotations

import math

import numpy as np

from idps.monitor.model import StationStatus, TrackSample
from idps.plan.model import PlannedCamera
from idps.plan.site import Site
from idps.sim import trajectory as traj
from idps.sim.camera import SimStation, segments_hit_boxes
from idps.sim.scenario import CameraModel, MarkerModel, TimingModel

REJECT_MOVED_MRAD = 1.0       # the engine rejects rays from a camera this far off its calibration


def _rotation(axis, angle):
    a = np.asarray(axis, float)
    a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


class Formation(traj.Trajectory):
    """A drone in a show formation: the shared route, offset in space and lagging in time."""

    def __init__(self, route: traj.Trajectory, offset, delay_s: float):
        self.route, self.offset, self.delay = route, np.asarray(offset, float), float(delay_s)

    def position(self, t):
        return self.route.position(np.asarray(t, float) - self.delay) + self.offset


class SimShow:
    def __init__(self, site: Site, cameras: list[PlannedCamera], n_drones: int = 6, seed: int = 1,
                 speed_mps: float = 2.5, dropout: float = 0.01, formation: bool = True):
        """formation: drones fly one shared, repeating route in a loose grid (like a show), so the
        same places are visited again and again; otherwise each drone flies its own random loop."""
        self.site = site
        self.rng = np.random.default_rng(seed)
        self.model = CameraModel(site.sensor)
        self.markers = MarkerModel()
        self.timing = TimingModel()
        self.dropout = dropout
        self.t = 0.0
        self.offline: set[str] = set()
        self.bumps: dict[str, tuple[np.ndarray, float]] = {}      # camera -> (rotation applied, mrad)
        self.occluders: list[tuple[np.ndarray, np.ndarray, str]] = []
        self.cams: list[PlannedCamera] = []
        self.stations: dict[str, SimStation] = {}
        self.set_cameras(cameras)
        if formation:
            cols = int(math.ceil(math.sqrt(n_drones)))
            spacing = 2.0
            span = (cols - 1) * spacing
            self.route = traj.Waypoints(self._inside_points(7, margin=0.6 + span / 2 + 0.5, z_margin=1.0), speed_mps)
            self.drones = []
            for k in range(n_drones):
                r, c = divmod(k, cols)
                off = np.array([(c - (cols - 1) / 2) * spacing, (r - (cols - 1) / 2) * spacing,
                                self.rng.uniform(-0.5, 0.5)])
                self.drones.append((k + 1, Formation(self.route, off, 0.15 * k)))
        else:
            self.route = None
            self.drones = [(k + 1, self._random_route(speed_mps)) for k in range(n_drones)]

    # ------------------------------------------------------------ setup
    def _inside_points(self, n, margin=0.6, z_margin=0.5):
        v = self.site.volume
        lo, hi = v.bbox_min, v.bbox_max
        z0 = max(lo[2] + z_margin, 0.5)
        z1 = max(hi[2] - z_margin, z0 + 0.1)
        out = []
        from idps.coverage.volume import distance_to_polygon
        for _ in range(200):
            if len(out) >= n:
                break
            xy = self.rng.uniform(lo[:2], hi[:2], size=(4 * n, 2))
            ok = v.contains_xy(xy) & (distance_to_polygon(xy, v.polygon)[0] >= margin)
            for p in xy[ok]:
                out.append([p[0], p[1], self.rng.uniform(z0, z1)])
        if len(out) < n:                                   # thin volume: relax the margin
            return self._inside_points(n, margin * 0.5, z_margin) if margin > 0.1 else \
                np.array([[*v.polygon.mean(0), (z0 + z1) / 2]] * n)
        return np.array(out[:n])

    def _random_route(self, speed):
        return traj.Waypoints(self._inside_points(10), speed, loop=True)

    def _station(self, k, cam: PlannedCamera) -> SimStation:
        st = SimStation(k, cam.position_m, cam.R, cam.lens_mm, self.model, self.timing, self.markers, self.rng, 1.0)
        st.occluders = [(lo, hi) for lo, hi, _ in self.occluders]
        if cam.id in self.bumps:
            st.R = self.bumps[cam.id][0] @ cam.R
        return st

    def set_cameras(self, cameras: list[PlannedCamera]):
        """The layout as installed changed (a fix was carried out)."""
        old = {c.id: c for c in self.cams}
        self.cams = list(cameras)
        stations = {}
        for k, c in enumerate(self.cams, 1):
            o = old.get(c.id)
            if o is not None and o.position_m == c.position_m and o.azimuth_deg == c.azimuth_deg \
                    and o.elevation_deg == c.elevation_deg and c.id in self.stations:
                stations[c.id] = self.stations[c.id]
            else:
                self.bumps.pop(c.id, None)          # re-installed: back on (a new) calibration
                stations[c.id] = self._station(k, c)
        self.stations = stations
        self.offline &= set(self.stations)

    # ------------------------------------------------------------ faults
    def set_offline(self, cid: str, off: bool = True):
        (self.offline.add if off else self.offline.discard)(cid)

    def bump(self, cid: str, mrad: float = 3.0):
        """Knock a camera: its view rotates by `mrad` about a random axis."""
        axis = self.rng.normal(size=3)
        Rb = _rotation(axis, mrad * 1e-3)
        self.bumps[cid] = (Rb, float(mrad))
        cam = next(c for c in self.cams if c.id == cid)
        self.stations[cid].R = Rb @ cam.R

    def restore(self, cid: str):
        """Servos re-aim the camera to its calibrated view."""
        self.bumps.pop(cid, None)
        cam = next((c for c in self.cams if c.id == cid), None)
        if cam is not None:
            self.stations[cid].R = cam.R.copy()

    def add_occluder(self, lo, hi, name="obstruction"):
        self.occluders.append((np.asarray(lo, float), np.asarray(hi, float), name))
        for st in self.stations.values():
            st.occluders = [(a, b) for a, b, _ in self.occluders]

    def clear_occluders(self):
        self.occluders.clear()
        for st in self.stations.values():
            st.occluders = []

    def obstruct(self, target=None, size_m: float = 0.8, keep: int | None = None):
        """Hang obstructions (a banner, a set piece, rigging) in front of the cameras that
        see `target`, leaving only `keep` (default: one fewer than the requirement) clear.
        Returns (target, blocked camera ids)."""
        from idps.coverage.fisher import contributions
        if target is None:
            if self.route is not None:                     # somewhere the show actually flies
                target = self.route.position(np.array([self.rng.uniform(0, self.route.cum[-1])]))[0]
            else:
                target = self._inside_points(1, margin=2.0)[0]
        target = np.asarray(target, float)
        pos = np.array([c.position_m for c in self.cams])
        R = np.array([c.R for c in self.cams])
        f = np.array([self.site.sensor.f_px(c.lens_mm) for c in self.cams])
        vis, _ = contributions(pos, R, f, target[None], self.site.sensor, self.site.requirements.noise,
                               self.site.limits)
        seeing = [k for k in np.nonzero(vis[:, 0])[0] if self.cams[k].id not in self.offline]
        keep = self.site.requirements.min_cameras - 1 if keep is None else keep
        seeing.sort(key=lambda k: np.linalg.norm(pos[k] - target))
        blocked = seeing[:max(0, len(seeing) - keep)]
        h = size_m / 2
        for k in blocked:
            d = target - pos[k]
            L = np.linalg.norm(d)
            centre = target - d / L * min(2.5, 0.5 * L)        # a prop hanging a couple of metres off

            self.add_occluder(centre - h, centre + h, f"obstruction in front of {self.cams[k].id}")
        return target, [self.cams[k].id for k in blocked]

    # ------------------------------------------------------------ running
    def drone_states(self, t):
        pos, up = [], []
        for _, route in self.drones:
            p = route.position(np.array([t]))[0]
            a = route.acceleration(np.array([t]))[0]
            _, _, z = traj.body_axes(a, 0.0)
            pos.append(p)
            up.append(z)
        return np.array(pos), np.array(up)

    def detections(self, t):
        """Per drone: frozenset of camera ids whose detections the engine would use."""
        P, up = self.drone_states(t)
        markers = P + up * 0.08
        D = len(P)
        seen = [set() for _ in range(D)]
        mk, m = self.markers, self.model
        for cid, st in self.stations.items():
            if cid in self.offline:
                continue
            if self.bumps.get(cid, (None, 0.0))[1] >= REJECT_MOVED_MRAD:
                continue                                   # rays don't agree with the others: rejected
            u, v, z = st.project(markers)
            d = st.pos - markers
            rng_m = np.linalg.norm(d, axis=1)
            cosu = np.einsum("ij,ij->i", d / rng_m[:, None], up)
            ok = (z > 0.3) & (u >= 3) & (u <= st.W - 4) & (v >= 3) & (v <= st.H - 4)
            ok &= cosu > mk.self_occlusion_cos
            emission = mk.emission_floor + (1 - mk.emission_floor) * np.maximum(0.0, cosu)
            peak = m.peak_at_10m * 0.5 * (mk.high + mk.low) * emission * (10.0 / np.maximum(rng_m, 0.1)) ** 2
            ok &= peak >= m.threshold_dn
            ok &= ~st._occluded(markers, P, np.arange(D))
            ok &= ~segments_hit_boxes(st.pos, markers, st.occluders)
            ok &= self.rng.random(D) >= self.dropout
            for i in np.nonzero(ok)[0]:
                seen[i].add(cid)
        return P, [frozenset(s) for s in seen]

    def step(self, dt: float = 0.1):
        """Advance the show; returns (track samples, station statuses) for this tick."""
        self.t += dt
        P, seen = self.detections(self.t)
        samples = [TrackSample(self.t, did, P[i], seen[i]) for i, (did, _) in enumerate(self.drones)]
        statuses = []
        for cid in self.stations:
            on = cid not in self.offline
            mrad = self.bumps.get(cid, (None, 0.0))[1]
            noise = abs(self.rng.normal(0, 0.03))
            statuses.append(StationStatus(self.t, cid, on, self.model.fps if on else 0.0, mrad + noise if on else 0.0))
        return samples, statuses
