"""The positioning engine: camera frames in, marker (drone) positions out.

For every engine epoch (1/120 s by default) it takes the frames the cameras captured in
that slot and:

1. **matches tracked markers**: each track's predicted position at a camera's exposure time
   is projected into that camera; the nearest dot within the gate is that marker
   (globally, nearest pairs first, each dot used once);
2. **triangulates** each track from its matched dots (rays shifted to one instant with the
   track's velocity), dropping a camera whose dot misses the solution by > 3 px;
3. **updates** the track's Kalman filter and records the dots' brightness for the ID decoder;
4. **starts new tracks** from the dots left over: pairs of cameras whose rays nearly cross
   inside the volume, confirmed by a dot in at least one more camera (3 cameras in all);
5. **reads IDs**: every half second each track's blink code is decoded from the last 2.5 s
   of brightness; two agreeing decodes name the track.

Output: for each confirmed track with an ID, its position (world frame and, if a venue
frame is given, NED), covariance and the number of cameras.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from idps.engine.calib import CameraCal, rays_batch
from idps.engine.idcode import decode
from idps.engine.ingest import CameraFrame
from idps.engine.tracker import CONFIRMED, TENTATIVE, KalmanCV, Track
from idps.protocol.packets import MFLAG_ELONGATED as ELONGATED
from idps.engine.triangulate import closest_approach, triangulate, triangulate_many


class StaticMask:
    """Learns each camera's fixed reflections: among the dots *not* matched to a track, one
    that stays within 0.5 px of the same spot for ``hold_s``. Such dots are not used to start
    tracks. (A drone resting on a spot keeps its track, so its dot is matched, not masked; if
    its track is lost, its dot is new to the mask and gets ``hold_s`` to be picked up again.)"""

    def __init__(self, hold_s: float = 2.0, radius_px: float = 0.5):
        self.hold, self.r = hold_s, radius_px
        self.xy = np.zeros((0, 2))
        self.first = np.zeros(0)
        self.last = np.zeros(0)

    def known(self, xy: np.ndarray, t: float) -> np.ndarray:
        """Dots (N, 2) sitting on a spot already known to be a fixed reflection (no update)."""
        if not len(self.xy) or not len(xy):
            return np.zeros(len(xy), bool)
        d = np.abs(xy[:, None, :] - self.xy[None, :, :]).max(axis=2)
        j = np.argmin(d, axis=1)
        return (d[np.arange(len(xy)), j] < self.r) & (t - self.first[j] > self.hold)

    def classify(self, xy: np.ndarray, t: float) -> np.ndarray:
        n = len(xy)
        static = np.zeros(n, bool)
        new = np.ones(n, bool)
        if len(self.xy) and n:
            d = np.abs(xy[:, None, :] - self.xy[None, :, :]).max(axis=2)
            j = np.argmin(d, axis=1)
            hit = d[np.arange(n), j] < self.r
            new = ~hit
            jj = j[hit]
            self.first[jj] = np.where(t - self.last[jj] > 0.1, t, self.first[jj])
            self.last[jj] = t
            static[hit] = t - self.first[jj] > self.hold
        if new.any():
            self.xy = np.concatenate([self.xy, xy[new]])
            self.first = np.concatenate([self.first, np.full(new.sum(), t)])
            self.last = np.concatenate([self.last, np.full(new.sum(), t)])
        keep = t - self.last <= 0.1                               # a spot missed for 0.1 s starts over anyway
        if not keep.all():
            self.xy, self.first, self.last = self.xy[keep], self.first[keep], self.last[keep]
        return static


@dataclass
class EngineConfig:
    epoch_hz: float = 120.0
    reorder_s: float = 0.008          # wait this long for late packets before closing an epoch
                                      # (wired PTP stations: 4-8 ms is enough; Wi-Fi stations need ~25 ms)
    sigma_px: float = 0.15            # centroid + calibration noise per ray, pixels
    gate_px: float = 4.0              # matching gate around a track's prediction (plus its uncertainty)
    max_resid_px: float = 3.0         # a ray missing the triangulated point by more is dropped
    init_gap_m: float = 0.03          # rays this close count as crossing (plus 0.3 % of range)
    init_support_px: float = 2.5      # a third camera's dot must be this close to the candidate
    min_cams_init: int = 3            # cameras needed to start a track
    min_cams_update: int = 2          # cameras needed to update a track
    confirm_hits: int = 8             # updates needed (within confirm_window epochs) to confirm
    confirm_window: int = 16
    coast_s: float = 0.5              # delete a track not updated for this long
    q_accel: float = 400.0            # Kalman process noise (m²/s³); drones change acceleration quickly
    innov_gate: float = 25.0          # reject a measurement this far from the prediction (Mahalanobis², ~5 sigma) ...
    innov_max_rejects: int = 3        # ... unless it happens this many times in a row (then it is real motion)
    inherit_radius_m: float = 0.5     # a new track this close to a lost one takes over its ID ...
    inherit_within_s: float = 1.0     # ... if that track was lost less than this long ago
    decode_every_s: float = 0.5       # read a track's blink code this often until it has an ID ...
    decode_every_named_s: float = 2.0 # ... and this often afterwards (to keep checking it)
    decodes_per_epoch: int = 2        # at most this many decodes per 1/120 s slot (spreads the work)
    decode_window_s: float = 2.5
    id_votes_needed: int = 2


@dataclass
class Output:
    t_ns: int
    marker_id: int
    pos: np.ndarray                   # world (drawing frame, Z up)
    vel: np.ndarray
    cov: np.ndarray
    n_cams: int
    track: int
    ned: np.ndarray | None = None


class Engine:
    def __init__(self, cams: dict[int, CameraCal], config: EngineConfig | None = None, bounds=None,
                 frame=None, backend: str = "auto"):
        """bounds: (lo, hi) world box where markers can be (default: around the cameras).
        frame: idps.core.frames.Frame to also output NED.
        backend: "cpp" (the compiled core, idps/engine/_core.cpp), "python" (this file, the
        reference) or "auto" (the core if it is built, else Python)."""
        self.cams = cams
        self.cfg = config or EngineConfig()
        P = np.array([c.position for c in cams.values()])
        self.bounds = bounds or (np.r_[P[:, :2].min(0) - 1.0, -0.3], np.r_[P[:, :2].max(0) + 1.0, P[:, 2].max()])
        self.frame = frame
        self.t0_ns: int | None = None
        self.buf: list[CameraFrame] = []
        self.next_epoch = 0
        self.tracks: list[Track] = []
        self.lost: list[tuple] = []           # (time lost, marker id, position, velocity, votes) of identified tracks
        self.n_tracks = 0
        self.sigma_ang = {sid: self.cfg.sigma_px / c.f_px for sid, c in cams.items()}
        self.static = {sid: StaticMask() for sid in cams}
        self.stats = {"epochs": 0, "frames": 0, "unknown_station": 0, "tracks_started": 0, "outputs": 0}
        self._core = None
        if backend not in ("auto", "cpp", "python"):
            raise ValueError(f"unknown backend {backend!r}")
        if backend != "python":
            try:
                from idps.engine import _core
            except ImportError:
                if backend == "cpp":
                    raise ImportError("the C++ engine core is not built: pip install -e . (needs a C++ compiler)")
            else:
                self._core = self._make_core(_core)
        self.backend = "cpp" if self._core is not None else "python"

    def _make_core(self, _core):
        cams = list(self.cams.values())
        self._cam_index = {c.station_id: i for i, c in enumerate(cams)}
        conf = {k: v for k, v in vars(self.cfg).items() if k != "epoch_hz"}
        return _core.Core(np.array([c.station_id for c in cams], np.int32), np.array([c.position for c in cams]),
                          np.array([c.R for c in cams]), np.array([c.f_px for c in cams]),
                          np.array([c.cx for c in cams]), np.array([c.cy for c in cams]),
                          np.array([c.distortion for c in cams]),
                          np.array([[c.width, c.height] for c in cams], np.int32),
                          np.asarray(self.bounds[0], float), np.asarray(self.bounds[1], float), conf)

    @property
    def n_live_tracks(self) -> int:
        return len(self._core.track_states()) if self._core is not None else len(self.tracks)

    def _process_core(self, frames: list[CameraFrame]) -> list[Output]:
        fcam = np.array([self._cam_index[f.station_id] for f in frames], np.int32)
        ft = np.array([self._t(f.t_ns) for f in frames])
        nz = [f for f in frames if f.n]
        dframe = np.repeat(np.arange(len(frames), dtype=np.int32), [f.n for f in frames])
        xy = np.concatenate([f.xy for f in nz]) if nz else np.zeros((0, 2))
        sm = np.concatenate([f.sum for f in nz]) if nz else np.zeros(0)
        fl = np.concatenate([f.mflags for f in nz]).astype(np.int32) if nz else np.zeros(0, np.int32)
        r = self._core.process(fcam, ft, dframe, np.ascontiguousarray(xy, float), np.ascontiguousarray(sm, float), fl)
        self.stats["tracks_started"] = self._core.tracks_started
        out = []
        ned = self.frame.ned(r["pos"]) if (self.frame is not None and len(r["pos"])) else None
        for k in range(len(r["track"])):
            o = Output(self.t0_ns + int(round(r["t"][k] * 1e9)), int(r["marker_id"][k]), r["pos"][k].copy(),
                       r["vel"][k].copy(), r["cov"][k].copy(), int(r["n_cams"][k]), int(r["track"][k]))
            if ned is not None:
                o.ned = ned[k]
            out.append(o)
        self.stats["outputs"] += len(out)
        return out

    # ---------------------------------------------------------------- time
    def _t(self, t_ns: int) -> float:
        return (t_ns - self.t0_ns) * 1e-9

    def feed(self, frame: CameraFrame) -> list[Output]:
        """Add one camera frame; returns the outputs of any epochs that became complete."""
        if frame.station_id not in self.cams:
            self.stats["unknown_station"] += 1
            return []
        if self.t0_ns is None:
            self.t0_ns = frame.t_ns
        t = self._t(frame.t_ns)
        self.buf.append(frame)
        self.stats["frames"] += 1
        self.latest = latest = max(getattr(self, "latest", t), t)
        out = []
        period = 1.0 / self.cfg.epoch_hz
        while (self.next_epoch + 1) * period + self.cfg.reorder_s <= latest:
            out += self._close_epoch(self.next_epoch)
            self.next_epoch += 1
        return out

    def flush(self) -> list[Output]:
        out = []
        if not self.buf:
            return out
        period = 1.0 / self.cfg.epoch_hz
        last = max(self._t(f.t_ns) for f in self.buf)
        while self.buf and self.next_epoch * period <= last:
            out += self._close_epoch(self.next_epoch)
            self.next_epoch += 1
        return out

    def _close_epoch(self, k: int) -> list[Output]:
        period = 1.0 / self.cfg.epoch_hz
        lo, hi = k * period, (k + 1) * period
        frames = [f for f in self.buf if lo <= self._t(f.t_ns) < hi]
        self.buf = [f for f in self.buf if self._t(f.t_ns) >= hi]       # late frames of past epochs are dropped
        if not frames:
            return []
        self.stats["epochs"] += 1
        return self._process_core(frames) if self._core is not None else self._process(frames)

    # ---------------------------------------------------------------- one epoch
    def _process(self, frames: list[CameraFrame]) -> list[Output]:
        cfg = self.cfg
        times = np.array([self._t(f.t_ns) for f in frames])
        T = float(np.median(times))
        # lines of sight for every dot of the epoch, all cameras in one pass
        owners = [self.cams[f.station_id] for f in frames for _ in range(f.n)]
        if owners:
            R = rays_batch(owners, np.concatenate([f.xy for f in frames if f.n]))
            k = 0
            for f in frames:
                f.rays, k = R[k:k + f.n], k + f.n
        for f in frames:
            if f.rays is None:
                f.rays = np.zeros((0, 3))
        for tr in self.tracks:
            tr.epochs += 1

        used = [np.zeros(f.n, bool) for f in frames]
        assigned = self._associate(frames, times, T, used)

        self._update_tracks(assigned, frames, times, T, used)

        for fi, f in enumerate(frames):                 # fixed reflections, among the dots left over
            f.static = np.zeros(f.n, bool)
            free = np.nonzero(~used[fi])[0]
            if len(free):
                f.static[free] = self.static[f.station_id].classify(f.xy[free], times[fi])
        self._start_tracks(frames, times, T, used)

        # life cycle
        alive = []
        for i, tr in enumerate(self.tracks):
            if tr.state == TENTATIVE:
                if tr.hits >= cfg.confirm_hits:
                    tr.state = CONFIRMED
                elif tr.epochs > cfg.confirm_window:
                    continue
            if T - tr.last_update > cfg.coast_s:
                if tr.marker_id is not None:
                    self.lost.append((tr.last_update, tr.marker_id, tr.kf.pos.copy(), tr.kf.vel.copy(), dict(tr.id_votes)))
                continue
            alive.append(tr)
        self.tracks = alive
        self.lost = [l for l in self.lost if T - l[0] < cfg.inherit_within_s]
        for tr in list(self.tracks):                   # unnamed tracks where a named one was just lost
            if tr.marker_id is None and tr in self.tracks and tr.updated_epoch == self.stats["epochs"]:
                self._inherit(tr, T)
        self._dedupe()

        due = sorted((tr for tr in self.tracks if tr.state == CONFIRMED and T >= tr.next_decode),
                     key=lambda tr: (tr.next_decode, tr.tid))
        for tr in due[:cfg.decodes_per_epoch]:
            self._decode(tr, T)
        out = []
        for tr in self.tracks:
            if tr.state == CONFIRMED and tr.marker_id is not None and tr.updated_epoch == self.stats["epochs"]:
                o = Output(self.t0_ns + int(round(tr.kf.t * 1e9)), tr.marker_id, tr.kf.pos.copy(), tr.kf.vel.copy(),
                           tr.kf.P[:3, :3].copy(), tr.n_cams, tr.tid)
                if self.frame is not None:
                    o.ned = self.frame.ned(o.pos[None])[0]
                out.append(o)
        self.stats["outputs"] += len(out)
        return out

    def _predict(self, idx, t):
        """Kalman predict of tracks ``idx`` to their times ``t`` (constant velocity,
        white-noise acceleration), all at once."""
        if not len(idx):
            return
        trs = [self.tracks[i] for i in idx]
        x = np.array([tr.kf.x for tr in trs])
        P = np.array([tr.kf.P for tr in trs])
        t = np.asarray(t, float)
        dt = np.maximum(t - np.array([tr.kf.t for tr in trs]), 0.0)
        q = self.cfg.q_accel
        x[:, :3] += x[:, 3:] * dt[:, None]
        I3 = np.eye(3)
        F = np.tile(np.eye(6), (len(dt), 1, 1))
        F[:, :3, 3:] = I3 * dt[:, None, None]
        Q = np.zeros_like(P)
        Q[:, :3, :3] = I3 * (q * dt ** 3 / 3)[:, None, None]
        Q[:, :3, 3:] = Q[:, 3:, :3] = I3 * (q * dt ** 2 / 2)[:, None, None]
        Q[:, 3:, 3:] = I3 * (q * dt)[:, None, None]
        P = np.einsum("nij,njk,nlk->nil", F, P, F) + Q
        for tr, xi, Pi, ti in zip(trs, x, P, np.maximum(t, [tr.kf.t for tr in trs])):
            tr.kf.x, tr.kf.P, tr.kf.t = xi, Pi, float(ti)

    def _associate(self, frames, times, T, used) -> dict[int, list]:
        cfg = self.cfg
        if not self.tracks:
            return {}
        fis = [fi for fi, f in enumerate(frames) if f.n]
        if not fis:
            return {}
        pos = np.array([tr.kf.pos for tr in self.tracks])
        vel = np.array([tr.kf.vel for tr in self.tracks])
        tk = np.array([tr.kf.t for tr in self.tracks])
        sig = np.sqrt(np.array([np.trace(tr.kf.P[:3, :3]) / 3 for tr in self.tracks]))
        sig = np.hypot(sig, np.sqrt(self.cfg.q_accel * np.maximum(T - tk, 0) ** 3 / 3))   # grown to now
        cams = [self.cams[frames[fi].station_id] for fi in fis]
        # every track's prediction at every frame's exposure time, in every camera: (F, T)
        Xp = pos[None] + vel[None] * (times[fis][:, None] - tk[None, :])[:, :, None]
        C = np.array([c.position for c in cams])
        Rm = np.array([c.R for c in cams])
        Xc = np.einsum("ftj,fij->fti", Xp - C[:, None, :], Rm)
        z = Xc[..., 2]
        zs = np.where(z > 1e-9, z, 1e-9)
        xn, yn = Xc[..., 0] / zs, Xc[..., 1] / zs
        k1, k2, p1, p2, k3 = (a[:, None] for a in np.array([c.distortion for c in cams]).T)
        r2 = xn * xn + yn * yn
        radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        fpx = np.array([c.f_px for c in cams])[:, None]
        U = np.array([c.cx for c in cams])[:, None] + fpx * (xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn))
        V = np.array([c.cy for c in cams])[:, None] + fpx * (yn * radial + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn)
        W = np.array([c.width for c in cams])[:, None]
        H = np.array([c.height for c in cams])[:, None]
        m = cfg.gate_px
        vis = (z > 0.3) & (U >= -m) & (U <= W - 1 + m) & (V >= -m) & (V <= H - 1 + m)
        gate = cfg.gate_px + 3 * sig[None, :] * fpx / np.maximum(z, 0.3)
        # a moving marker cannot be on a known fixed reflection (only a resting one can)
        moving = np.linalg.norm(vel, axis=1) > 0.05
        cand = []                                                      # (distance px, track, frame, dot)
        for k, fi in enumerate(fis):
            if not vis[k].any():
                continue
            f = frames[fi]
            xy = f.xy
            d = np.hypot(xy[None, :, 0] - U[k][:, None], xy[None, :, 1] - V[k][:, None])
            ok = (d < gate[k][:, None]) & vis[k][:, None]
            refl = self.static[f.station_id].known(xy, times[fi])
            if refl.any():
                ok &= ~(moving[:, None] & refl[None, :])
            for ti, di in zip(*np.nonzero(ok)):
                cand.append((d[ti, di], ti, fi, di))
        cand.sort()
        taken_dot, taken_tf = set(), set()
        assigned: dict[int, list] = {}
        for _, ti, fi, di in cand:
            if (fi, di) in taken_dot or (ti, fi) in taken_tf:
                continue
            taken_dot.add((fi, di))
            taken_tf.add((ti, fi))
            assigned.setdefault(ti, []).append((fi, di))
            used[fi][di] = True
        return assigned

    def _rays(self, frames, obs):
        O, D, F, S = [], [], [], []
        for fi, di in obs:
            f = frames[fi]
            cam = self.cams[f.station_id]
            O.append(cam.position)
            D.append(f.rays[di])
            F.append(cam.f_px)
            S.append(f.station_id)
        return np.array(O), np.array(D), np.array(F), S

    def _update_tracks(self, assigned, frames, times, T, used):
        """Triangulate every matched track at once and update their filters. Dots of a track
        that could not be updated are freed again for starting new tracks."""
        cfg = self.cfg
        for ti, obs in list(assigned.items()):
            # a dot the camera flags as elongated is two markers merged: its centre lies between
            # them. Leave it out when the track has 3 other cameras.
            clean = [(fi, di) for fi, di in obs if not frames[fi].mflags[di] & ELONGATED]
            if len(clean) >= 3 and len(clean) < len(obs):
                for fi, di in obs:
                    if (fi, di) not in clean:
                        used[fi][di] = False
                assigned[ti] = clean
        rows = [(ti, fi, di) for ti, obs in assigned.items() if len(obs) >= cfg.min_cams_update
                for fi, di in obs]
        for ti, obs in assigned.items():
            if len(obs) < cfg.min_cams_update:
                for fi, di in obs:
                    used[fi][di] = False
        if not rows:
            return
        tix = np.array([r[0] for r in rows])
        fis = [r[1] for r in rows]
        O = np.array([self.cams[frames[fi].station_id].position for fi in fis])
        D = np.array([frames[fi].rays[di] for _, fi, di in rows])
        F = np.array([self.cams[frames[fi].station_id].f_px for fi in fis])
        n = len(self.tracks)
        # each track is measured at the mean exposure time of the cameras that saw it: the
        # motion corrections then average to zero, so a velocity error hardly moves the point
        tobs = times[fis]
        Tm = np.bincount(tix, tobs, minlength=n) / np.maximum(np.bincount(tix, minlength=n), 1)
        upd = np.unique(tix)
        self._predict(upd, Tm[upd])
        dt = tobs - Tm[tix]
        X0 = np.array([tr.kf.pos for tr in self.tracks])
        V = np.array([tr.kf.vel for tr in self.tracks])
        sig = max(self.sigma_ang.values())
        X, cov, ok, keep, ncam = triangulate_many(tix, O, D, F, X0, V[tix] * dt[:, None], n, sig,
                                                   cfg.max_resid_px, cfg.min_cams_update)
        # batched Kalman update of the tracks that triangulated
        rejected: set[int] = set()
        idx = np.nonzero(ok)[0]
        if len(idx):
            x = np.array([self.tracks[i].kf.x for i in idx])
            P = np.array([self.tracks[i].kf.P for i in idx])
            S = P[:, :3, :3] + cov[idx] + np.eye(3) * 1e-8
            Si = np.linalg.inv(S)
            y = X[idx] - x[:, :3]
            d2 = np.einsum("ni,nij,nj->n", y, Si, y)
            K = np.einsum("nij,njk->nik", P[:, :, :3], Si)
            x = x + np.einsum("nij,nj->ni", K, y)
            P = P - np.einsum("nij,njk->nik", K, P[:, :3, :])
            for k, i in enumerate(idx):
                tr = self.tracks[i]
                if tr.state == CONFIRMED and d2[k] > cfg.innov_gate and tr.rejects < cfg.innov_max_rejects:
                    tr.rejects += 1                    # an outlier (e.g. a wrongly matched dot): coast this epoch,
                    rejected.add(i)                    # but keep its dots (no new track from them)
                    continue
                tr.rejects = 0
                tr.kf.x, tr.kf.P = x[k], P[k]
                tr.hits += 1
                tr.last_update = T
                tr.updated_epoch = self.stats["epochs"]
                tr.n_cams = int(ncam[i])
        for r, (ti, fi, di) in enumerate(rows):
            f = frames[fi]
            if ti in rejected:
                continue
            if not ok[ti]:
                used[fi][di] = False
                continue
            if keep[r]:
                self.tracks[ti].add_brightness(times[fi], f.station_id, f.sum[di])

    def _start_tracks(self, frames, times, T, used):
        cfg = self.cfg
        lo, hi = self.bounds
        free = [np.nonzero(~u & ~f.static)[0] for u, f in zip(used, frames)]
        rays = [f.rays[idx] for f, idx in zip(frames, free)]
        cands = []
        for a in range(len(frames)):
            for b in range(a + 1, len(frames)):
                if not len(free[a]) or not len(free[b]) or frames[a].station_id == frames[b].station_id:
                    continue
                oa, ob = self.cams[frames[a].station_id].position, self.cams[frames[b].station_id].position
                da, db = rays[a][:, None, :], rays[b][None, :, :]
                X, gap, s, t = closest_approach(oa, da, ob, db)
                ok = (gap < cfg.init_gap_m + 0.003 * np.maximum(s, t)) & (s > 0.3) & (t > 0.3)
                ok &= np.all((X >= lo) & (X <= hi), axis=-1)
                for i, j in zip(*np.nonzero(ok)):
                    cands.append((X[i, j], [(a, free[a][i]), (b, free[b][j])], gap[i, j]))
        if not cands:
            return
        scored = []
        for X, obs, gap in cands:
            support = list(obs)
            for c in range(len(frames)):
                if c in (obs[0][0], obs[1][0]) or not len(free[c]):
                    continue
                cam = self.cams[frames[c].station_id]
                u, v, z = cam.project(X[None])
                if z[0] <= 0.3:
                    continue
                d = np.hypot(frames[c].xy[free[c], 0] - u[0], frames[c].xy[free[c], 1] - v[0])
                j = int(np.argmin(d))
                if d[j] < cfg.init_support_px:
                    support.append((c, free[c][j]))
            if len({frames[c].station_id for c, _ in support}) >= cfg.min_cams_init:
                scored.append((-len(support), gap, X, support))
        scored.sort(key=lambda r: (r[0], r[1]))
        taken = set()
        for _, _, X, support in scored:
            if any(s in taken for s in support):
                continue
            O, D, F, S = self._rays(frames, support)
            p = triangulate(O, D, F, max(self.sigma_ang[s] for s in S), max_resid_px=cfg.max_resid_px * 2,
                            min_cams=cfg.min_cams_init)
            if p is None:
                continue
            taken.update(support)
            self.n_tracks += 1
            tr = Track(self.n_tracks, KalmanCV(p.X, T, p.cov + np.eye(3) * 1e-6, q=cfg.q_accel), T)
            tr.last_update = T
            tr.n_cams = p.n
            tr.next_decode = T + 1.0
            self._inherit(tr, T)
            for (fi, di), keep in zip(support, p.used):
                used[fi][di] = True
                if keep:
                    tr.add_brightness(times[fi], frames[fi].station_id, frames[fi].sum[di])
            self.tracks.append(tr)
            self.stats["tracks_started"] += 1

    def _inherit(self, tr: Track, T: float):
        """A new track where an identified marker was just lost (an occlusion, a sudden jump)
        takes over that ID straight away, instead of waiting ~3 s to read the blink code again.
        An identified track that is still coasting there (lost its dots to this one) hands over."""
        cfg = self.cfg
        best = None
        for other in self.tracks:
            if other.marker_id is not None and other.updated_epoch < self.stats["epochs"] - 2:
                dt = T - other.kf.t
                d = np.linalg.norm(other.kf.pos + other.kf.vel * dt - tr.kf.pos)
                if d < cfg.inherit_radius_m + 4.0 * dt and (best is None or d < best[0]):
                    best = (d, other.marker_id, dict(other.id_votes), other)
        for t_lost, mid, pos, vel, votes in self.lost:
            dt = T - t_lost
            d = np.linalg.norm(pos + vel * dt - tr.kf.pos)
            if d < cfg.inherit_radius_m + 4.0 * dt and (best is None or d < best[0]):
                best = (d, mid, votes, None)
        if best is None or any(o.marker_id == best[1] and o is not best[3] for o in self.tracks):
            return
        _, mid, votes, old = best
        if old is not None:
            self.tracks.remove(old)
        self.lost = [l for l in self.lost if l[1] != mid]
        tr.marker_id, tr.id_votes = mid, votes
        tr.hits = max(tr.hits, self.cfg.confirm_hits - 2)      # confirm quickly: it is a known marker

    def _dedupe(self):
        """Two tracks on the same marker: keep the older (more history)."""
        if len(self.tracks) < 2:
            return
        P = np.array([t.kf.pos for t in self.tracks])
        d = np.linalg.norm(P[:, None] - P[None], axis=2)
        i, j = np.nonzero(np.triu(d < 0.02, 1))
        if len(i):
            drop = {max(self.tracks[a].tid, self.tracks[b].tid) for a, b in zip(i, j)}
            self.tracks = [t for t in self.tracks if t.tid not in drop]

    def _decode(self, tr: Track, T: float):
        cfg = self.cfg
        tr.next_decode = T + (cfg.decode_every_named_s if tr.marker_id is not None else cfg.decode_every_s)
        t, lv = tr.levels(T - cfg.decode_window_s)
        mid, n, _ = decode(t, lv)
        if mid is None:
            return
        tr.id_votes[mid] = tr.id_votes.get(mid, 0) + n
        best = max(tr.id_votes, key=tr.id_votes.get)
        if tr.id_votes[best] >= cfg.id_votes_needed and best != tr.marker_id:
            # the same ID on another track: the one with more votes keeps it
            for other in self.tracks:
                if other is not tr and other.marker_id == best:
                    if other.id_votes.get(best, 0) >= tr.id_votes[best]:
                        return
                    other.marker_id = None
            tr.marker_id = best
