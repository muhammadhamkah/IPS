"""One simulated camera station: frame timing, projection with lens distortion,
marker radiometry, centroid noise, occlusion, merging and spurious blobs."""
from __future__ import annotations

import math

import numpy as np

from idps.protocol import packets as P
from idps.sim.scenario import CameraModel, MarkerModel, TimingModel


def distort(xn: np.ndarray, yn: np.ndarray, k):
    """Brown-Conrady on normalised coordinates."""
    k1, k2, p1, p2, k3 = k
    r2 = xn * xn + yn * yn
    radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
    xd = xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn)
    yd = yn * radial + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn
    return xd, yd


def segments_hit_boxes(p0, X: np.ndarray, boxes) -> np.ndarray:
    """Segments p0 -> X[i] crossing any axis-aligned box (lo, hi) (slab test). (M,) bool."""
    X = np.atleast_2d(np.asarray(X, float))
    hit = np.zeros(len(X), bool)
    if not boxes:
        return hit
    p0 = np.asarray(p0, float)
    d = X - p0
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = np.where(np.abs(d) > 1e-12, 1.0 / d, np.inf)
        for lo, hi in boxes:
            t1 = (np.asarray(lo, float) - p0) * inv
            t2 = (np.asarray(hi, float) - p0) * inv
            # a zero direction component: inside the slab means (-inf, inf), outside means empty
            par = np.abs(d) <= 1e-12
            inside = (p0 >= np.asarray(lo)) & (p0 <= np.asarray(hi))
            t1 = np.where(par, np.where(inside, -np.inf, np.inf), t1)
            t2 = np.where(par, np.where(inside, np.inf, -np.inf), t2)
            tmin = np.max(np.minimum(t1, t2), axis=1)
            tmax = np.min(np.maximum(t1, t2), axis=1)
            hit |= (tmax >= np.maximum(tmin, 0.0)) & (tmin <= 1.0)
    return hit


class SimStation:
    def __init__(self, station_id: int, pos, R, lens_mm: float, model: CameraModel, timing: TimingModel,
                 markers: MarkerModel, rng: np.random.Generator, duration_s: float):
        self.id = station_id
        self.pos = np.asarray(pos, float)
        self.R = np.asarray(R, float)
        self.m, self.timing, self.mk = model, timing, markers
        self.rng = rng
        s = model.sensor
        self.W, self.H = s.width_px, s.height_px
        self.f = s.f_px(lens_mm)
        self.cx, self.cy = (self.W - 1) / 2, (self.H - 1) / 2
        self.exposure_s = model.exposure_us * 1e-6

        # Frame timing in true time.
        if timing.sync == "hw_trigger":
            self.period = 1.0 / model.fps
            self.phase = 0.0
            self.sync_state = P.SYNC_HW_TRIGGER
        else:
            self.period = 1.0 / (model.fps * (1 + rng.normal(0, model.sensor_clock_ppm) * 1e-6))
            self.phase = rng.uniform(0, self.period)
            self.sync_state = P.SYNC_PTP if timing.sync == "ptp" else P.SYNC_FREE_RUN
        # Station clock error (what the timestamp carries relative to true time).
        free = timing.sync == "free_run"
        self.clock_offset = rng.normal(0, (timing.offset_sigma_us if not free else 2000.0) * 1e-6)
        self.clock_drift = rng.normal(0, timing.drift_ppm * 1e-6) if not free else rng.normal(0, 30e-6)
        self.n_frames = int(math.floor((duration_s - self.phase) / self.period)) + 1
        self.reflections = [(rng.uniform(5, self.W - 5), rng.uniform(5, self.H - 5), rng.uniform(80, 400))
                            for _ in range(model.static_reflections)]
        self.dropped = 0
        self.occluders: list = []         # scenery boxes (lo, hi) blocking lines of sight

    # ------------------------------------------------------------------
    def frame_time(self, k: int) -> float:
        t = self.phase + k * self.period
        if self.timing.sync == "hw_trigger":
            t += self.rng.normal(0, self.timing.trigger_jitter_us * 1e-6)
        return t

    def timestamp_ns(self, t_true: float, epoch_ns: int) -> int:
        t = t_true + self.clock_offset + self.clock_drift * t_true + self.rng.normal(0, self.timing.jitter_sigma_us * 1e-6)
        return epoch_ns + int(round(t * 1e9))

    def project(self, X: np.ndarray):
        Xc = (X - self.pos) @ self.R.T
        z = Xc[:, 2]
        zs = np.where(z > 1e-6, z, 1.0)
        xd, yd = distort(Xc[:, 0] / zs, Xc[:, 1] / zs, self.m.distortion)
        return self.cx + self.f * xd, self.cy + self.f * yd, z

    # ------------------------------------------------------------------
    def observe(self, t: float, marker_pos: np.ndarray, body_up: np.ndarray, levels: np.ndarray,
                drone_centres: np.ndarray, marker_drone: np.ndarray):
        """Markers seen in one frame. marker_pos (M, 3), body_up (M, 3), levels (M,) mean drive level
        over the exposure, drone_centres (D, 3), marker_drone (M,) index of each marker's drone.
        Returns (list[P.Marker], truth) where truth is a list of (marker index or -1, x, y)."""
        m, mk, rng = self.m, self.mk, self.rng
        M = len(marker_pos)
        out, truth = [], []
        if M:
            u, v, z = self.project(marker_pos)
            d = self.pos - marker_pos
            rng_m = np.linalg.norm(d, axis=1)
            dirn = d / rng_m[:, None]
            cosu = np.einsum("ij,ij->i", dirn, body_up)
            ok = (z > 0.3) & (u >= 0) & (u <= self.W - 1) & (v >= 0) & (v <= self.H - 1)
            ok &= cosu > mk.self_occlusion_cos
            ok &= ~self._occluded(marker_pos, drone_centres, marker_drone)
            ok &= ~segments_hit_boxes(self.pos, marker_pos, self.occluders)
            emission = mk.emission_floor + (1 - mk.emission_floor) * np.maximum(0.0, cosu)
            peak_true = (m.peak_at_10m * levels * emission * (10.0 / np.maximum(rng_m, 0.1)) ** 2
                         * (m.exposure_us / m.exposure_ref_us))
            ok &= peak_true >= m.threshold_dn
            idx = np.nonzero(ok)[0]
            blobs = []
            two_pi_s2 = 2 * math.pi * m.psf_sigma_px ** 2
            for i in idx:
                pk = peak_true[i]
                sat = pk > m.saturation_dn
                total = pk * two_pi_s2
                if sat:
                    total *= (m.saturation_dn / pk) ** 0.5       # clipped core: sub-linear growth
                total = max(1.0, total + rng.normal(0, math.sqrt(total)))
                sigma = math.sqrt(m.centroid_floor_px ** 2 + (m.shot_noise_k / math.sqrt(total)) ** 2
                                  + (0.08 ** 2 if sat else 0.0))
                x, y = u[i] + rng.normal(0, sigma), v[i] + rng.normal(0, sigma)
                if m.pixel_locking and m.psf_sigma_px < 1.0:
                    amp = 0.25 * math.exp(-2.0 * m.psf_sigma_px ** 2)
                    x -= amp * math.sin(2 * math.pi * (x - round(x)))
                    y -= amp * math.sin(2 * math.pi * (y - round(y)))
                area = min(255, int(math.pi * (2 * m.psf_sigma_px) ** 2 * max(1.0, math.log(max(pk, 1) / m.threshold_dn + 1))))
                blobs.append([x, y, total, min(pk, m.saturation_dn), area, P.MFLAG_SATURATED if sat else 0, int(i)])
            blobs = self._merge(blobs)
            for b in blobs:
                x, y = b[0], b[1]
                fl = b[5] | (P.MFLAG_EDGE if (x < 3 or y < 3 or x > self.W - 4 or y > self.H - 4) else 0)
                out.append(P.Marker(float(x), float(y), int(b[2]), int(b[3]), int(b[4]), fl))
                truth.append((b[6], float(u[b[6]]) if b[6] >= 0 else x, float(v[b[6]]) if b[6] >= 0 else y))
        # spurious blobs: fixed reflections + random false positives
        for rx, ry, pk in self.reflections:
            out.append(P.Marker(rx + rng.normal(0, 0.02), ry + rng.normal(0, 0.02), int(pk * 9), int(pk), 9, 0))
            truth.append((-1, rx, ry))
        for _ in range(rng.poisson(m.false_positive_rate)):
            fx, fy, pk = rng.uniform(0, self.W - 1), rng.uniform(0, self.H - 1), rng.uniform(m.threshold_dn, 80)
            out.append(P.Marker(fx, fy, int(pk * 4), int(pk), 2, 0))
            truth.append((-1, fx, fy))
        return out, truth

    def _occluded(self, X, centres, marker_drone):
        """Ray camera->marker passes within body_radius of another drone's centre."""
        if len(centres) < 2:
            return np.zeros(len(X), bool)
        r = self.mk.body_radius_m
        seg = X - self.pos                                         # (M, 3)
        L2 = np.einsum("ij,ij->i", seg, seg)
        rel = centres[None, :, :] - self.pos                       # (1, D, 3)
        t = np.clip(np.einsum("mk,mdk->md", seg, np.broadcast_to(rel, (len(X),) + rel.shape[1:])) / L2[:, None], 0, 1)
        closest = self.pos + t[..., None] * seg[:, None, :]
        dist = np.linalg.norm(closest - centres[None], axis=-1)    # (M, D)
        dist[np.arange(len(X)), marker_drone] = np.inf             # own body handled by self_occlusion_cos
        # ignore hits right at the marker end (the marker sits on top of its own drone)
        return np.any((dist < r) & (t < 0.995), axis=1)

    def _merge(self, blobs):
        """Blobs closer than ~2.5 PSF widths merge into one elongated blob."""
        if len(blobs) < 2:
            return blobs
        lim = 2.5 * 2 * self.m.psf_sigma_px
        blobs = sorted(blobs, key=lambda b: -b[2])
        merged = []
        for b in blobs:
            for mb in merged:
                if math.hypot(b[0] - mb[0], b[1] - mb[1]) < lim:
                    w = mb[2] + b[2]
                    mb[0] = (mb[0] * mb[2] + b[0] * b[2]) / w
                    mb[1] = (mb[1] * mb[2] + b[1] * b[2]) / w
                    mb[2] = w
                    mb[3] = max(mb[3], b[3])
                    mb[4] = min(255, mb[4] + b[4])
                    mb[5] |= b[5] | P.MFLAG_ELONGATED
                    break
            else:
                merged.append(list(b))
        return merged
