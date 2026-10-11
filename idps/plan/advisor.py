"""Advisor: find the fixes that remove the most problem points.

Given the current camera layout (planned, hand-edited, or later the as-built
layout from calibration) it evaluates three kinds of fix and ranks them by how
many evaluation points stop failing the requirement:

  * re-aim  - same camera, same mount, new pan/tilt (motorised gimbal: one click)
  * move    - same camera to a nearby allowed mount position (+ best aim)
  * add     - one new camera at the best allowed position

Every recommendation is exact geometry from the same Fisher-information model
as the planner, and carries tape-measure install text, so it can be checked and
applied rather than guessed. Live phases will feed measured coverage, poses and
camera health into the same functions.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from idps.core.camera import rotation_from_az_el
from idps.core.noise import P95_PER_RMS
from idps.coverage.fisher import contributions, rms_mm
from idps.plan.candidates import CandidateSet, generate_candidates
from idps.plan.model import PlannedCamera
from idps.plan.optimizer import Objective
from idps.plan.planner import build_cameras, site_grid
from idps.plan.site import Site


SERVO_PAN_LIMIT_DEG = 90.0        # hobby pan servos: about ±90° from the base's pan-zero


@dataclass
class Recommendation:
    kind: str                     # "reaim" | "move" | "add"
    camera_id: str | None         # camera changed (None for add)
    new_camera: PlannedCamera     # the camera after the change (id kept for reaim/move)
    failing_before: int
    failing_after: int
    deficit_gain: float
    title: str
    detail: str

    @property
    def points_fixed(self) -> int:
        return self.failing_before - self.failing_after


@dataclass
class Focus:
    """Restrict the advisor to a problem region seen live (a repeated tracking loss).

    Only evaluation points within `radius_m` of `center` count. `blocked` cameras are
    the ones the model expects to see the region but that the live data shows do not
    (scenery, a banner, rigging): their contribution there is removed, also after a
    re-aim, since turning a camera does not clear its line of sight. `min_cameras`
    defaults to the requirement; the brain raises it by one when the model looks fine
    but tracking still drops (merged blobs, reflections), to buy margin there."""
    center: tuple
    radius_m: float
    blocked: frozenset = frozenset()
    min_cameras: int | None = None
    points: object = None             # (K, 3) where the losses actually happened: evaluated too
    block_at: object = None           # measured line-of-sight blocks at those points:
                                      # {camera id: (position when measured, (K,) bool)}. They hold while the
                                      # camera stays where it was (also after a re-aim), not after a move.


@dataclass
class Health:
    cameras: int
    points: int
    failing: int
    fraction_ok: float
    failing_volume_m3: float
    worst_camera: str | None      # camera whose loss hurts most
    worst_camera_points: int


class Advisor:
    def __init__(self, site: Site, cameras: list[PlannedCamera], move_radius_m: float = 4.0,
                 focus: Focus | None = None, offline: frozenset | set = frozenset()):
        """offline: ids of cameras that are down. They stay in the layout (and keep their
        ids) but contribute nothing, and are never re-aimed or moved."""
        self.site, self.cams = site, list(cameras)
        self.grid = site_grid(site)
        self.pts = self.grid.points
        extra = 0
        if focus is not None and focus.points is not None and len(focus.points):
            extra = len(focus.points)
            self.pts = np.vstack([self.pts, np.asarray(focus.points, float).reshape(-1, 3)])
        req = site.requirements
        self.focus = focus
        self.offline = set(offline)
        # Per-point camera requirement: raised inside the focus region. Points elsewhere keep the
        # plain requirement and still count, so a fix for one region cannot break another.
        self.need = np.full(len(self.pts), req.min_cameras)
        self.mask = None
        if focus is not None:
            d = np.linalg.norm(self.pts - np.asarray(focus.center, float), axis=1)
            r = float(focus.radius_m)
            while (d <= r).sum() < 8 and r < 50:      # make sure the region holds some points
                r *= 1.5
            self.mask = d <= r
            if extra:
                self.mask[-extra:] = True
            if focus.min_cameras is not None:
                self.need[self.mask] = int(focus.min_cameras)
        self.min_cameras = int(self.need.max())
        self.obj = Objective(self.need, req.p95_mm / P95_PER_RMS)
        self.move_radius = move_radius_m
        self._cands: CandidateSet | None = None
        if self.cams:
            pos = np.array([c.position_m for c in self.cams], float)
            R = np.array([c.R for c in self.cams])
            f = np.array([site.sensor.f_px(c.lens_mm) for c in self.cams])
            self.vis, self.inf = self._contrib(pos, R, f)
            for k, c in enumerate(self.cams):
                if c.id in self.offline:
                    self.vis[k], self.inf[k] = False, 0.0
                elif self._is_blocked(c.id):
                    self.vis[k, self.mask], self.inf[k, self.mask] = False, 0.0
            if extra and focus.block_at:
                P0 = len(self.pts) - extra
                for k, c in enumerate(self.cams):
                    B = self._measured_block(c)
                    if B is not None:
                        self.vis[k, P0:][B], self.inf[k, P0:][B] = False, 0.0
        else:
            self.vis = np.zeros((0, len(self.pts)), bool)
            self.inf = np.zeros((0, len(self.pts), 6))
        self.n = self.vis.sum(0)
        self.info = self.inf.sum(0)

    # ------------------------------------------------------------- basics
    def _contrib(self, pos, R, f_px):
        return contributions(np.atleast_2d(pos), R.reshape(-1, 3, 3), f_px, self.pts, self.site.sensor,
                             self.site.requirements.noise, self.site.limits)

    def _measured_block(self, cam: PlannedCamera):
        """(K,) mask of loss points this camera was measured not to see through, or None."""
        if self.focus is None or not self.focus.block_at or cam.id not in self.focus.block_at:
            return None
        pos, B = self.focus.block_at[cam.id]
        if np.linalg.norm(np.asarray(pos, float) - np.asarray(cam.position_m, float)) > 0.01:
            return None                                      # moved since: the evidence no longer applies
        return np.asarray(B, bool)

    def _is_blocked(self, cam_id) -> bool:
        return self.focus is not None and cam_id in self.focus.blocked

    def _failing(self, n, info):
        """Points failing the *requirement* (not the design margin), with the raised camera
        count inside a focus region."""
        req = self.site.requirements
        p95 = rms_mm(info) * P95_PER_RMS
        return ~((n >= self.need) & (p95 <= req.p95_mm))

    def _deficit(self, n, info):
        return self.obj.voxel_deficit(n, info)

    def _evaluate(self, n0, info0, vis, inf):
        """For C options added to base state (n0, info0): failing count and total deficit."""
        n = n0[None, :] + vis
        info = info0[None, :, :] + inf
        fail = self._failing(n, info).sum(axis=1)
        deficit = self._deficit(n, info).sum(axis=1)
        return fail, deficit

    @property
    def candidates(self) -> CandidateSet:
        if self._cands is None:
            self._cands = generate_candidates(self.site)
        return self._cands

    def health(self) -> Health:
        """Whole-layout health; in focus mode, of the focus region only."""
        fail = self._failing(self.n, self.info)
        if self.mask is not None:
            fail = fail & self.mask
        g = self.grid
        vol = self.site.volume.volume_m3
        frac_bad_grid = float(fail[:g.n_grid].mean()) if g.n_grid else float(fail.mean())
        worst, worst_pts = None, 0
        for k, c in enumerate(self.cams):
            if c.id in self.offline:
                continue
            lost = self._failing(self.n - self.vis[k], self.info - self.inf[k])
            if self.mask is not None:
                lost &= self.mask
            extra = int((lost & ~fail).sum())
            if extra > worst_pts:
                worst, worst_pts = c.id, extra
        npts = len(self.pts) if self.mask is None else int(self.mask.sum())
        return Health(len(self.cams) - len(self.offline & {c.id for c in self.cams}), npts, int(fail.sum()),
                      float(1 - fail.sum() / max(npts, 1)), frac_bad_grid * vol, worst, worst_pts)

    # ------------------------------------------------------------- fixes
    def _spacing_ok(self, positions, exclude: int | None, mount_keys=None, own_key=None):
        """Mask of candidate positions at least min spacing away from every other camera."""
        sp = float(self.site.candidates.min_camera_spacing_m)
        ok = np.ones(len(positions), bool)
        if sp <= 0:
            return ok
        for k, c in enumerate(self.cams):
            if k == exclude:
                continue
            ok &= np.linalg.norm(positions - np.array(c.position_m), axis=1) >= sp - 1e-9
        return ok

    def _batch_best(self, n0, info0, idx, batch=256):
        """Best candidate (index into self.candidates) among idx for base state."""
        c = self.candidates
        best = (None, math.inf, math.inf)
        R = c.R
        for s in range(0, len(idx), batch):
            j = idx[s:s + batch]
            vis, inf = self._contrib(c.pos[j], R[j], c.f_px[j])
            fail, deficit = self._evaluate(n0, info0, vis, inf)
            k = int(np.lexsort((deficit, fail))[0])
            if (fail[k], deficit[k]) < best[1:]:
                best = (int(j[k]), int(fail[k]), float(deficit[k]))
        return best

    def recommend(self, top: int = 6, kinds=("reaim", "move", "add"), progress=None) -> list[Recommendation]:
        base_fail = int(self._failing(self.n, self.info).sum())
        base_def = float(self._deficit(self.n, self.info).sum())
        if base_fail == 0:
            return []
        recs: list[Recommendation] = []
        c = self.candidates

        if "reaim" in kinds or "move" in kinds:
            for k, cam in enumerate(self.cams):
                if cam.id in self.offline:
                    continue
                if progress:
                    progress(f"Checking {cam.id} ({k + 1}/{len(self.cams)})")
                n0, info0 = self.n - self.vis[k], self.info - self.inf[k]
                if "reaim" in kinds:
                    rec = self._best_reaim(k, cam, n0, info0, base_fail, base_def)
                    if rec:
                        recs.append(rec)
                if "move" in kinds:
                    same_kind = c.mount_type == cam.mount_type
                    near = np.linalg.norm(c.pos - np.array(cam.position_m), axis=1) <= self.move_radius
                    moved = np.linalg.norm(c.pos - np.array(cam.position_m), axis=1) > 0.05
                    idx = np.where(same_kind & near & moved & self._spacing_ok(c.pos, k))[0]
                    if len(idx):
                        j, fail, deficit = self._batch_best(n0, info0, idx)
                        if j is not None and (fail < base_fail or deficit < base_def - 1e-6):
                            recs.append(self._rec("move", cam, j, base_fail, fail, base_def - deficit))
        if "add" in kinds:
            if progress:
                progress("Searching for the best new camera position")
            idx = np.where(self._spacing_ok(c.pos, None))[0]
            if len(idx):
                j, fail, deficit = self._batch_best(self.n, self.info, idx)
                if j is not None and fail < base_fail:
                    recs.append(self._rec("add", None, j, base_fail, fail, base_def - deficit))
        recs.sort(key=lambda r: (-r.points_fixed, -r.deficit_gain, {"reaim": 0, "move": 1, "add": 2}[r.kind]))
        # keep the best fix per camera, then the best add
        out, seen = [], set()
        for r in recs:
            key = r.camera_id or "add"
            if key in seen or (r.points_fixed <= 0 and r.deficit_gain <= 1e-6):
                continue
            seen.add(key)
            out.append(r)
        return out[:top]

    def _best_reaim(self, k, cam, n0, info0, base_fail, base_def):
        cs = self.site.candidates
        els = cs.truss_elevations_deg if cam.mount_type == "truss" else cs.tripod_elevations_deg
        aims = [(0.0, e) if abs(abs(e) - 90) < 1e-6 else (a, e)
                for e in els for a in np.arange(0.0, 360.0, cs.azimuth_step_deg)]
        aims = sorted(set((round(a, 3), round(e, 3)) for a, e in aims)
                      - {(round(cam.azimuth_deg % 360, 3), round(cam.elevation_deg, 3))})
        if not aims:
            return None
        az = np.array([a for a, _ in aims])
        el = np.array([e for _, e in aims])
        pos = np.repeat(np.array(cam.position_m, float)[None], len(aims), 0)
        vis, inf = self._contrib(pos, rotation_from_az_el(az, el), np.full(len(aims), self.site.sensor.f_px(cam.lens_mm)))
        if self._is_blocked(cam.id):                        # its line of sight there stays blocked
            vis[:, self.mask], inf[:, self.mask] = False, 0.0
        B = self._measured_block(cam)
        if B is not None:
            P0 = len(self.pts) - len(B)
            vis[:, P0:][:, B], inf[:, P0:][:, B] = False, 0.0
        fail, deficit = self._evaluate(n0, info0, vis, inf)
        b = int(np.lexsort((deficit, fail))[0])
        if fail[b] >= base_fail and deficit[b] >= base_def - 1e-6:
            return None
        from idps.plan.model import install_angles
        new = PlannedCamera(**{**cam.__dict__})
        new.azimuth_deg, new.elevation_deg = float(az[b]), float(el[b])
        new.tilt_deg = new.elevation_deg
        # Prefer a pure servo move from the gimbal base as installed; only ask for the base
        # to be turned when the new aim is beyond the servo's pan range.
        pan_from_base = (new.azimuth_deg - cam.base_heading_deg + 180.0) % 360.0 - 180.0
        if abs(new.elevation_deg) > 89.9:
            pan_from_base = cam.pan_deg                      # straight up/down: pan is irrelevant
        title = f"Re-aim {cam.id}"
        if abs(pan_from_base) <= SERVO_PAN_LIMIT_DEG:
            new.base_heading_deg, new.pan_deg = cam.base_heading_deg, pan_from_base
            detail = (f"No physical move (motorised): pan {cam.pan_deg:+.1f}° → {new.pan_deg:+.1f}°, "
                      f"tilt {cam.tilt_deg:+.1f}° → {new.tilt_deg:+.1f}°.")
        else:
            new.base_heading_deg, new.pan_deg, _ = install_angles(new.azimuth_deg, new.elevation_deg)
            detail = (f"Turn the gimbal base from {cam.base_heading_deg:.0f}° to {new.base_heading_deg:.0f}°, then pan "
                      f"{new.pan_deg:+.1f}°, tilt {new.tilt_deg:+.1f}° (beyond the servo's ±{SERVO_PAN_LIMIT_DEG:.0f}° pan).")
        new.install_note = (cam.install_note.split("; gimbal")[0]
                            + f"; gimbal pan-zero towards azimuth {new.base_heading_deg:.0f} deg")
        return Recommendation("reaim", cam.id, new, base_fail, int(fail[b]), float(base_def - deficit[b]), title, detail)

    def _rec(self, kind, cam: PlannedCamera | None, j: int, base_fail, fail, gain) -> Recommendation:
        new = build_cameras(self.site, self.candidates, [j])[0]
        if cam is not None:
            new.id = cam.id
        else:
            used = {c.id for c in self.cams}
            k = len(self.cams) + 1
            while f"C{k:03d}" in used:
                k += 1
            new.id = f"C{k:03d}"
        where = _where(self.site, new)
        if kind == "move":
            title = f"Move {cam.id}"
            if cam.mount_type == "truss" and new.mount_id == cam.mount_id and cam.along_truss_m is not None:
                delta = new.along_truss_m - cam.along_truss_m
                move = (f"Slide it along truss {cam.mount_id} from {cam.along_truss_m:.2f} m to "
                        f"{new.along_truss_m:.2f} m from the start end ({delta:+.2f} m)")
            else:
                d = np.array(new.position_m) - np.array(cam.position_m)
                move = f"Move it {np.linalg.norm(d[:2]):.2f} m (ΔX {d[0]:+.2f}, ΔY {d[1]:+.2f} m) to {where}"
                if abs(d[2]) > 0.01:
                    move += f", camera height {cam.position_m[2]:.2f} → {new.position_m[2]:.2f} m"
            detail = f"{move}; then pan {new.pan_deg:+.1f}°, tilt {new.tilt_deg:+.1f}° (base {new.base_heading_deg:.0f}°)."
        else:
            title = f"Add camera {new.id}"
            detail = (f"Mount it {where}, height {new.position_m[2]:.2f} m; base {new.base_heading_deg:.0f}°, "
                      f"pan {new.pan_deg:+.1f}°, tilt {new.tilt_deg:+.1f}°, {new.lens_mm:g} mm lens.")
        return Recommendation(kind, cam.id if cam else None, new, base_fail, fail, gain, title, detail)


def _where(site: Site, cam: PlannedCamera) -> str:
    """Tape-measure location of one camera, using the same references as the install sheet."""
    from idps.plan.measure import chains_for_site
    chs = chains_for_site(site, [cam])
    if not chs:
        return f"at X {cam.position_m[0]:.2f}, Y {cam.position_m[1]:.2f}"
    ch = chs[0]
    s, _, off = ch.stations[0]
    if ch.kind == "truss":
        return f"on truss {ch.ref}, {s:.2f} m from its start end"
    if s < 0:
        along = f"{-s:.2f} m before {ch.origin_label} on the line of edge {ch.ref}"
    elif s > ch.length:
        along = f"{s - ch.length:.2f} m beyond {ch.end_label} on the line of edge {ch.ref}"
    else:
        along = f"{s:.2f} m along edge {ch.ref} from {ch.origin_label}"
    return f"on a tripod {along}, {off:.2f} m out from the edge"


def auto_fix(site: Site, cameras: list[PlannedCamera], max_steps: int = 12, kinds=("reaim", "move", "add"),
             progress=None, cancel=None, focus: Focus | None = None, offline=frozenset()):
    """Apply the best fix, re-check, repeat. Returns (ordered list of fixes, final cameras).
    Stops when every point meets the requirement, nothing helps, or max_steps is reached."""
    steps: list[Recommendation] = []
    cams = list(cameras)
    for step in range(max_steps):
        if cancel is not None and cancel():
            break
        adv = Advisor(site, cams, focus=focus, offline=offline)
        recs = adv.recommend(top=1, kinds=kinds,
                             progress=(lambda m, s=step: progress(f"Step {s + 1}: {m}")) if progress else None)
        if not recs:
            break
        steps.append(recs[0])
        cams = apply(cams, recs[0])
        if focus is not None and recs[0].kind == "move" and recs[0].camera_id in focus.blocked:
            # moved away from whatever blocked it: judge its new view on the model
            focus = replace(focus, blocked=focus.blocked - {recs[0].camera_id})
        if recs[0].failing_after == 0:
            break
    return steps, cams


def apply(cameras: list[PlannedCamera], rec: Recommendation) -> list[PlannedCamera]:
    """Return a new camera list with the recommendation applied."""
    out = [c for c in cameras if c.id != rec.camera_id] if rec.camera_id else list(cameras)
    out.append(rec.new_camera)
    order = {c.id: i for i, c in enumerate(cameras)}
    out.sort(key=lambda c: (order.get(c.id, len(order)), c.id))
    return out
