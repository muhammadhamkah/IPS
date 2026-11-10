"""Where every camera stands and points, from one LED moved through the room (``idps-calib room``).

1. **Start.** The pair of cameras that share the most LED positions gives a first relative pose
   (essential matrix, RANSAC). The LED positions they share are triangulated; every other camera
   is then placed against those points (PnP), the one with the most points first.
2. **Bundle adjustment.** All camera poses and all LED positions are refined together so the
   LED's projections fit the dots (sparse least squares, robust loss; observations that still
   miss by several pixels are dropped and it runs again). Optionally each lens's focal length and
   first two distortion terms are refined too.
3. **World frame.** The reconstruction has no scale, origin or direction of its own. They come
   from the rest points (the LED left still at measured places: three or more, not in a line),
   else from the cameras' planned positions.

Needs SciPy and OpenCV (pip install 'idps[calib]')."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from idps.calib.lens import Lens
from idps.calib.tracks import Observations
from idps.engine.calib import CameraCal


@dataclass
class RoomResult:
    cams: dict                  # station id -> CameraCal (world frame)
    lenses: dict                # station id -> Lens (refined if asked)
    points: np.ndarray          # (P, 3) LED positions, NaN where not reconstructed
    rms_px: dict                # station id -> residual of its dots (RMS, pixels)
    n_obs: dict                 # station id -> dots used
    dropped: dict               # station id -> dots dropped as outliers
    frame: str                  # "rest points" | "planned camera positions" | "none"
    ref_residual_mm: list = field(default_factory=list)   # rest points: distance from where they should be
    failed: list = field(default_factory=list)            # stations that could not be placed
    notes: list = field(default_factory=list)

    @property
    def rms_all_px(self) -> float:
        n = sum(self.n_obs.values())
        return float(np.sqrt(sum(self.rms_px[s] ** 2 * self.n_obs[s] for s in self.rms_px) / max(n, 1)))


def _cal(lens: Lens, sid: int = 0) -> CameraCal:
    return CameraCal(sid, np.zeros(3), np.eye(3), lens.f_px, lens.cx, lens.cy, np.asarray(lens.distortion, float),
                     lens.width, lens.height)


def _project(Xc, f, cx, cy, k):
    """Camera-frame points (M, 3) -> distorted pixels (M, 2); f, cx, cy (M,), k (M, 5)."""
    z = np.where(Xc[:, 2] > 1e-6, Xc[:, 2], 1e-6)
    xn, yn = Xc[:, 0] / z, Xc[:, 1] / z
    r2 = xn * xn + yn * yn
    radial = 1 + k[:, 0] * r2 + k[:, 1] * r2 * r2 + k[:, 4] * r2 ** 3
    xd = xn * radial + 2 * k[:, 2] * xn * yn + k[:, 3] * (r2 + 2 * xn * xn)
    yd = yn * radial + k[:, 2] * (r2 + 2 * yn * yn) + 2 * k[:, 3] * xn * yn
    return np.stack([cx + f * xd, cy + f * yd], axis=1)


def _triangulate(R, t, xn, min_cams: int = 2):
    """One point from its normalised image coordinates in several cameras (x_cam = R X + t).
    R (k, 3, 3), t (k, 3), xn (k, 2). Returns X or None (behind a camera, or rays nearly parallel)."""
    if len(R) < min_cams:
        return None
    A = np.concatenate([xn[:, 0:1] * R[:, 2] - R[:, 0], xn[:, 1:2] * R[:, 2] - R[:, 1]])
    b = -np.concatenate([xn[:, 0] * t[:, 2] - t[:, 0], xn[:, 1] * t[:, 2] - t[:, 1]])
    X, *_ = np.linalg.lstsq(A, b, rcond=None)
    Xc = np.einsum("kij,j->ki", R, X) + t
    if (Xc[:, 2] <= 0.05).any():
        return None
    d = Xc / np.linalg.norm(Xc, axis=1, keepdims=True)
    rays = np.einsum("kji,kj->ki", R, d)
    cosmin = min(float(rays[a] @ rays[b]) for a in range(len(R)) for b in range(a + 1, len(R)))
    if cosmin > np.cos(np.radians(1.0)):                    # all rays within 1 degree: no depth
        return None
    return X


def _similarity(src, dst, with_scale: bool = True):
    """dst ~ s R src + T (least squares, Umeyama)."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    ms, md = src.mean(0), dst.mean(0)
    a, b = src - ms, dst - md
    U, S, Vt = np.linalg.svd(b.T @ a / len(src))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / (a ** 2).sum() * len(src)) if with_scale else 1.0
    return s, R, md - s * R @ ms


def _spread(P) -> float:
    """How far from a line the points are: second singular value of the centred set (0: collinear)."""
    P = np.asarray(P, float)
    if len(P) < 3:
        return 0.0
    return float(np.linalg.svd(P - P.mean(0), compute_uv=False)[1] / np.sqrt(len(P)))


def solve_room(obs: Observations, lenses: dict, refs: list | None = None, planned: dict | None = None,
               refine_lens: bool = False, log=print) -> RoomResult:
    """obs: matched LED observations. lenses: station id -> Lens. refs: world position of each rest
    point, in time order ([x, y, z] or None to skip one). planned: station id -> planned camera
    position (and optionally view direction: a (position, direction) pair), used for the world
    frame when there are no rest points."""
    import cv2
    from scipy.optimize import least_squares
    from scipy.sparse import lil_matrix
    from scipy.spatial.transform import Rotation

    sids = list(obs.stations)
    C, P = len(sids), len(obs.t)
    seen = obs.seen
    cals = [_cal(lenses[s], s) for s in sids]
    xn = np.full((P, C, 2), np.nan)
    for c in range(C):
        m = seen[:, c]
        if m.any():
            xn[m, c, 0], xn[m, c, 1] = cals[c].normalised(obs.xy[m, c, 0], obs.xy[m, c, 1])
    fmean = float(np.mean([l.f_px for l in lenses.values()]))
    notes: list[str] = []

    # ---------------------------------------------------------------- 1. first pair
    pairs = sorted(((int((seen[:, a] & seen[:, b]).sum()), a, b) for a in range(C) for b in range(a + 1, C)),
                   reverse=True)
    pose: dict[int, tuple] = {}
    for n, a, b in pairs:
        if n < 30:
            break
        m = seen[:, a] & seen[:, b]
        pa, pb = xn[m, a], xn[m, b]
        E, mask = cv2.findEssentialMat(pa, pb, focal=1.0, pp=(0.0, 0.0), method=cv2.RANSAC, prob=0.999,
                                       threshold=2.0 / fmean)
        if E is None or E.shape != (3, 3):
            continue
        ngood, R, t, mask2 = cv2.recoverPose(E, pa, pb, focal=1.0, pp=(0.0, 0.0), mask=mask)
        if ngood < 0.6 * n:
            continue
        pose = {a: (np.eye(3), np.zeros(3)), b: (R, t.ravel())}
        first = [a, b]
        # enough parallax between the two? (two cameras side by side looking the same way give none)
        Rs, ts = np.array([np.eye(3), R]), np.array([np.zeros(3), t.ravel()])
        got = [_triangulate(Rs, ts, np.array([x1, x2])) is not None for x1, x2 in zip(pa[::max(1, n // 60)], pb[::max(1, n // 60)])]
        if np.mean(got) > 0.5:
            break
        pose = {}
    if not pose:
        raise ValueError("no two cameras share enough LED positions (30 at least) seen from different angles: "
                         "move the LED through the space the cameras share, slowly, for longer")

    def triangulate_all():
        X = np.full((P, 3), np.nan)
        reg = sorted(pose)
        Rr, tr = np.array([pose[c][0] for c in reg]), np.array([pose[c][1] for c in reg])
        for p in range(P):
            k = seen[p, reg]
            if k.sum() < 2:
                continue
            x = _triangulate(Rr[k], tr[k], xn[p, np.array(reg)[k]])
            if x is None:
                continue
            Xc = np.einsum("kij,j->ki", Rr[k], x) + tr[k]
            err = np.linalg.norm(Xc[:, :2] / Xc[:, 2:3] - xn[p, np.array(reg)[k]], axis=1) * fmean
            if err.max() < 6.0:
                X[p] = x
        return X

    X = triangulate_all()
    # ---------------------------------------------------------------- 1b. the other cameras, most points first
    failed = []
    while len(pose) < C:
        have = ~np.isnan(X[:, 0])
        cand = sorted(((int((have & seen[:, c]).sum()), c) for c in range(C) if c not in pose and c not in failed),
                      reverse=True)
        if not cand:
            break
        n, c = cand[0]
        ok = False
        if n >= 12:
            m = have & seen[:, c]
            ok, rvec, tvec, inl = cv2.solvePnPRansac(X[m].astype(np.float64), xn[m, c].astype(np.float64), np.eye(3), None,
                                                    reprojectionError=4.0 / fmean, iterationsCount=300, confidence=0.999)
            ok = bool(ok) and inl is not None and len(inl) >= max(10, 0.5 * n)
            if ok:                                            # refine on the inliers
                ii = inl.ravel()
                rvec, tvec = cv2.solvePnPRefineLM(X[m][ii].astype(np.float64), xn[m, c][ii].astype(np.float64),
                                                  np.eye(3), None, rvec, tvec)
        if ok:
            pose[c] = (cv2.Rodrigues(rvec)[0], tvec.ravel())
            Xc = X[m] @ pose[c][0].T + pose[c][1]
            e = np.linalg.norm(Xc[:, :2] / Xc[:, 2:3] - xn[m, c], axis=1) * fmean
            log(f"  station {sids[c]}: placed from {n} LED positions ({len(inl)} agree, median miss {np.median(e):.2f} px)")
            X = triangulate_all()
        else:
            failed.append(c)
    reg = first + sorted(c for c in pose if c not in first)   # the first pair leads: its first camera is the
    if failed:                                                # fixed origin, their distance pins the scale
        notes.append("not placed (too few LED positions shared with the others): stations "
                     + ", ".join(str(sids[c]) for c in failed))

    # ---------------------------------------------------------------- 2. bundle adjustment
    ci = {c: k for k, c in enumerate(reg)}
    nc = len(reg)
    f0 = np.array([lenses[sids[c]].f_px for c in reg])
    cx = np.array([lenses[sids[c]].cx for c in reg])
    cy = np.array([lenses[sids[c]].cy for c in reg])
    k0 = np.array([np.asarray(lenses[sids[c]].distortion, float) for c in reg])
    base = float(np.linalg.norm(pose[reg[1]][1])) if nc > 1 else 1.0

    def run(X, use):
        """use (P, C) bool: the observations in this pass. Returns poses, points, residuals."""
        pts = np.nonzero(~np.isnan(X[:, 0]) & (use[:, reg].sum(1) >= 2))[0]
        pi = {p: k for k, p in enumerate(pts)}
        oc, op, uv = [], [], []
        for p in pts:
            for c in reg:
                if use[p, c]:
                    oc.append(ci[c]); op.append(pi[p]); uv.append(obs.xy[p, c])
        oc, op, uv = np.array(oc), np.array(op), np.array(uv)
        M = len(oc)
        ncam = 6 * (nc - 1)
        nl = 3 * nc if refine_lens else 0
        n = ncam + nl + 3 * len(pts)
        x0 = np.zeros(n)
        for c in reg[1:]:
            k = ci[c] - 1
            x0[6 * k:6 * k + 3] = Rotation.from_matrix(pose[c][0]).as_rotvec()
            x0[6 * k + 3:6 * k + 6] = pose[c][1]
        x0[ncam + nl:] = X[pts].ravel()

        def unpack(x):
            Rm = np.concatenate([np.eye(3)[None], Rotation.from_rotvec(x[:ncam].reshape(-1, 6)[:, :3]).as_matrix()]) \
                if nc > 1 else np.eye(3)[None]
            tm = np.concatenate([np.zeros((1, 3)), x[:ncam].reshape(-1, 6)[:, 3:]]) if nc > 1 else np.zeros((1, 3))
            f, k = f0.copy(), k0.copy()
            if refine_lens:
                L = x[ncam:ncam + nl].reshape(-1, 3)
                f = f0 * (1 + L[:, 0])
                k[:, 0] += L[:, 1]
                k[:, 1] += L[:, 2]
            return Rm, tm, f, k, x[ncam + nl:].reshape(-1, 3)

        def fun(x):
            Rm, tm, f, k, Xp = unpack(x)
            Xc = np.einsum("mij,mj->mi", Rm[oc], Xp[op]) + tm[oc]
            r = (_project(Xc, f[oc], cx[oc], cy[oc], k[oc]) - uv).ravel()
            g = 100.0 * (np.linalg.norm(tm[1]) - base) if nc > 1 else 0.0     # the free scale, pinned
            return np.append(r, g)

        S = lil_matrix((2 * M + 1, n), dtype=int)
        rows = np.arange(M)
        for d in range(2):
            for j in range(3):
                S[2 * rows + d, ncam + nl + 3 * op + j] = 1
            mov = oc > 0
            for j in range(6):
                S[2 * rows[mov] + d, 6 * (oc[mov] - 1) + j] = 1
            if refine_lens:
                for j in range(3):
                    S[2 * rows + d, ncam + 3 * oc + j] = 1
        if nc > 1:
            S[2 * M, 0:6] = 1
        sol = least_squares(fun, x0, jac_sparsity=S, method="trf", loss="soft_l1", f_scale=1.0, x_scale="jac",
                            max_nfev=60, ftol=1e-10, xtol=1e-10)
        Rm, tm, f, k, Xp = unpack(sol.x)
        res = np.linalg.norm(sol.fun[:-1].reshape(-1, 2), axis=1)
        return Rm, tm, f, k, pts, Xp, oc, op, res

    def adopt(Rm, tm, f, k, pts, Xp):
        """Take a pass's result as the next one's start (poses, lenses, points)."""
        nonlocal base
        for c in reg:
            pose[c] = (Rm[ci[c]], tm[ci[c]])
        if nc > 1:
            base = float(np.linalg.norm(tm[1]))
        if refine_lens:
            f0[:], k0[:] = f, k
            for c in reg:
                l = lenses[sids[c]]
                cal = _cal(Lens(float(f[ci[c]]), l.cx, l.cy, list(k[ci[c]]), l.width, l.height), sids[c])
                m = seen[:, c]
                xn[m, c, 0], xn[m, c, 1] = cal.normalised(obs.xy[m, c, 0], obs.xy[m, c, 1])
        Xn = np.full((P, 3), np.nan)
        Xn[pts] = Xp
        return Xn

    use = seen.copy()
    use[:, failed] = False
    Rm, tm, f, k, pts, Xp, oc, op, res = run(X, use)
    # with the refined cameras, the LED positions the rough start could not place come back
    Xn = adopt(Rm, tm, f, k, pts, Xp)
    Xt = triangulate_all()
    Xn = np.where(np.isnan(Xn), Xt, Xn)
    Rm, tm, f, k, pts, Xp, oc, op, res = run(Xn, use)
    # drop the dots that still miss (a reflection, a second light), then once more
    limit = max(1.5, 4.0 * float(np.median(res)))
    bad = res > limit
    dropped = {s: 0 for s in sids}
    if bad.any():
        for c_k, p_k in zip(oc[bad], op[bad]):
            use[pts[p_k], reg[c_k]] = False
            dropped[sids[reg[c_k]]] += 1
        Rm, tm, f, k, pts, Xp, oc, op, res = run(adopt(Rm, tm, f, k, pts, Xp), use)
    # a camera left with almost nothing was never really placed
    weak = [c for c in reg if (oc == ci[c]).sum() < 12 and c not in first]
    for c in weak:
        notes.append(f"station {sids[c]}: only {int((oc == ci[c]).sum())} of its dots fit; left out")
    failed += weak
    good = [c for c in reg if c not in weak]

    Xall = np.full((P, 3), np.nan)
    Xall[pts] = Xp
    out_lens = {}
    for c in good:
        l = lenses[sids[c]]
        out_lens[sids[c]] = Lens(float(f[ci[c]]), l.cx, l.cy, [float(v) for v in k[ci[c]]], l.width, l.height, sids[c],
                                 l.rms_px, l.images_used, l.coverage,
                                 list(l.notes) + (["focal length, k1 and k2 refined in the room calibration"] if refine_lens else []))
    rms = {sids[c]: float(np.sqrt(np.mean(res[oc == ci[c]] ** 2))) for c in good}
    nobs = {sids[c]: int((oc == ci[c]).sum()) for c in good}
    centres = {c: -Rm[ci[c]].T @ tm[ci[c]] for c in good}

    # ---------------------------------------------------------------- 3. world frame
    s, Rw, Tw, frame, ref_res = 1.0, np.eye(3), np.zeros(3), "none", []
    ref_pairs = []
    if refs:
        if len(refs) != len(obs.rest):
            notes.append(f"{len(obs.rest)} rest points were found in the recording but {len(refs)} are listed: "
                         "they are matched in time order, check the list")
        for (a, _), w in zip(obs.rest, refs):
            if w is not None and not np.isnan(Xall[a, 0]):
                ref_pairs.append((Xall[a], np.asarray(w, float)))
    if len(ref_pairs) >= 3 and _spread([w for _, w in ref_pairs]) > 0.05:
        src, dst = np.array([a for a, _ in ref_pairs]), np.array([b for _, b in ref_pairs])
        s, Rw, Tw = _similarity(src, dst)
        ref_res = [float(v) * 1000 for v in np.linalg.norm(s * src @ Rw.T + Tw - dst, axis=1)]
        frame = "rest points"
    elif planned and sum(sids[c] in planned for c in good) >= 2:
        if refs:
            notes.append("fewer than 3 usable rest points (not in one line): the world frame comes from the "
                         "cameras' planned positions instead")
        cs = [c for c in good if sids[c] in planned]
        pl = {c: planned[sids[c]] for c in cs}
        ppos = {c: np.asarray(v[0] if isinstance(v, tuple) else v, float) for c, v in pl.items()}
        src, dst = np.array([centres[c] for c in cs]), np.array([ppos[c] for c in cs])
        if len(cs) >= 3 and _spread(dst) > 0.05:
            s, Rw, Tw = _similarity(src, dst)
        else:
            # two cameras (or all in a line): scale from their distance, direction from where they look too
            s = float(np.linalg.norm(dst[1] - dst[0]) / np.linalg.norm(src[1] - src[0]))
            a_src, a_dst = [s * p for p in src], list(dst)
            for c in cs:
                if isinstance(pl[c], tuple):
                    d = np.asarray(pl[c][1], float)
                    a_src.append(s * centres[c] + Rm[ci[c]].T @ np.array([0, 0, 1.0]))
                    a_dst.append(ppos[c] + d / np.linalg.norm(d))
            _, Rw, Tw = _similarity(np.array(a_src), np.array(a_dst), with_scale=False)   # (a_src is already scaled)
            notes.append("world frame from two cameras' planned positions and aims: good to the accuracy they "
                         "were measured and aimed with. Use rest points for a better one")
        frame = "planned camera positions"
    else:
        notes.append("no rest points and no planned camera positions: the result has no scale and no world "
                     "frame (the first camera is the origin)")

    cams = {}
    for c in good:
        Rc = Rm[ci[c]] @ Rw.T
        cams[sids[c]] = CameraCal(sids[c], s * Rw @ centres[c] + Tw, Rc, float(f[ci[c]]), float(cx[ci[c]]), float(cy[ci[c]]),
                                  np.asarray(k[ci[c]], float), lenses[sids[c]].width, lenses[sids[c]].height)
    Xw = s * Xall @ Rw.T + Tw
    return RoomResult(cams, out_lens, Xw, rms, nobs, dropped, frame, ref_res, [sids[c] for c in failed], notes)


def to_calibration_doc(res: RoomResult, source: str = "idps-calib room") -> dict:
    """The ``idps-calibration/1`` document the engine loads (idps.engine.calib.load_calibration)."""
    any_cam = next(iter(res.cams.values()))
    doc = {"format": "idps-calibration/1", "source": source, "width_px": int(any_cam.width),
           "height_px": int(any_cam.height),
           "quality": {"world_frame": res.frame, "rms_px": round(res.rms_all_px, 4),
                       "rest_point_error_mm": [round(v, 2) for v in res.ref_residual_mm],
                       "not_placed": list(res.failed), "notes": list(res.notes)},
           "cameras": []}
    for sid, c in sorted(res.cams.items()):
        doc["cameras"].append({
            "station_id": int(sid), "f_px": round(float(c.f_px), 4), "cx": round(float(c.cx), 4),
            "cy": round(float(c.cy), 4), "distortion": [round(float(v), 8) for v in c.distortion],
            "quality": {"rms_px": round(res.rms_px[sid], 4), "dots": res.n_obs[sid], "dots_dropped": res.dropped[sid]},
            "estimate": {"position_m": [round(float(v), 5) for v in c.position],
                         "R_world_to_cam": [[round(float(v), 8) for v in row] for row in c.R]}})
    return doc
