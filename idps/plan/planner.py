"""Turn optimiser output into an install plan, and evaluate any plan."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from idps.core.noise import P95_PER_RMS
from idps.coverage.fisher import contributions, guaranteed_visible, rms_mm
from idps.coverage.volume import EvalGrid, distance_to_polygon, volume_grid
from idps.plan.candidates import CandidateSet, generate_candidates
from idps.plan.model import Plan, PlannedCamera, install_angles
from idps.plan.optimizer import Optimiser
from idps.plan.site import Site


@dataclass
class Evaluation:
    grid: EvalGrid
    n_cams: np.ndarray        # (V,)
    p95_mm: np.ndarray        # (V,) predicted p95 3D error; inf if < 2 cameras
    meets: np.ndarray         # (V,) bool
    critical: np.ndarray      # (C,) points that would fail without each camera
    robust: np.ndarray        # (V,) bool: still meets requirements if any one camera is lost
    summary: dict

    @property
    def points(self) -> np.ndarray:
        return self.grid.points


def site_grid(site: Site) -> EvalGrid:
    return volume_grid(site.volume, site.eval_spacing_m, site.eval_inset_m)


def evaluate(site: Site, cameras: list[PlannedCamera]) -> Evaluation:
    grid = site_grid(site)
    pts = grid.points
    req = site.requirements
    V = len(pts)
    if cameras:
        pos = np.array([c.position_m for c in cameras])
        R = np.array([c.R for c in cameras])
        f_px = np.array([site.sensor.f_px(c.lens_mm) for c in cameras])
        vis, inf = contributions(pos, R, f_px, pts, site.sensor, req.noise, site.limits)
    else:
        vis, inf = np.zeros((0, V), bool), np.zeros((0, V, 6))
    n = vis.sum(0)
    info = inf.sum(0)
    p95 = rms_mm(info) * P95_PER_RMS
    meets = (n >= req.min_cameras) & (p95 <= req.p95_mm)

    critical = np.zeros(len(cameras), int)
    robust = meets.copy()
    for k in range(len(cameras)):
        p95_k = rms_mm(info - inf[k]) * P95_PER_RMS
        meets_k = ((n - vis[k]) >= req.min_cameras) & (p95_k <= req.p95_mm)
        critical[k] = int((meets & ~meets_k).sum())
        robust &= meets_k

    vol = site.volume.volume_m3
    g = slice(0, grid.n_grid)     # volume estimates use the regular grid only
    finite = p95[np.isfinite(p95)]
    by_count = {str(k): float(np.mean(np.minimum(n, 3) == k)) for k in range(3)}
    by_count["3+"] = float(np.mean(n >= 3))
    summary = {
        "cameras": len(cameras),
        "cameras_on_trusses": sum(c.mount_type == "truss" for c in cameras),
        "cameras_on_tripods": sum(c.mount_type == "tripod" for c in cameras),
        "tripods": len({tuple(np.round(c.position_m[:2], 2)) for c in cameras if c.mount_type == "tripod"}),
        "evaluation_points": int(V),
        "volume_m3": vol,
        "fraction_meeting_all_requirements": float(meets.mean()),
        "fraction_meeting_with_any_one_camera_lost": float(robust.mean()),
        "fraction_with_min_cameras": float(np.mean(n >= req.min_cameras)),
        "fraction_by_camera_count": by_count,
        "volume_below_min_cameras_m3": float(np.mean(n[g] < req.min_cameras) * vol) if grid.n_grid else 0.0,
        "volume_over_accuracy_limit_m3": float(np.mean(~(p95[g] <= req.p95_mm)) * vol) if grid.n_grid else 0.0,
        "p95_error_mm_median": float(np.median(finite)) if finite.size else None,
        "p95_error_mm_worst": float(p95.max()) if np.isfinite(p95).all() else None,
        "requirements": {"min_cameras": req.min_cameras, "p95_mm": req.p95_mm,
                         "noise": req.noise.to_dict()},
        "design_margin": {"min_cameras": site.design.min_cameras, "p95_mm": site.design.p95_mm},
    }
    return Evaluation(grid, n, p95, meets, critical, robust, summary)


def fine_check(site: Site, cameras: list[PlannedCamera], max_points: int = 400_000, finest_m: float = 0.25,
               cancel=None) -> dict:
    """Re-check a finished layout on a much denser grid than the optimiser uses.

    The planner works on a coarse grid (metres) because coverage only changes where a point
    crosses the edge of a camera's view; this pass looks between those samples for holes the
    coarse grid could have missed. The spacing is as fine as `finest_m` allows within
    `max_points`. Returns a summary dict (also stored in the plan summary as "fine_check")."""
    req = site.requirements
    vol = site.volume.volume_m3
    spacing = max(finest_m, (vol / max_points) ** (1 / 3))
    spacing = float(np.ceil(spacing * 20) / 20)                      # round up to 5 cm
    grid = volume_grid(site.volume, spacing, site.eval_inset_m)
    pts = grid.points
    out = {"spacing_m": spacing, "points": int(len(pts)), "failing": 0, "fraction_meeting": 1.0,
           "failing_volume_m3": 0.0, "worst_spots": [], "unproven": 0, "guaranteed": False,
           "guarantee_radius_m": 0.0}
    if not cameras or not len(pts):
        return out
    pos = np.array([c.position_m for c in cameras])
    R = np.array([c.R for c in cameras])
    f_px = np.array([site.sensor.f_px(c.lens_mm) for c in cameras])
    fail = np.zeros(len(pts), bool)
    unproven = np.zeros(len(pts), bool)
    # every point of the volume lies within this distance of a sample (half the cell diagonal)
    radius = spacing * np.sqrt(3) / 2
    chunk = max(1000, 3_000_000 // max(len(cameras), 1))
    for a in range(0, len(pts), chunk):
        if cancel is not None and cancel():
            break
        P = pts[a:a + chunk]
        vis, inf = contributions(pos, R, f_px, P, site.sensor, req.noise, site.limits)
        n = vis.sum(0)
        p95 = rms_mm(inf.sum(0)) * P95_PER_RMS
        fail[a:a + chunk] = ~((n >= req.min_cameras) & (p95 <= req.p95_mm))
        sure = guaranteed_visible(pos, R, f_px, P, site.sensor, site.limits, radius).sum(0)
        unproven[a:a + chunk] = ~fail[a:a + chunk] & (sure < req.min_cameras)
    g = slice(0, grid.n_grid)
    out["guarantee_radius_m"] = float(radius)
    out["unproven"] = int(unproven.sum())        # meets at the sample, but a camera's view edge is within reach
    out["guaranteed"] = bool(not fail.any() and not unproven.any())
    out["failing"] = int(fail.sum())
    out["fraction_meeting"] = float(1 - fail.mean())
    out["failing_volume_m3"] = float(fail[g].mean() * vol) if grid.n_grid else 0.0
    if fail.any():                                   # where: group failing points into ~2 m cells
        P = pts[fail]
        cells: dict = {}
        for p in P:
            cells.setdefault(tuple(np.floor(p / 2.0).astype(int)), []).append(p)
        spots = sorted(cells.values(), key=len, reverse=True)[:5]
        out["worst_spots"] = [{"x": float(np.mean(c, 0)[0]), "y": float(np.mean(c, 0)[1]),
                               "z_min": float(np.min(c, 0)[2]), "z_max": float(np.max(c, 0)[2]),
                               "points": len(c)} for c in spots]
    return out


def fine_check_text(fc: dict) -> str:
    sp = fc["spacing_m"]
    head = f"Fine check every {sp * 100:.0f} cm ({fc['points']:,} points):"
    if not fc["failing"]:
        if fc.get("guaranteed"):
            return (f"{head} every point meets the requirement, and the camera count is guaranteed for the "
                    f"whole space between them, at any resolution (each sample's {fc['guarantee_radius_m'] * 100:.0f} cm "
                    f"surroundings lie fully inside enough cameras' views).")
        return (f"{head} every point meets the requirement. At {fc.get('unproven', 0):,} of them a camera's view "
                f"edge passes within {fc.get('guarantee_radius_m', 0) * 100:.0f} cm, so the space right there is "
                f"not proven; the coverage tiles still count it as covered.")
    where = "; ".join(f"X {s['x']:.1f}, Y {s['y']:.1f}, {s['z_min']:.1f}–{s['z_max']:.1f} m high"
                      for s in fc["worst_spots"][:3])
    return (f"{head} {fc['failing']:,} points ({100 * (1 - fc['fraction_meeting']):.2f} %, about "
            f"{fc['failing_volume_m3']:,.1f} m³) fall short between the planning grid's samples, mostly near {where}.")


def _outside_note(site: Site, x: float, y: float) -> str:
    """Installer-friendly location relative to the drawn volume outline."""
    d, e, along = distance_to_polygon(np.array([[x, y]]), site.volume.polygon)
    k = int(e[0])
    k2 = (k + 1) % len(site.volume.polygon)
    side = "inside" if site.volume.contains_xy(np.array([[x, y]]))[0] else "outside"
    return (f"{d[0]:.2f} m {side} edge E{k} (V{k}->V{k2}), {along[0]:.2f} m along it from V{k}")


def build_cameras(site: Site, cands: CandidateSet, selected: list[int]) -> list[PlannedCamera]:
    trusses = {t.id: t for t in site.trusses}
    cams = []
    for i in selected:
        base, pan, tilt = install_angles(float(cands.az[i]), float(cands.el[i]))
        x, y, z = (float(v) for v in cands.pos[i])
        if cands.mount_type[i] == "truss":
            t = trusses[cands.mount_id[i]]
            s = float(cands.along_m[i])
            note = (f"Truss {t.id}: {s:.2f} m from its start end ({t.start[0]:.1f}, {t.start[1]:.1f}),"
                    f" camera hung {site.candidates.truss_drop_m:.2f} m below the truss")
            along = s
        else:
            note = (f"Tripod (zone {cands.mount_id[i]}) at X {x:.2f}, Y {y:.2f};"
                    f" {_outside_note(site, x, y)}; camera at {z:.2f} m on the mast")
            along = None
        note += f"; gimbal pan-zero towards azimuth {base:.0f} deg"
        cams.append(PlannedCamera(
            id="", mount_type=str(cands.mount_type[i]), mount_id=str(cands.mount_id[i]),
            position_m=[x, y, z], azimuth_deg=float(cands.az[i]), elevation_deg=float(cands.el[i]),
            lens_mm=float(cands.lens_mm[i]), along_truss_m=along, base_heading_deg=base,
            pan_deg=pan, tilt_deg=tilt, install_note=note))

    def sort_key(c):
        if c.mount_type == "truss":
            return (0, c.mount_id, c.along_truss_m, c.azimuth_deg, c.elevation_deg)
        return (1, c.mount_id, c.position_m[0], c.position_m[1], c.position_m[2], c.azimuth_deg)

    cams.sort(key=sort_key)
    for k, c in enumerate(cams, 1):
        c.id = f"C{k:03d}"
    _neighbour_spacing(cams)
    _mast_heights(cams)
    return cams


def _mast_heights(cams: list[PlannedCamera]) -> None:
    """Tell installers how tall each tripod mast must be when it carries
    several cameras."""
    masts: dict = {}
    for c in cams:
        if c.mount_type == "tripod":
            masts.setdefault(tuple(np.round(c.position_m[:2], 2)), []).append(c)
    for group in masts.values():
        top = max(c.position_m[2] for c in group)
        ids = ", ".join(c.id for c in group)
        for c in group:
            c.install_note += f"; mast height {top:.2f} m" + (f" (shared by {ids})" if len(group) > 1 else "")


def _neighbour_spacing(cams: list[PlannedCamera]) -> None:
    """Distance from each camera's mount to the nearest *other* mount of the
    same kind (same truss for truss mounts; any tripod for tripods)."""
    for c in cams:
        if c.mount_type == "truss":
            others = [o.along_truss_m for o in cams if o.mount_type == "truss" and o.mount_id == c.mount_id
                      and abs(o.along_truss_m - c.along_truss_m) > 1e-6]
            c.neighbour_spacing_m = min((abs(o - c.along_truss_m) for o in others), default=None)
        else:
            p = np.array(c.position_m[:2])
            others = [np.array(o.position_m[:2]) for o in cams if o.mount_type == "tripod"
                      and np.linalg.norm(np.array(o.position_m[:2]) - p) > 1e-6]
            c.neighbour_spacing_m = (float(min(np.linalg.norm(o - p) for o in others))
                                     if others else None)


def optimise(site: Site, log=print, max_cameras: int | None = None, cancel=None):
    for w in site.warnings():
        log(f"warning: {w}")
    cands = generate_candidates(site)
    pts = site_grid(site).points
    result = Optimiser(site, cands, pts, log=log).run(max_cameras=max_cameras, cancel=cancel)
    cams = build_cameras(site, cands, result.selected)
    ev = evaluate(site, cams)
    summary = dict(ev.summary)
    summary["optimiser"] = {"candidates": len(cands), "seconds": round(result.seconds, 1),
                            "stopped_because": result.stopped_because}
    log("Fine check of the finished layout…")
    summary["fine_check"] = ev.summary["fine_check"] = fine_check(site, cams, cancel=cancel)
    log(fine_check_text(summary["fine_check"]))
    plan = Plan(site.name, site.source_path, cams, summary)
    return plan, ev
