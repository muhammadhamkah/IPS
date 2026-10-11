"""Check a show against the venue, the camera layout and the drones' limits.

A show that looks fine in Blender can still fly a drone outside the performance area, through
a place the cameras cannot see well enough, faster than the drone can follow, or too close to
another drone. Every check below uses the venue's NED frame and the same accuracy model as the
planner, so "inside coverage" here means exactly what the planner proved.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from idps.core.noise import P95_PER_RMS
from idps.coverage.fisher import contributions, rms_mm
from idps.coverage.volume import distance_to_polygon
from idps.show.model import Show
from idps.show.placement import Placement


@dataclass
class Limits:
    """Drone and spacing limits (indoor defaults; set them for your airframe)."""
    max_speed_h_mps: float = 3.0        # horizontal
    max_speed_up_mps: float = 2.0
    max_speed_down_mps: float = 1.5
    max_accel_mps2: float = 4.0
    min_separation_m: float = 1.0       # between any two drones, centre to centre
    min_home_spacing_m: float = 1.0     # take-off / landing spots
    sample_dt_s: float = 0.05


@dataclass
class Finding:
    kind: str                            # outside | margin | coverage | speed | accel | separation | home
    severity: str                        # "error" | "warning"
    drones: tuple
    t: float                             # first time it happens (s)
    t_last: float
    ned: tuple
    message: str
    count: int = 1


@dataclass
class ShowReport:
    drones: int
    duration_s: float
    findings: list[Finding] = field(default_factory=list)
    stats: dict = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not any(f.severity == "error" for f in self.findings)

    def summary(self) -> str:
        e = sum(f.severity == "error" for f in self.findings)
        w = sum(f.severity == "warning" for f in self.findings)
        s = self.stats
        head = ("✅ Show OK" if self.ok else "❌ Show has problems") + \
            f": {self.drones} drones, {self.duration_s:.1f} s; {e} error(s), {w} warning(s)."
        if s:
            head += (f" Max speed {s['max_speed_mps']:.2f} m/s, max acceleration {s['max_accel_mps2']:.2f} m/s², "
                     f"closest approach {s['min_separation_m']:.2f} m")
            cf = s.get("covered_fraction")
            head += (f", {cf * 100:.1f} % of flight time with the required cameras and accuracy."
                     if cf == cf else " (no camera plan given: coverage not checked).")
        return head


def _merge(findings: list[Finding], kind, severity, drones, t, ned, message, gap=1.0):
    """Collapse repeats of the same problem for the same drones into one finding."""
    for f in reversed(findings):
        if f.kind == kind and f.drones == drones and t - f.t_last <= gap:
            f.t_last, f.count = t, f.count + 1
            return
    findings.append(Finding(kind, severity, drones, t, t, tuple(round(float(x), 2) for x in ned), message))


def check_show(show: Show, site, cameras=None, placement: Placement | None = None,
               limits: Limits | None = None, coverage_dt_s: float = 0.25) -> ShowReport:
    """site: idps.plan.site.Site (venue, origin, requirements). cameras: planned cameras (optional)."""
    pl = placement or Placement()
    lim = limits or Limits()
    f = site.frame
    vol = site.volume
    rep = ShowReport(len(show.drones), show.duration)
    if not show.drones:
        rep.findings.append(Finding("empty", "error", (), 0, 0, (0, 0, 0), "The show has no drones."))
        return rep
    t0, t1 = show.t_start, show.duration
    t = np.arange(t0, t1 + 1e-9, lim.sample_dt_s)
    names = [d.name for d in show.drones]
    P = np.stack([pl.to_ned(d.sample(t)) for d in show.drones])               # (N, T, 3) NED
    V = np.stack([pl.vec_to_ned(d.sample(t, 1)) for d in show.drones])
    A = np.stack([pl.vec_to_ned(d.sample(t, 2)) for d in show.drones])
    active = np.stack([(t >= d.t_start) & (t <= d.t_end) for d in show.drones])
    xyz = f.from_ned(P.reshape(-1, 3)).reshape(P.shape)                       # drawing frame, z = height
    req = site.requirements
    margin = site.eval_inset_m

    # --- inside the performance area (and so below every camera, D17)
    for i, name in enumerate(names):
        xy = xyz[i, :, :2]
        h = xyz[i, :, 2]
        inside = vol.contains_xy(xy) & (h >= vol.z_min - 1e-6) & (h <= vol.z_max + 1e-6)
        d_wall, _, _ = distance_to_polygon(xy, vol.polygon)
        near = inside & ((d_wall < margin) | (h > vol.z_max - margin))
        airborne = active[i] & (h > 0.3)
        for k in np.nonzero(airborne & ~inside)[0]:
            if h[k] > vol.z_max:
                msg = (f"{name} climbs to {h[k]:.2f} m, above the {vol.z_max:g} m ceiling: higher than the "
                       f"cameras (D17), where they cannot see its marker.")
            else:
                msg = (f"{name} leaves the performance area ({d_wall[k]:.2f} m outside its outline, "
                       f"{h[k]:.2f} m high), where the cameras are not planned to see it.")
            _merge(rep.findings, "outside", "error", (name,), t[k], P[i, k], msg)
        for k in np.nonzero(airborne & near)[0]:
            _merge(rep.findings, "margin", "warning", (name,), t[k], P[i, k],
                   f"{name} flies within {margin:g} m of a wall or the ceiling, outside the region where "
                   f"coverage is proven (planner inset).")

    # --- limits
    vh = np.linalg.norm(V[..., :2], axis=-1)
    up, down = -V[..., 2], V[..., 2]
    acc = np.linalg.norm(A, axis=-1)
    for i, name in enumerate(names):
        for k in np.nonzero(active[i] & ((vh[i] > lim.max_speed_h_mps + 1e-6) | (up[i] > lim.max_speed_up_mps + 1e-6)
                                         | (down[i] > lim.max_speed_down_mps + 1e-6)))[0]:
            _merge(rep.findings, "speed", "error", (name,), t[k], P[i, k],
                   f"{name} is too fast: {vh[i, k]:.2f} m/s horizontal, {max(up[i, k], -down[i, k]) + 0.0:+.2f} m/s "
                   f"vertical "
                   f"(limits {lim.max_speed_h_mps:g} / up {lim.max_speed_up_mps:g} / down {lim.max_speed_down_mps:g}).")
        for k in np.nonzero(active[i] & (acc[i] > lim.max_accel_mps2 + 1e-6))[0]:
            _merge(rep.findings, "accel", "error", (name,), t[k], P[i, k],
                   f"{name} accelerates at {acc[i, k]:.2f} m/s² (limit {lim.max_accel_mps2:g}).")

    # --- separation between drones
    N = len(names)
    min_sep = np.inf
    if N > 1:
        iu, ju = np.triu_indices(N, 1)
        for a in range(0, len(t), 400):
            seg = P[:, a:a + 400]
            D = np.linalg.norm(seg[iu] - seg[ju], axis=-1)                     # (pairs, T)
            both = active[iu, a:a + 400] & active[ju, a:a + 400]
            D = np.where(both, D, np.inf)
            min_sep = min(min_sep, float(D.min()))
            for p, k in zip(*np.nonzero(D < lim.min_separation_m)):
                i, j = iu[p], ju[p]
                _merge(rep.findings, "separation", "error", (names[i], names[j]), t[a + k], P[i, a + k],
                       f"{names[i]} and {names[j]} come within {D[p, k]:.2f} m of each other "
                       f"(minimum {lim.min_separation_m:g} m).")
        homes = P[:, 0]
        for i in range(N):
            for j in range(i + 1, N):
                dist = float(np.linalg.norm(homes[i, :2] - homes[j, :2]))
                if dist < lim.min_home_spacing_m:
                    rep.findings.append(Finding("home", "error", (names[i], names[j]), float(t[0]), float(t[0]),
                                                tuple(np.round(homes[i], 2)),
                                                f"{names[i]} and {names[j]} start {dist:.2f} m apart "
                                                f"(minimum {lim.min_home_spacing_m:g} m)."))

    # --- camera coverage along the flight (the planner's accuracy model)
    covered = None
    if cameras:
        pos = np.array([c.position_m for c in cameras])
        R = np.array([c.R for c in cameras])
        fp = np.array([site.sensor.f_px(c.lens_mm) for c in cameras])
        step = max(1, int(round(coverage_dt_s / lim.sample_dt_s)))
        ks = np.arange(0, len(t), step)
        good = tot = 0
        for i, name in enumerate(names):
            sel = ks[active[i, ks] & (xyz[i, ks, 2] > 0.3)]
            if not len(sel):
                continue
            vis, inf = contributions(pos, R, fp, xyz[i, sel], site.sensor, req.noise, site.limits)
            n = vis.sum(0)
            p95 = rms_mm(inf.sum(0)) * P95_PER_RMS
            okk = (n >= req.min_cameras) & (p95 <= req.p95_mm)
            good += int(okk.sum())
            tot += len(sel)
            for k, ok_k, nk, pk in zip(sel, okk, n, p95):
                if not ok_k:
                    why = (f"seen by {nk} camera(s), needs {req.min_cameras}" if nk < req.min_cameras else
                           f"predicted error {pk:.1f} mm, limit {req.p95_mm:g} mm")
                    _merge(rep.findings, "coverage", "error", (name,), t[k], P[i, k],
                           f"{name} flies where tracking is not good enough: {why}.", gap=coverage_dt_s * 2)
        covered = good / tot if tot else 1.0

    rep.stats = {
        "max_speed_mps": float(np.max(np.linalg.norm(V, axis=-1)[active])) if active.any() else 0.0,
        "max_accel_mps2": float(np.max(acc[active])) if active.any() else 0.0,
        "min_separation_m": float(min_sep) if np.isfinite(min_sep) else float("inf"),
        "covered_fraction": float(covered) if covered is not None else float("nan"),
        "max_height_m": float(-P[..., 2].min()),
    }
    rep.findings.sort(key=lambda x: (x.severity != "error", x.t))
    return rep
