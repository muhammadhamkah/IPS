"""Self-contained HTML report: summary, layout plan view, per-altitude
coverage (camera count) and predicted-accuracy maps, camera table."""
from __future__ import annotations

import base64
import html
import io
from pathlib import Path

import numpy as np

from idps.core.camera import direction_from_az_el
from idps.coverage.volume import offset_ring
from idps.plan.model import Plan
from idps.plan.planner import Evaluation
from idps.plan.site import Site

# Reference palette (dataviz skill): status steps for the camera-count map,
# one-hue blue ramp for the accuracy map, categorical slots for mount types.
STATUS = {"0": "#d03b3b", "1": "#ec835a", "2": "#fab219", "3+": "#0ca30c"}
BLUE_RAMP = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]
SERIES = {"truss": "#2a78d6", "tripod": "#eb6834"}
TEXT, TEXT2, SURFACE, GRID = "#0b0b0b", "#52514e", "#fcfcfb", "#e4e3df"


def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": TEXT2, "xtick.color": TEXT2,
        "ytick.color": TEXT2, "axes.titlecolor": TEXT, "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    })
    return plt


def _png(fig) -> str:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
    import matplotlib.pyplot as plt
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def _edges(c: np.ndarray) -> np.ndarray:
    if len(c) == 1:
        return np.array([c[0] - 0.5, c[0] + 0.5])
    mid = (c[:-1] + c[1:]) / 2
    return np.concatenate([[c[0] - (mid[0] - c[0])], mid, [c[-1] + (c[-1] - mid[-1])]])


def layout_figure(site: Site, plan: Plan) -> str:
    plt = _mpl()
    fig, ax = plt.subplots(figsize=(7.5, 7.5))
    (x0, y0), (x1, y1) = site.volume_min[:2], site.volume_max[:2]
    poly = site.volume.polygon
    ax.add_patch(plt.Polygon(poly, closed=True, fill=False, ec=TEXT2, lw=1.2))
    centre = poly.mean(0)
    for k, v in enumerate(poly):
        off = v - centre
        off = off / (np.linalg.norm(off) + 1e-9) * 10
        ax.annotate(f"V{k}", v, fontsize=7, color=TEXT, xytext=off, textcoords="offset points",
                    ha="center", va="center")
    for z in site.tripod_zones:
        if z.polygon is not None:
            ax.add_patch(plt.Polygon(z.polygon, closed=True, fc="#f0efec", ec="none", zorder=0))
        else:
            for d in z.offset_m:
                ring = offset_ring(poly, d, 1e9)          # offset vertices only
                ring = ring[1::2] if len(ring) == 2 * len(poly) else ring
                ax.add_patch(plt.Polygon(ring, closed=True, fill=False, ec="#d6d4cd", lw=1,
                                         ls=(0, (3, 2)), zorder=0))
    for r in site.no_go_zones:
        ax.add_patch(plt.Polygon(r, closed=True, fc="none", ec=TEXT2, hatch="xx", lw=0.5, zorder=0))
    for t in site.trusses:
        ax.plot(*np.array([t.start, t.end]).T, color=GRID, lw=2, ls=(0, (2, 2)), zorder=1)
        for s0, s1 in t.usable:
            ax.plot(*np.array([t.point_at(s0), t.point_at(s1)]).T, color="#b4b2ab", lw=2.5, zorder=1)
        ax.annotate(t.id, t.start, fontsize=7, color=TEXT2, xytext=(-4, 3), textcoords="offset points",
                    ha="right")
    span = max(x1 - x0, y1 - y0)
    arrow = span * 0.07
    for kind, colour in SERIES.items():
        cams = [c for c in plan.cameras if c.mount_type == kind]
        if not cams:
            continue
        P = np.array([c.position_m for c in cams])
        ax.scatter(P[:, 0], P[:, 1], s=22, color=colour, ec=SURFACE, lw=1, zorder=3,
                   label=f"{kind} camera ({len(cams)})")
        for c in cams:
            d = direction_from_az_el(c.azimuth_deg, c.elevation_deg)
            if np.hypot(d[0], d[1]) > 0.05:
                ax.annotate("", xy=(c.position_m[0] + d[0] * arrow, c.position_m[1] + d[1] * arrow),
                            xytext=c.position_m[:2],
                            arrowprops=dict(arrowstyle="-|>", color=colour, lw=1.2, mutation_scale=8),
                            zorder=2)
        if len(plan.cameras) <= 60:
            for c in cams:
                ax.annotate(c.id, c.position_m[:2], fontsize=6, color=TEXT2, xytext=(3, 3),
                            textcoords="offset points")
    ax.plot([0, span * 0.06], [0, 0], color=TEXT, lw=1.5)
    ax.plot([0, 0], [0, span * 0.06], color=TEXT, lw=1.5)
    ax.annotate("+X", (span * 0.06, 0), fontsize=7, color=TEXT, xytext=(3, -3),
                textcoords="offset points")
    ax.annotate("+Y", (0, span * 0.06), fontsize=7, color=TEXT, xytext=(-3, 3),
                textcoords="offset points", ha="right")
    ax.annotate("origin", (0, 0), fontsize=7, color=TEXT, xytext=(-4, 4), textcoords="offset points",
                ha="right")
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Camera layout (plan view). Arrows: aim direction; dashed grey: no usable truss"
                 " / tripod strip limits",
                 fontsize=9, loc="left")
    ax.grid(color=GRID, lw=0.5)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1), frameon=False)
    return _png(fig)


def level_figure(site: Site, ev: Evaluation, k: int) -> str:
    plt = _mpl()
    from matplotlib.colors import BoundaryNorm, LinearSegmentedColormap, ListedColormap
    g = ev.grid
    xs, ys, zs = g.xs, g.ys, g.zs
    lvl = g.level_of()
    on = lvl == k
    grid_idx = np.where(on[:g.n_grid])[0]
    ring_idx = np.where(on)[0]
    ring_idx = ring_idx[ring_idx >= g.n_grid]
    inside = g.inside
    n_map = np.full(inside.shape, np.nan)
    p_map = np.full(inside.shape, np.nan)
    n_map[inside] = ev.n_cams[grid_idx]
    p_map[inside] = ev.p95_mm[grid_idx]
    n_map, p_map = n_map.T, p_map.T
    limit = site.requirements.p95_mm
    xe, ye = _edges(xs), _edges(ys)
    poly = site.volume.polygon
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.8), layout="constrained")

    cmap = ListedColormap([STATUS["0"], STATUS["1"], STATUS["2"], STATUS["3+"]])
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5, 3.5], 4)
    q1 = a1.pcolormesh(xe, ye, np.ma.masked_invalid(np.minimum(n_map, 3)), cmap=cmap, norm=norm, ec=SURFACE,
                       lw=1)
    rp = ev.points[ring_idx]
    a1.scatter(rp[:, 0], rp[:, 1], c=np.minimum(ev.n_cams[ring_idx], 3), cmap=cmap, norm=norm, s=14,
               marker="s", ec=SURFACE, lw=0.6, zorder=3)
    if inside.sum() <= 400:
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                if np.isfinite(n_map[j, i]):
                    v = int(n_map[j, i])
                    a1.text(x, y, str(v), ha="center", va="center", fontsize=6,
                            color="#ffffff" if v == 0 or v >= 3 else TEXT)
    handles = [plt.Rectangle((0, 0), 1, 1, color=STATUS[k_]) for k_ in ("0", "1", "2", "3+")]
    a1.legend(handles, ["0 cameras", "1 camera", "2 cameras", "3+ cameras"], loc="upper center",
              bbox_to_anchor=(0.5, -0.12), ncol=4, frameon=False, fontsize=7)
    a1.set_title(f"Cameras seeing each point, Z = {zs[k]:.1f} m (squares: edge points)", fontsize=9,
                 loc="left")

    ramp = LinearSegmentedColormap.from_list("blue", BLUE_RAMP)

    def shown(p):
        return np.where(np.isfinite(p), np.minimum(p, 2 * limit), 2 * limit)

    m = a2.pcolormesh(xe, ye, np.ma.masked_where(np.isnan(p_map), shown(p_map)), cmap=ramp, vmin=0,
                      vmax=2 * limit, ec=SURFACE, lw=1)
    fail = ~(p_map <= limit) & ~np.isnan(p_map)
    if fail.any():
        a2.pcolormesh(xe, ye, np.ma.masked_where(~fail, np.ones_like(p_map)), cmap=ListedColormap(["none"]),
                      hatch="////", ec=STATUS["0"], lw=1)
    rpv = ev.p95_mm[ring_idx]
    a2.scatter(rp[:, 0], rp[:, 1], c=shown(rpv), cmap=ramp, vmin=0, vmax=2 * limit, s=14, marker="s",
               ec=np.where(rpv <= limit, SURFACE, STATUS["0"]), lw=0.8, zorder=3)
    cb = fig.colorbar(m, ax=a2, shrink=0.85)
    cb.set_label(f"predicted p95 3D error (mm), capped at {2 * limit:g}")
    cb.ax.axhline(limit, color=TEXT, lw=1)
    a2.legend([plt.Rectangle((0, 0), 1, 1, fc="none", ec=STATUS["0"], hatch="////")],
              [f"over {limit:g} mm or < 2 cameras"], loc="upper center", bbox_to_anchor=(0.5, -0.12),
              frameon=False, fontsize=7)
    a2.set_title(f"Predicted accuracy, Z = {zs[k]:.1f} m", fontsize=9, loc="left")
    for a, mesh in ((a1, q1), (a2, m)):
        outline = plt.Polygon(poly, closed=True, fill=False, ec=TEXT2, lw=1, zorder=4)
        a.add_patch(outline)
        mesh.set_clip_path(outline)          # cells never spill past the drawn volume
        for coll in a.collections:
            if coll is not mesh and coll.get_array() is not None and coll.get_offsets().size == 0:
                coll.set_clip_path(outline)
        a.set_aspect("equal")
        a.set_xlabel("X (m)")
        a.set_ylabel("Y (m)")
    return _png(fig)


CSS = f"""
body {{ background:{SURFACE}; color:{TEXT}; font:14px/1.45 system-ui, sans-serif; margin:0; }}
main {{ max-width:1180px; margin:0 auto; padding:24px 16px 64px; }}
h1 {{ font-size:22px; margin:0 0 4px; }} h2 {{ font-size:17px; margin:32px 0 8px; }}
.muted {{ color:{TEXT2}; }}
.tiles {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(170px,1fr)); gap:12px; margin:16px 0; }}
.tile {{ border:1px solid {GRID}; border-radius:8px; padding:10px 12px; }}
.tile b {{ display:block; font-size:22px; }}
.ok {{ color:#006300; }} .bad {{ color:#b42828; }}
table {{ border-collapse:collapse; width:100%; font-size:12px; }}
th, td {{ border-bottom:1px solid {GRID}; padding:4px 6px; text-align:left; white-space:nowrap; }}
td.note {{ white-space:normal; color:{TEXT2}; }}
.scroll {{ overflow-x:auto; }}
img {{ max-width:100%; }}
"""


def write_report(path: str | Path, site: Site, plan: Plan, ev: Evaluation, title: str | None = None):
    s = plan.summary
    req = site.requirements
    zs = ev.grid.zs
    lvl = ev.grid.level_of()
    ok_all = s["fraction_meeting_all_requirements"] >= 1.0 - 1e-12

    def pct(v):
        return f"{100 * v:.1f} %"

    worst = s.get("p95_error_mm_worst")
    tiles = [
        ("Cameras", f"{s['cameras']}", f"{s['cameras_on_trusses']} on trusses, "
                                       f"{s['cameras_on_tripods']} on {s['tripods']} tripods"),
        ("Meets all requirements", pct(s["fraction_meeting_all_requirements"]),
         f"&ge; {req.min_cameras} cameras and p95 &le; {req.p95_mm:g} mm"),
        (f"&ge; {req.min_cameras} cameras", pct(s["fraction_with_min_cameras"]),
         f"{s['volume_below_min_cameras_m3']:.0f} m&sup3; below {req.min_cameras}"),
        ("Survives losing any one camera", pct(s["fraction_meeting_with_any_one_camera_lost"]),
         "points still meeting all requirements"),
        ("Predicted p95 error", f"{s['p95_error_mm_median']:.1f} mm" if s["p95_error_mm_median"] else "n/a",
         f"median; worst {worst:.1f} mm" if worst is not None else "worst: unbounded (points with < 2 cameras)"),
    ]
    tiles_html = "".join(f'<div class="tile"><span class="muted">{a}</span><b>{b}</b>'
                         f'<span class="muted">{c}</span></div>' for a, b, c in tiles)

    def level_row(k, z):
        m = lvl == k
        p = ev.p95_mm[m]
        fin = p[np.isfinite(p)]
        worst_k = f"{p.max():.1f}" if np.isfinite(p).all() else "unbounded"
        med = f"{np.median(fin):.1f}" if fin.size else "n/a"
        return (f"<tr><td>{z:.1f}</td><td>{pct(np.mean(ev.meets[m]))}</td>"
                f"<td>{pct(np.mean(ev.n_cams[m] >= req.min_cameras))}</td><td>{int(ev.n_cams[m].min())}</td>"
                f"<td>{med}</td><td>{worst_k}</td></tr>")

    level_rows = "".join(level_row(k, z) for k, z in enumerate(zs))

    cam_rows = "".join(
        f"<tr><td>{c.id}</td><td>{c.mount_type}</td><td>{c.mount_id}</td>"
        f"<td>{'' if c.along_truss_m is None else f'{c.along_truss_m:.2f}'}</td>"
        f"<td>{c.position_m[0]:.2f}</td><td>{c.position_m[1]:.2f}</td><td>{c.position_m[2]:.2f}</td>"
        f"<td>{c.base_heading_deg:.0f}</td><td>{c.pan_deg:.1f}</td><td>{c.tilt_deg:.1f}</td>"
        f"<td>{c.lens_mm:g}</td>"
        f"<td>{'' if c.neighbour_spacing_m is None else f'{c.neighbour_spacing_m:.2f}'}</td>"
        f"<td>{int(ev.critical[i])}</td><td class='note'>{html.escape(c.install_note)}</td></tr>"
        for i, c in enumerate(plan.cameras))

    levels_html = "".join(f'<img alt="Coverage and accuracy at Z = {z:.1f} m" '
                          f'src="data:image/png;base64,{level_figure(site, ev, k)}">'
                          for k, z in enumerate(zs))
    noise = req.noise
    vertex_list = ", ".join(f"V{k} ({x:g}, {y:g})" for k, (x, y) in enumerate(site.volume.polygon))
    verdict = ('<p class="ok"><b>Every evaluation point meets the requirements.</b></p>' if ok_all else
               f'<p class="bad"><b>{pct(1 - s["fraction_meeting_all_requirements"])} of evaluation points '
               f'do not meet the requirements</b> (hatched / non-green cells below).</p>')
    doc = f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{html.escape(title or 'IDPS camera plan')}</title><style>{CSS}</style></head><body><main>
<h1>{html.escape(title or 'IDPS camera plan')}: {html.escape(site.name)}</h1>
<p class="muted">Flight volume: {len(site.volume.polygon)}-vertex floor outline
({site.volume.floor_area_m2:,.0f} m&sup2;) extruded from Z {site.volume.z_min:g} to {site.volume.z_max:g} m
({site.volume.volume_m3:,.0f} m&sup3;); bounding box X {site.volume_min[0]:g}..{site.volume_max[0]:g} m,
Y {site.volume_min[1]:g}..{site.volume_max[1]:g} m. Vertices: {vertex_list}. Coordinates are in the tracking frame (origin = calibration square).
Evaluated on {s['evaluation_points']} points spaced about {site.eval_spacing_m[0]:g} x {site.eval_spacing_m[1]:g} x {site.eval_spacing_m[2]:g} m,
including the boundary. Predictions assume the <b>error budget</b>: centroid {noise.sigma_px:g} px,
camera rotation {noise.sigma_rot_rad * 1e3:g} mrad, camera position {noise.sigma_pos_m * 1e3:g} mm,
timing {noise.sigma_t_s * 1e3:g} ms at {noise.speed_mps:g} m/s. No occlusion by drones or obstacles is modelled.
The optimiser designed for <b>&ge; {site.design.min_cameras} cameras and p95 &le; {site.design.p95_mm:g} mm</b>
(margin over the requirement of &ge; {req.min_cameras} cameras and {req.p95_mm:g} mm) so the plan tolerates
occlusion and a lost camera.</p>
{verdict}
<div class="tiles">{tiles_html}</div>
<h2>Layout</h2>
<img alt="Camera layout plan view" src="data:image/png;base64,{layout_figure(site, plan)}">
<h2>Per altitude</h2>
<div class="scroll"><table><tr><th>Z (m)</th><th>meets all</th><th>&ge; {req.min_cameras} cameras</th>
<th>min cameras</th><th>median p95 (mm)</th><th>worst p95 (mm)</th></tr>{level_rows}</table></div>
{levels_html}
<h2>Camera install table</h2>
<p class="muted">Set the gimbal base so its pan-zero points at <i>base heading</i> (world azimuth: 0 = +X, 90 = +Y),
level it, then the servos go to <i>pan</i>/<i>tilt</i>. <i>Critical</i> = evaluation points that would stop meeting
the requirements if this camera were missing.</p>
<div class="scroll"><table><tr><th>ID</th><th>mount</th><th>truss/zone</th><th>along truss (m)</th><th>X</th><th>Y</th>
<th>Z</th><th>base heading</th><th>pan</th><th>tilt</th><th>lens (mm)</th><th>spacing to neighbour (m)</th>
<th>critical</th><th>install note</th></tr>{cam_rows}</table></div>
</main></body></html>"""
    Path(path).write_text(doc)
