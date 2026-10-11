"""Command line: idps-plan optimise SITE.yaml -o OUT/ | idps-plan evaluate PLAN.yaml --site SITE.yaml -o OUT/"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from idps.plan.model import Plan, install_angles
from idps.plan.planner import evaluate, optimise
from idps.plan.site import load_site


def _write_outputs(out: Path, site, plan, ev, title):
    out.mkdir(parents=True, exist_ok=True)
    plan.save(out / "plan.yaml")
    plan.write_csv(out / "cameras.csv")
    try:
        from idps.plan.report import write_report
        write_report(out / "report.html", site, plan, ev, title)
    except ImportError:
        print("note: matplotlib not installed; skipping report.html", file=sys.stderr)
    print(json.dumps({k: v for k, v in plan.summary.items()
                      if k not in ("requirements", "design_margin")}, indent=2))
    print(f"wrote {out}/plan.yaml, cameras.csv, report.html")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="idps-plan", description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    o = sub.add_parser("optimise", help="find camera count, positions and pan/tilt for a site")
    o.add_argument("site")
    o.add_argument("-o", "--out", required=True)
    o.add_argument("--max-cameras", type=int)
    o.add_argument("--verify-spacing", type=float, metavar="M",
                   help="also check the final plan on a finer grid with this spacing (m)")
    e = sub.add_parser("evaluate", help="evaluate an existing (possibly hand-edited) plan")
    e.add_argument("plan")
    e.add_argument("--site", help="site file (default: the one recorded in the plan)")
    e.add_argument("-o", "--out", required=True)
    a = ap.parse_args(argv)

    if a.cmd == "optimise":
        site = load_site(a.site)
        plan, ev = optimise(site, max_cameras=a.max_cameras)
        if a.verify_spacing:
            import copy
            fine = copy.deepcopy(site)
            fine.eval_spacing_m = (a.verify_spacing,) * 3
            fs = evaluate(fine, plan.cameras).summary
            plan.summary["verification_on_finer_grid"] = {
                "spacing_m": a.verify_spacing, "evaluation_points": fs["evaluation_points"],
                "fraction_meeting_all_requirements": fs["fraction_meeting_all_requirements"],
                "fraction_with_min_cameras": fs["fraction_with_min_cameras"],
                "p95_error_mm_worst": fs["p95_error_mm_worst"]}
        _write_outputs(Path(a.out), site, plan, ev, "IDPS camera plan")
    else:
        plan = Plan.load(a.plan)
        site_path = a.site or plan.site_file
        if not site_path:
            ap.error("plan has no site_file; pass --site")
        site = load_site(site_path)
        # Position and azimuth/elevation are the source of truth; the installer
        # angles are re-derived so a hand-edited plan stays consistent.
        for c in plan.cameras:
            c.base_heading_deg, c.pan_deg, c.tilt_deg = install_angles(c.azimuth_deg, c.elevation_deg)
        ev = evaluate(site, plan.cameras)
        plan.summary = ev.summary
        _write_outputs(Path(a.out), site, plan, ev, "IDPS plan evaluation")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
