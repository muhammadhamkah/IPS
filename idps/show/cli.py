"""idps-show: check a show designed in Blender (Skybrush Studio) against the venue, export
one trajectory file per PX4 drone, and build the venue inside Blender.

  idps-show blender-venue --site venue.yaml --plan plan.yaml -o idps_venue.py
  idps-show check  show.skyc --site venue.yaml --plan plan.yaml
  idps-show export show.skyc --site venue.yaml --plan plan.yaml -o out/show
  idps-show fly    out/show/manifest.json --link 7=udpout:192.168.50.107:14550 ...   (PC -> drones, by drone ID)
  idps-show play   out/show/drone_007.json --dry-run        (one drone, e.g. PX4 SITL)
  idps-show demo   -o demo.skyc --drones 8
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _site_and_cams(a):
    from idps.plan.model import Plan
    from idps.plan.site import load_site
    site = load_site(a.site)
    cams = Plan.load(a.plan).cameras if a.plan else None
    return site, cams


def _placement(a):
    from idps.show.placement import Placement
    off = tuple(float(x) for x in a.offset.split(",")) if a.offset else (0.0, 0.0, 0.0)
    return Placement(a.axes.upper(), a.rotate, off)


def _limits(a):
    from idps.show.check import Limits
    return Limits(max_speed_h_mps=a.max_speed, max_accel_mps2=a.max_accel, min_separation_m=a.min_sep,
                  min_home_spacing_m=a.min_sep)


def _print_report(rep, limit=25):
    print(rep.summary())
    for f in rep.findings[:limit]:
        span = f"{f.t:6.2f} s" + (f"–{f.t_last:.2f} s" if f.t_last > f.t + 1e-9 else "")
        mark = "ERROR  " if f.severity == "error" else "warning"
        print(f"  {mark} {span:>16}  N {f.ned[0]:6.2f} E {f.ned[1]:6.2f} D {f.ned[2]:6.2f}  {f.message}")
    if len(rep.findings) > limit:
        print(f"  … and {len(rep.findings) - limit} more")


def cmd_check(a) -> int:
    from idps.show.check import check_show
    from idps.show.readers import load_show
    site, cams = _site_and_cams(a)
    show = load_show(a.show)
    rep = check_show(show, site, cams, _placement(a), _limits(a))
    if not cams:
        print("(no --plan given: camera coverage along the paths is not checked)")
    _print_report(rep)
    return 0 if rep.ok else 1


def cmd_export(a) -> int:
    from idps.show.check import check_show
    from idps.show.export import export_px4
    from idps.show.readers import load_show
    site, cams = _site_and_cams(a)
    show = load_show(a.show)
    pl = _placement(a)
    rep = check_show(show, site, cams, pl, _limits(a))
    _print_report(rep, limit=10)
    if not rep.ok and not a.force:
        print("Not exported: fix the errors above, or pass --force to export anyway (not for flying).")
        return 1
    path = export_px4(show, a.out, pl, a.rate, rep, venue={"site": str(a.site), "origin": site.frame.to_dict()})
    print(f"wrote {len(show.drones)} drone files (by drone ID) and {path}")
    return 0


def cmd_blender(a) -> int:
    from idps.show.blender import blender_script
    site, cams = _site_and_cams(a)
    Path(a.out).write_text(blender_script(site, cams, _placement(a)))
    print(f"wrote {a.out}: in Blender, Scripting tab -> Open -> Run Script, then design inside 'IDPS venue'")
    return 0


def cmd_demo(a) -> int:
    from idps.show.demo import write_demo_skyc
    p = write_demo_skyc(a.out, n=a.drones, radius=a.radius, height=a.height)
    print(f"wrote {p}")
    return 0


def cmd_play(a) -> int:
    from idps.show.runner import main as play
    return play([a.drone_file] + (["--dry-run"] if a.dry_run else []) + (["--start", str(a.start)] if a.start else [])
                + ["--device", a.device, "--baud", str(a.baud)])


def cmd_fly(a) -> int:
    from idps.show.fleet import FleetPlayer, parse_links
    links = parse_links(a.link)
    if not links:
        print("give each drone's link: --link DRONE_ID=DEVICE (e.g. 7=udpout:192.168.50.107:14550)")
        return 2
    if not a.sitl:
        print("Live positions come from the IDPS engine, which is not built yet. Use --sitl to fly "
              "PX4 SITL drones (they position themselves).")
        return 2
    from pymavlink import mavutil
    import time
    mavs = {}
    for did, dev in links.items():
        m = mavutil.mavlink_connection(dev)
        m.wait_heartbeat()
        if m.target_system != did:
            print(f"{dev} answered as MAV_SYS_ID {m.target_system}, not drone {did}: wrong drone or ID not set")
            return 1
        mavs[did] = m
    fleet = FleetPlayer(a.manifest, mavs)
    start = a.start if a.start else time.time() + 5.0
    print(f"flying {len(mavs)} drone(s), show starts in {start - time.time():.1f} s; "
          "arm and switch to OFFBOARD now")
    fleet.run(start)
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="idps-show", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def venue_args(p, need_plan=False):
        p.add_argument("--site", required=True, help="venue site file (performance area, origin, requirements)")
        p.add_argument("--plan", required=need_plan, help="camera plan (plan.yaml) for the coverage check")
        p.add_argument("--axes", default="ENU", choices=["ENU", "NWU", "NED", "enu", "nwu", "ned"],
                       help="show axes: ENU = Blender +X East, +Y North, +Z up (default)")
        p.add_argument("--rotate", type=float, default=0.0, help="turn the show clockwise (degrees)")
        p.add_argument("--offset", help="move the show: N,E,D metres")

    def limit_args(p):
        p.add_argument("--max-speed", type=float, default=3.0, help="horizontal speed limit (m/s)")
        p.add_argument("--max-accel", type=float, default=4.0, help="acceleration limit (m/s²)")
        p.add_argument("--min-sep", type=float, default=1.0, help="minimum distance between drones (m)")

    c = sub.add_parser("check", help="check a show against the venue, cameras and drone limits")
    c.add_argument("show", help=".skyc, Skybrush CSV ZIP, folder of CSVs or one CSV")
    venue_args(c)
    limit_args(c)
    e = sub.add_parser("export", help="check, then write one file per drone ID + manifest.json")
    e.add_argument("show")
    venue_args(e)
    limit_args(e)
    e.add_argument("-o", "--out", default="out/show")
    e.add_argument("--rate", type=float, default=20.0, help="samples per second in the drone files")
    e.add_argument("--force", action="store_true", help="export even if the check finds errors")
    b = sub.add_parser("blender-venue", help="write a Blender script that builds the venue in Blender")
    venue_args(b)
    b.add_argument("-o", "--out", default="idps_venue.py")
    d = sub.add_parser("demo", help="write a small demo show (.skyc)")
    d.add_argument("-o", "--out", default="demo.skyc")
    d.add_argument("--drones", type=int, default=8)
    d.add_argument("--radius", type=float, default=2.4)
    d.add_argument("--height", type=float, default=1.8)
    f = sub.add_parser("fly", help="fly a show from the PC: positions and setpoints to every drone")
    f.add_argument("manifest", help="manifest.json from idps-show export")
    f.add_argument("--link", action="append", help="DRONE_ID=DEVICE, one per drone (MAVLink over Wi-Fi/UDP); the drone's MAV_SYS_ID must match")
    f.add_argument("--start", type=float, help="show start, Unix time (s); default: now + 5 s")
    f.add_argument("--sitl", action="store_true", help="PX4 SITL drones: setpoints only, no IDPS positions")
    p = sub.add_parser("play", help="play one drone file through PX4 offboard (one drone, e.g. SITL)")
    p.add_argument("drone_file")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--start", type=float)
    p.add_argument("--device", default="/dev/ttyAMA0")
    p.add_argument("--baud", type=int, default=921600)
    a = ap.parse_args(argv)
    return {"check": cmd_check, "export": cmd_export, "blender-venue": cmd_blender, "demo": cmd_demo,
            "fly": cmd_fly, "play": cmd_play}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
