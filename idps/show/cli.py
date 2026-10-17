"""idps-show: check a show designed in Blender (Skybrush Studio) against the venue, export
one trajectory file per PX4 drone, and build the venue inside Blender.

  idps-show blender-venue --site venue.yaml --plan plan.yaml -o idps_venue.py
  idps-show check  show.skyc --site venue.yaml --plan plan.yaml
  idps-show export show.skyc --site venue.yaml --plan plan.yaml -o out/show
  idps-show upload out/show/manifest.json --link 7=udpout:192.168.50.107:14550 ...  (file to each drone's SD card)
  idps-show start  out/show/manifest.json --link ...       (all drones start together; Ctrl+C aborts)
  idps-show status out/show/manifest.json --link ...
  idps-show abort  --link ...
  idps-show play   out/show/drone_007.json --dry-run        (print one drone's path; testing only)
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


def _pads(a):
    if not getattr(a, "pads", None):
        return None
    from idps.show.pads import load_pads
    return load_pads(a.pads)


def _limits(a):
    from idps.show.check import Limits
    return Limits(max_speed_h_mps=a.max_speed, max_accel_mps2=a.max_accel, min_separation_m=a.min_sep,
                  min_home_spacing_m=a.min_sep)


def _print_report(rep, limit=25):
    print(rep.summary())
    if rep.stats_pads:
        s = rep.stats_pads
        print("Charging pads: take-off {:.1f} s (closest {:.2f} m), landing after the show {:.1f} s (closest {:.2f} m), "
              "after a hold timeout {:.1f} s (closest {:.2f} m).".format(
                  s["take-off"], s["take-off closest"], s["landing after the show"],
                  s["landing after the show closest"], s["landing after a hold timeout"],
                  s["landing after a hold timeout closest"]))
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
    rep = check_show(show, site, cams, _placement(a), _limits(a), pads=_pads(a))
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
    pads = _pads(a)
    rep = check_show(show, site, cams, pl, _limits(a), pads=pads)
    _print_report(rep, limit=10)
    if not rep.ok and not a.force:
        print("Not exported: fix the errors above, or pass --force to export anyway (not for flying).")
        return 1
    path = export_px4(show, a.out, pl, a.rate, rep, venue={"site": str(a.site), "origin": site.frame.to_dict()},
                      pads=pads)
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


def _connect(a) -> dict:
    """--link DRONE_ID=DEVICE -> {drone ID: pymavlink connection}; each drone must answer with
    that MAV_SYS_ID (a mixed-up address would otherwise reach the wrong drone)."""
    import time
    from pymavlink import mavutil
    from idps.show import onboard as ob
    from idps.show.fleet import parse_links
    links = parse_links(a.link)
    if not links:
        raise SystemExit("give each drone's link: --link DRONE_ID=DEVICE (e.g. 7=udpout:192.168.50.107:14550)")
    mavs = {}
    for did, dev in links.items():
        m = mavutil.mavlink_connection(dev, source_system=255, source_component=190)
        hb, end = None, time.monotonic() + 10
        while hb is None and time.monotonic() < end:     # the drone's own heartbeat, not a ground station's it forwards
            hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if hb is not None and hb.type == mavutil.mavlink.MAV_TYPE_GCS:
                hb = None
        if hb is None:
            raise SystemExit(f"drone {did}: no heartbeat on {dev}")
        m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
        if m.target_system != did:
            raise SystemExit(f"{dev} answered as MAV_SYS_ID {m.target_system}, not drone {did}: wrong drone or ID not set")
        ob.send_heartbeat(m)
        mavs[did] = m
    time.sleep(0.5)                                  # let PX4 register the ground station
    return mavs


def cmd_upload(a) -> int:
    from idps.show import onboard as ob
    drones = ob.drones_from_manifest(a.manifest, _connect(a))
    path = "idps_show.trj" if a.sitl else ob.SD_PATH
    bad = 0
    for did, d in sorted(drones.items()):
        for attempt in range(3):            # file transfer over UDP: a busy link can lose some of it
            try:
                ob.upload(d.mav, d.trj, path)
            except Exception as e:          # noqa: BLE001 - reported below as the drone's answer
                print(f"drone {did}: upload attempt {attempt + 1} failed: {e}")
            r = ob.command(d.mav, did, ob.LOAD, d.crc)
            ok = r.ok and r.crc == d.crc
            if ok:
                break
        bad += not ok
        print(f"drone {did}: {'OK' if ok else 'FAILED'}  {len(d.trj)} bytes, crc {d.crc:08x}, drone says {r.result} / "
              f"{r.state}, crc {r.crc:08x} {r.text}")
    return 1 if bad else 0


def cmd_status(a) -> int:
    from idps.show import onboard as ob
    drones = ob.drones_from_manifest(a.manifest, _connect(a))
    for did, d in sorted(drones.items()):
        r = ob.command(d.mav, did, ob.STATUS)
        match = "matches" if r.crc == d.crc else "DIFFERS from the show (upload again)"
        print(f"drone {did}: {r.state}, file crc {r.crc:08x} {match}")
    return 0


def cmd_start(a) -> int:
    import time
    from idps.show import onboard as ob
    mavs = _connect(a)
    drones = ob.drones_from_manifest(a.manifest, mavs)
    sync = ob.ClockSync(mavs).start()
    print(f"clock sync to {len(mavs)} drone(s); starting in {ob.ClockSync.WARMUP_S:.0f} s ...")
    time.sleep(ob.ClockSync.WARMUP_S)
    import json
    man = json.loads(open(a.manifest).read())
    duration = man["duration_s"]
    pads = man.get("pads")
    if a.positions and not _wait_tracked(a, sync):
        return 1
    if pads:
        # charging pads: launch (batches take off and fly to the air grid), wait until every drone
        # holds there, then start the show from the air
        nb = max(d.get("batch", 1) for d in man["drones"])
        gap = float(pads["batch_gap_s"])
        t_launch, replies = ob.start_all(drones, lead_s=a.lead, action=ob.LAUNCH)
        for did, r in sorted(replies.items()):
            print(f"drone {did}: launch {r.result} ({r.state}) {r.text}")
        if not all(r.ok for r in replies.values()):
            print("Not started: every drone was told to abort. Fix the refused drone(s) and start again.")
            sync.stop()
            return 1
        print(f"launch in {a.lead:.0f} s: {nb} batch(es) {gap:g} s apart, then every drone holds in the grid", flush=True)
        give_up = t_launch / 1e6 + gap * (nb - 1) + float(pads["hold_timeout_s"]) + gap * nb + 40.0
        states = ob.wait_holding(drones, timeout_s=a.lead + gap * (nb - 1) + 40.0)
        late = [did for did, r in states.items() if r.progress_code != ob.HOLDING]
        if late:
            print("Not started: drone(s) " + ", ".join(f"{did} ({states[did].state})" for did in sorted(late))
                  + " not holding in the grid; the others land on their pads when the hold times out.")
            _wait_until(give_up, sync)
            return 1
        print(f"every drone holds in the grid ({len(drones)})", flush=True)
        t_pc, replies = ob.start_all(drones, lead_s=6.0, action=ob.START, abort_on_refusal=False)
        for did, r in sorted(replies.items()):
            print(f"drone {did}: {r.result} ({r.state}) {r.text}")
        if not all(r.ok for r in replies.values()):
            print("Not started: a drone refused the start; every drone lands on its pad when the hold times out.")
            _wait_until(give_up, sync)
            return 1
        end = t_pc / 1e6 + duration + gap * nb + 30.0      # the show, then the batches back to their pads
    else:
        t_pc, replies = ob.start_all(drones, lead_s=a.lead)
        for did, r in sorted(replies.items()):
            print(f"drone {did}: {r.result} ({r.state}) {r.text}")
        if not all(r.ok for r in replies.values()):
            print("Not started: every drone was told to abort. Fix the refused drone(s) and start again.")
            sync.stop()
            return 1
        end = t_pc / 1e6 + duration + 15.0
    print(f"show starts in {t_pc / 1e6 - time.monotonic():.0f} s and lasts {duration:.0f} s. Keep this running "
          "(it keeps the drones' clocks); Ctrl+C (or SIGTERM) aborts: every drone lands where it is.")
    print(f"show time 0 = PC clock (time.monotonic) {t_pc / 1e6:.3f} s", flush=True)
    import signal

    def _stop(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _stop)            # a dashboard or script stopping us = abort too
    try:
        while time.monotonic() < end:
            time.sleep(0.2)
    except KeyboardInterrupt:
        for did, d in drones.items():
            print(f"drone {did}: abort -> {ob.command(d.mav, did, ob.ABORT).result}")
        return 1
    finally:
        sync.stop()
    return 0


TRACK_TOL_M = 0.1          # on its own pad slot (the next slot is 0.2 m away)


def _wait_tracked(a, sync) -> bool:
    """No take-off before the cameras see every drone on its own spot: wait for it (up to
    --track-wait s), then say which drones are missing or misplaced."""
    import time
    from idps.show.fleet import Fleet, seen_in_log
    fleet = Fleet(a.manifest, {})
    fleet.links = {did: None for did in fleet.homes}
    end, last = time.monotonic() + a.track_wait, 0.0
    while True:
        problems = fleet.preflight(seen_in_log(a.positions), tol_m=TRACK_TOL_M)
        if not problems:
            print(f"tracked: every drone on its spot ({len(fleet.homes)})", flush=True)
            return True
        if time.monotonic() >= end:
            print("Not started: not every drone is tracked on its spot: " + "; ".join(problems), flush=True)
            sync.stop()
            return False
        if time.monotonic() - last >= 5.0:
            last = time.monotonic()
            print("waiting for tracking: " + "; ".join(problems[:4]) + (" …" if len(problems) > 4 else ""), flush=True)
        time.sleep(0.5)


def _wait_until(t_mono: float, sync) -> None:
    """Keep the drones' clocks synced until t_mono (time.monotonic), e.g. while they land after a hold timeout."""
    import time
    try:
        while time.monotonic() < t_mono:
            time.sleep(0.2)
    finally:
        sync.stop()


def cmd_abort(a) -> int:
    from idps.show import onboard as ob
    for did, m in sorted(_connect(a).items()):
        print(f"drone {did}: abort -> {ob.command(m, did, ob.ABORT).result}")
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
        p.add_argument("--pads", help="charging pads (pads.yaml): the drones take off from them in batches to the "
                       "show's first frame and land back from its last (see idps/show/pads.py)")

    def limit_args(p):
        p.add_argument("--max-speed", type=float, default=3.0, help="horizontal speed limit (m/s)")
        p.add_argument("--max-accel", type=float, default=4.0, help="acceleration limit (m/s²)")
        p.add_argument("--min-sep", type=float, default=1.0, help="minimum distance between drones (m)")

    c = sub.add_parser("check", help="check a show against the venue, cameras and drone limits")
    c.add_argument("show", help=".skyc, Skybrush CSV ZIP, folder of CSVs or one CSV, or a folder/ZIP of XTJZ files")
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
    def link_args(p):
        p.add_argument("--link", action="append", help="DRONE_ID=DEVICE, one per drone, e.g. "
                       "7=udpout:192.168.50.107:14550 (MAVLink over Wi-Fi); the drone's MAV_SYS_ID must match")

    u = sub.add_parser("upload", help="upload each drone's show file to its SD card and verify it")
    u.add_argument("manifest", help="manifest.json from idps-show export")
    link_args(u)
    u.add_argument("--sitl", action="store_true", help="PX4 SITL drones (file in the instance folder)")
    st = sub.add_parser("status", help="each drone's show state and whether its file matches")
    st.add_argument("manifest")
    link_args(st)
    sa = sub.add_parser("start", help="start every drone's show at the same instant (keeps running)")
    sa.add_argument("manifest")
    link_args(sa)
    sa.add_argument("--lead", type=float, default=15.0, help="seconds from the command to the show start")
    sa.add_argument("--positions", help="the engine's live log (idps-engine live -o positions.csv): no take-off "
                    "before the cameras see every drone on its own spot")
    sa.add_argument("--track-wait", type=float, default=60.0, help="how long to wait for that (s)")
    ab = sub.add_parser("abort", help="every drone lands where it is")
    link_args(ab)
    p = sub.add_parser("play", help="testing: stream one drone file from the PC through PX4 offboard, or print it")
    p.add_argument("drone_file")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--start", type=float)
    p.add_argument("--device", default="/dev/ttyAMA0")
    p.add_argument("--baud", type=int, default=921600)
    a = ap.parse_args(argv)
    return {"check": cmd_check, "export": cmd_export, "blender-venue": cmd_blender, "demo": cmd_demo,
            "upload": cmd_upload, "status": cmd_status, "start": cmd_start, "abort": cmd_abort,
            "play": cmd_play}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
