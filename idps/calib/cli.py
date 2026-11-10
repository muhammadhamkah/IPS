"""idps-calib: camera calibration (docs/CALIBRATION.md).

    idps-calib board -o board.svg                      a checkerboard to print
    idps-calib lens b*.pgm --station 1 -o lens_1.yaml  one lens, from pictures of the board
    idps-calib record -o room.idpsraw --seconds 90     record the stations while the LED is moved
    idps-calib room room.idpsraw --lens lens_*.yaml --points points.yaml -o calibration.yaml
"""
from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path
from struct import Struct

import numpy as np
import yaml

_REC = Struct("<QI")                  # the packet log's record: [u64 time ns][u32 length][packet]


def _cmd_board(a) -> int:
    from idps.calib.board import write_board
    p = write_board(a.out, a.cols, a.rows, a.square_mm)
    print(f"{p}: {a.cols} x {a.rows} squares of {a.square_mm:g} mm.\n"
          "Print it at 100 % (no 'fit to page'), measure one square with a ruler, and glue the sheet flat on\n"
          "something stiff. If a square is not exactly that size, pass what you measure as --square-mm to "
          "'idps-calib lens'.")
    return 0


def _cmd_lens(a) -> int:
    from idps.calib.lens import calibrate_lens
    try:
        lens = calibrate_lens(a.images, a.cols, a.rows, a.square_mm, a.station)
    except ImportError:
        print("this needs OpenCV: pip install 'idps[calib]'", file=sys.stderr)
        return 2
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    out = a.out or (f"lens_{a.station}.yaml" if a.station is not None else "lens.yaml")
    lens.save(out)
    hfov = 2 * np.degrees(np.arctan(lens.width / 2 / lens.f_px))
    print(f"lens: focal length {lens.f_px:.1f} px ({hfov:.1f} deg across), centre ({lens.cx:.1f}, {lens.cy:.1f}), "
          f"distortion k1 {lens.distortion[0]:+.4f} k2 {lens.distortion[1]:+.4f}")
    ok = lens.rms_px <= 0.1
    print(f"quality: corners fit to {lens.rms_px:.3f} px over {lens.images_used} pictures "
          f"({'good' if ok else 'usable' if lens.rms_px <= 0.2 else 'poor'}; the target is 0.1 px), "
          f"the board covered {lens.coverage * 100:.0f} % of the image")
    for n in lens.notes:
        print(f"  note: {n}")
    print(f"saved {out}")
    return 0


def _cmd_record(a) -> int:
    from idps.engine.ingest import Ingest
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    sock.bind(("0.0.0.0", a.port))
    sock.settimeout(0.2)
    ing = Ingest()
    counts: dict[int, list] = {}          # station -> [frames, frames with exactly one dot] this second
    total: dict[int, int] = {}
    print(f"recording on UDP port {a.port} for {a.seconds:g} s to {a.out} (Ctrl-C stops early).\n"
          "Leave the LED still for 3 s on each measured point first, in the order of your points file,\n"
          "then move it slowly through the whole space the cameras see.")
    t_end, t_next, n = time.time() + a.seconds, time.time() + 1.0, 0
    with open(a.out, "wb") as f:
        f.write(b"IDPSRAW1")
        try:
            while time.time() < t_end:
                try:
                    data = sock.recv(65536)
                except socket.timeout:
                    data = None
                if data:
                    f.write(_REC.pack(time.time_ns(), len(data)) + data)
                    n += 1
                    for fr in ing.feed(data):
                        c = counts.setdefault(fr.station_id, [0, 0])
                        c[0] += 1
                        c[1] += fr.n == 1
                        total[fr.station_id] = total.get(fr.station_id, 0) + (fr.n == 1)
                if time.time() >= t_next:
                    t_next += 1.0
                    if counts:
                        row = "  ".join(f"st {s}: {'sees it' if c[1] > 0.5 * c[0] else 'no' if c[1] == 0 else 'partly'}"
                                        for s, c in sorted(counts.items()))
                        seeing = sum(c[1] > 0.5 * c[0] for c in counts.values())
                        print(f"{t_end - time.time():5.0f} s left | {row} | {seeing} camera{'s' if seeing != 1 else ''}"
                              f"{'' if seeing >= 2 else '  <- needs 2 or more'}")
                    else:
                        print(f"{t_end - time.time():5.0f} s left | no station is sending (is idps-station running, "
                              f"with --engine this computer's address?)")
                    counts = {}
        except KeyboardInterrupt:
            print("stopped")
    print(f"{n} packets saved to {a.out}; frames with the LED: "
          + (", ".join(f"station {s}: {v}" for s, v in sorted(total.items())) or "none"))
    return 0 if total else 1


def _load_points(path, site) -> list:
    doc = yaml.safe_load(Path(path).read_text())
    pts = doc.get("points", doc) if isinstance(doc, dict) else doc
    frame = None
    out = []
    for p in pts:
        if p is None or (isinstance(p, dict) and p.get("skip")):
            out.append(None)
        elif isinstance(p, dict) and "ned" in p:
            if frame is None:
                if not site:
                    raise ValueError("points given as ned: need --site (the venue file with the origin and North)")
                from idps.plan.site import load_site
                frame = load_site(site).frame
            out.append([float(v) for v in frame.from_ned(np.asarray(p["ned"], float)[None])[0]])
        else:
            out.append([float(v) for v in (p["xyz"] if isinstance(p, dict) else p)])
    return out


def _cmd_room(a) -> int:
    try:
        from idps.calib.solve import solve_room, to_calibration_doc
    except ImportError:
        print("this needs SciPy and OpenCV: pip install 'idps[calib]'", file=sys.stderr)
        return 2
    from idps.calib.lens import Lens, load_lens, nominal_lens
    from idps.calib.tracks import frames_from_packets, observations, single_dot_tracks
    from idps.sim.runner import read_raw

    frames = frames_from_packets(read_raw(a.recording))
    if not frames:
        print("error: no camera frames in the recording", file=sys.stderr)
        return 1
    tracks, _ = single_dot_tracks(frames)
    print("recording:")
    for s, tr in tracks.items():
        print(f"  station {s}: {tr.frames} frames, the LED alone in {len(tr.t)}"
              + (f", {tr.fixed_spots} fixed reflection{'s' if tr.fixed_spots != 1 else ''} ignored" if tr.fixed_spots else ""))
    obs = observations(tracks, max_points=a.max_points)
    print(f"  {len(obs.t)} LED positions seen by 2 or more cameras, {len(obs.rest)} rest point"
          f"{'s' if len(obs.rest) != 1 else ''}"
          + (" at " + ", ".join(f"{obs.t[k]:.0f} s" for k, _ in obs.rest) if obs.rest else ""))

    # lenses: measured files first, then the plan's lens, then a field of view
    planned, lenses, plan_f = None, {}, {}
    if a.plan:
        from idps.core.camera import SENSORS
        from idps.plan.model import Plan
        plan = Plan.load(a.plan)
        planned = {k + 1: (c.pos, c.R[2]) for k, c in enumerate(plan.cameras)}       # station k+1 = k-th camera
        plan_f = {k + 1: SENSORS["OV9281"].f_px(c.lens_mm) for k, c in enumerate(plan.cameras)}
    for spec in a.lens or []:
        sid, _, path = spec.rpartition("=")
        lens = load_lens(path)
        sid = int(sid) if sid else lens.station_id
        if sid is None:
            print(f"error: {path} does not say which station it is for: write --lens ID={path}", file=sys.stderr)
            return 1
        lenses[int(sid)] = lens
    unmeasured = []
    for s in obs.stations:
        if s in lenses:
            continue
        unmeasured.append(s)
        if s in plan_f:
            lenses[s] = Lens(float(plan_f[s]), 639.5, 399.5, [0.0] * 5, notes=["nominal: the plan's lens, not measured"])
        elif a.fov_deg:
            lenses[s] = nominal_lens(a.fov_deg)
        else:
            print(f"error: no lens for station {s}: give --lens lens_{s}.yaml, or --fov-deg (its field of view "
                  "across) to start from the lens's nominal figure", file=sys.stderr)
            return 1
    refine = a.refine_lens or bool(unmeasured)
    if unmeasured:
        print(f"  stations {', '.join(map(str, unmeasured))}: lens not measured; starting from its nominal figure "
              "and refining it here (measured lenses are better: idps-calib lens)")

    refs = None
    if a.points:
        try:
            refs = _load_points(a.points, a.site)
        except (ValueError, KeyError) as e:
            print(f"error: {a.points}: {e}", file=sys.stderr)
            return 1
    print("solving:")
    try:
        res = solve_room(obs, lenses, refs=refs, planned=planned, refine_lens=refine)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    print(f"\nresult: {len(res.cams)} camera{'s' if len(res.cams) != 1 else ''} placed, world frame from {res.frame}")
    for s, c in sorted(res.cams.items()):
        flag = "good" if res.rms_px[s] <= 0.15 else "usable" if res.rms_px[s] <= 0.4 else "POOR"
        print(f"  station {s}: at ({c.position[0]:+.3f}, {c.position[1]:+.3f}, {c.position[2]:+.3f}) m, dots fit to "
              f"{res.rms_px[s]:.3f} px over {res.n_obs[s]} ({flag})"
              + (f", {res.dropped[s]} dropped" if res.dropped[s] else "")
              + (f", focal length {c.f_px:.1f} px" if refine else ""))
    if res.ref_residual_mm:
        worst = max(res.ref_residual_mm)
        print(f"  rest points: within {worst:.1f} mm of where they were measured"
              + ("" if worst < 5 else "  <- check the measured points and their order"))
    for n in res.notes:
        print(f"  note: {n}")
    doc = to_calibration_doc(res)
    doc["recording"] = str(a.recording)
    Path(a.out).write_text(yaml.safe_dump(doc, sort_keys=False, default_flow_style=None, width=120))
    print(f"saved {a.out} (use it with: idps-engine live --calib {a.out} ...)")
    if a.save_lenses and refine:
        for s, l in sorted(res.lenses.items()):
            p = Path(a.save_lenses) / f"lens_{s}.yaml"
            l.save(p)
            print(f"saved {p}")
    return 0 if not res.failed and res.frame != "none" else 3


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="idps-calib", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    b = sub.add_parser("board", help="write a checkerboard to print (SVG)")
    b.add_argument("-o", "--out", default="board.svg")
    b.add_argument("--cols", type=int, default=10, help="squares across (10)")
    b.add_argument("--rows", type=int, default=7, help="squares down (7)")
    b.add_argument("--square-mm", type=float, default=25.0, help="square size (25 mm: fits A4 landscape)")
    b.set_defaults(fn=_cmd_board)

    l = sub.add_parser("lens", help="measure one lens from pictures of the board")
    l.add_argument("images", nargs="+", help="pictures (idps-station snapshot --out ...; PGM, PNG or JPG), 15-25 of them")
    l.add_argument("--station", type=int, help="the station this lens is on")
    l.add_argument("--cols", type=int, default=10)
    l.add_argument("--rows", type=int, default=7)
    l.add_argument("--square-mm", type=float, default=25.0, help="the printed square's real size (measure it)")
    l.add_argument("-o", "--out", help="lens file (default lens_<station>.yaml)")
    l.set_defaults(fn=_cmd_lens)

    r = sub.add_parser("record", help="record the stations' dots while the LED is moved through the room")
    r.add_argument("-o", "--out", default="room.idpsraw")
    r.add_argument("--port", type=int, default=5600)
    r.add_argument("--seconds", type=float, default=90.0)
    r.set_defaults(fn=_cmd_record)

    m = sub.add_parser("room", help="work out where the cameras are from a recording")
    m.add_argument("recording", help="packet log from 'idps-calib record'")
    m.add_argument("--lens", action="append", help="a lens file (repeat; ID=file if the file names no station)")
    m.add_argument("--fov-deg", type=float, help="for stations with no lens file: the lens's field of view across")
    m.add_argument("--refine-lens", action="store_true", help="also refine every lens's focal length and distortion")
    m.add_argument("--points", help="YAML list of where the LED rested, in order: [x, y, z] (plan frame, Z up, m), "
                                    "{ned: [n, e, d]} (needs --site), or null to skip one")
    m.add_argument("--plan", help="the camera plan: planned positions give the world frame when there are no points")
    m.add_argument("--site", help="venue file (origin and North), for points given as ned")
    m.add_argument("--max-points", type=int, default=2500, help="LED positions used (2500; fewer is faster)")
    m.add_argument("--save-lenses", metavar="DIR", help="also write the refined lens files there")
    m.add_argument("-o", "--out", default="calibration.yaml")
    m.set_defaults(fn=_cmd_room)

    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
