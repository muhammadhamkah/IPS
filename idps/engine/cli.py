"""idps-engine: run the positioning engine.

  idps-engine replay out/sim/packets.idpsraw --calib out/sim/calibration.yaml --truth out/sim/truth.csv
  idps-engine live --port 5600 --calib calibration.yaml --site venue.yaml \
      --link 1=udpout:192.168.50.101:14550 --link 2=...      (positions to each drone, by drone ID)
"""
from __future__ import annotations

import argparse
import csv
import sys
import time

import numpy as np


def cmd_replay(a) -> int:
    from idps.engine.calib import load_calibration
    from idps.engine.engine import Engine, EngineConfig
    from idps.engine.ingest import Ingest
    from idps.sim.runner import read_raw
    cams = load_calibration(a.calib, "true" if a.true_calib else "estimate")
    frame = None
    if a.site:
        from idps.plan.site import load_site
        frame = load_site(a.site).frame
    eng = Engine(cams, EngineConfig(reorder_s=a.reorder_ms / 1000), frame=frame, backend=a.backend)
    ing = Ingest()
    outputs = []
    t0 = time.perf_counter()
    for _, pkt in read_raw(a.raw):
        for f in ing.feed(pkt):
            outputs += eng.feed(f)
    outputs += eng.flush()
    wall = time.perf_counter() - t0
    span = (outputs[-1].t_ns - outputs[0].t_ns) * 1e-9 if outputs else 0.0
    lost = sum(s.lost for s in ing.stations.values())
    print(f"[{eng.backend}] {eng.stats['frames']} frames from {len(ing.stations)} stations ({lost} packets lost), "
          f"{eng.stats['epochs']} epochs, {eng.stats['tracks_started']} tracks started, {len(outputs)} positions; "
          f"{wall:.1f} s to process {span:.1f} s of data ({span / max(wall, 1e-9):.1f}x real time)")
    if a.out:
        with open(a.out, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_ns", "marker_id", "x", "y", "z", "n", "e", "d", "n_cams", "sigma_mm"])
            for o in outputs:
                ned = o.ned if o.ned is not None else [float("nan")] * 3
                w.writerow([o.t_ns, o.marker_id, *(f"{v:.5f}" for v in o.pos), *(f"{v:.5f}" for v in ned),
                            o.n_cams, f"{1000 * (o.cov.trace() / 3) ** 0.5:.3f}"])
        print(f"wrote {a.out}")
    if a.truth:
        from idps.engine.evaluate import read_truth, score
        res, unknown = score(outputs, read_truth(a.truth))
        for r in res:
            print("  " + r.line())
        if unknown:
            print(f"  {unknown} outputs with an ID that is not in the truth")
    return 0


def vel_ned(frame, v) -> list:
    """World (drawing frame, Z up) velocity -> NED."""
    return [float(v[:2] @ frame.n_hat), float(v[:2] @ frame.e_hat), -float(v[2])]


MAX_LATE_S = 0.1    # a position this much later than EKF2_EV_DELAY says is not sent: a short gap does
                    # no harm (EKF2 predicts with the IMU), a stale position pulls the drone away


def sent_position(ned, vel, age_s: float, ev_delay_ms: float | None):
    """The position to send for a fix filmed age_s ago -> (NED, too late?). PX4 takes every position
    as filmed EKF2_EV_DELAY before it arrived; a fix sent sooner or later than that (a busy PC, a
    camera frame held up) is moved along its velocity to that moment, so a moving drone is not pulled
    back or forward by the difference. ev_delay_ms None: sent as it is."""
    if ev_delay_ms is None:
        return np.asarray(ned, float), False
    lag = age_s - ev_delay_ms / 1000
    if lag > MAX_LATE_S:
        return None, True
    return np.asarray(ned, float) + np.asarray(vel, float) * max(lag, -MAX_LATE_S), False


def cmd_live(a) -> int:
    import socket
    from idps.engine.calib import load_calibration
    from idps.engine.engine import Engine, EngineConfig
    from idps.engine.ingest import Ingest
    from idps.plan.site import load_site
    from idps.show.fleet import odometry_fields
    frame = load_site(a.site).frame
    eng = Engine(load_calibration(a.calib), EngineConfig(reorder_s=a.reorder_ms / 1000), frame=frame)
    ing = Ingest()
    mavs = {}
    if a.link:
        from idps.show.cli import _connect
        mavs = _connect(a)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for size in (8 << 20, 4 << 20, 2 << 20, 1 << 20):    # macOS refuses buffers over its limit (Linux caps them)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, size)
            break
        except OSError:
            pass
    sock.bind((a.host, a.port))
    sock.settimeout(0.2)
    out_f = open(a.out, "w", newline="") if a.out else None
    w = csv.writer(out_f) if out_f else None
    if w:
        w.writerow(["t_ns", "marker_id", "n", "e", "d", "vn", "ve", "vd", "n_cams", "sigma_mm"])
    print(f"listening on udp {a.host}:{a.port}; positions to drone(s) {sorted(mavs) or 'none'}; Ctrl+C to stop")
    last_print, sent, dropped, seen, ages = time.monotonic(), 0, 0, {}, []
    try:
        while True:
            try:
                data = sock.recv(2048)
            except socket.timeout:
                data = None
            outs = []
            if data:
                for f in ing.feed(data):
                    outs += eng.feed(f)
            now = time.monotonic()
            now_ns = time.time_ns()                         # the system time base (PTP) the exposures are in
            for o in outs:
                v = vel_ned(frame, o.vel)
                seen[o.marker_id] = (now, o.n_cams)
                mav = mavs.get(o.marker_id)                 # drone ID = marker ID (D21)
                if mav is not None:
                    # PX4 stamps a position on arrival; EKF2_EV_DELAY is the exposure-to-arrival
                    # time, so the status line reports the part of it spent up to here
                    age = (now_ns - o.t_ns) * 1e-9
                    ages.append(age)
                    ned, late = sent_position(o.ned, v, age, a.ev_delay_ms)
                    if late:
                        dropped += 1
                    else:
                        mav.mav.odometry_send(**odometry_fields(ned, int(now * 1e6), 0.0, v))
                        sent += 1
                if w:
                    w.writerow([o.t_ns, o.marker_id, *(f"{x:.4f}" for x in o.ned), *(f"{x:.3f}" for x in v),
                                o.n_cams, f"{1000 * (o.cov.trace() / 3) ** 0.5:.2f}"])
            if now - last_print >= 2.0:
                live = {m: c for m, (t, c) in seen.items() if now - t < 0.5}
                lat = (f"; exposure to send (ms) median {1000 * np.median(ages):.0f}, max {1000 * max(ages):.0f}"
                       if ages else "")
                print(f"{len(ing.stations)} stations, {eng.stats['frames']} frames, {eng.n_live_tracks} tracks; "
                      f"drones with positions: {sorted(live) or 'none'}; sent {sent}{lat}"
                      + (f"; {dropped} too late, not sent" if dropped else ""), flush=True)
                last_print, ages = now, []
    except KeyboardInterrupt:
        pass
    finally:
        if out_f:
            out_f.close()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="idps-engine", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("replay", help="run the engine over a recorded packet log (e.g. from idps-sim)")
    r.add_argument("raw", help="packets.idpsraw")
    r.add_argument("--calib", required=True, help="calibration.yaml")
    r.add_argument("--truth", help="truth.csv from idps-sim: score the output against it")
    r.add_argument("--site", help="venue site file: also output NED")
    r.add_argument("-o", "--out", help="write positions to this CSV")
    r.add_argument("--true-calib", action="store_true", help="simulator only: use the exact camera poses")
    r.add_argument("--reorder-ms", type=float, default=8.0, help="wait for late packets (Wi-Fi stations: ~25)")
    r.add_argument("--backend", default="auto", choices=["auto", "cpp", "python"],
                   help="engine core: compiled C++ (default when built) or the Python reference")
    lv = sub.add_parser("live", help="run live: camera-station packets in, each drone's position out")
    lv.add_argument("--port", type=int, default=5600, help="UDP port the stations send to")
    lv.add_argument("--host", default="0.0.0.0")
    lv.add_argument("--calib", required=True)
    lv.add_argument("--site", required=True, help="venue site file (origin and North, for NED)")
    lv.add_argument("--link", action="append", help="DRONE_ID=DEVICE: send that drone its position (MAVLink ODOMETRY)")
    lv.add_argument("-o", "--out", help="also log positions to this CSV")
    lv.add_argument("--reorder-ms", type=float, default=8.0, help="wait for late packets (Wi-Fi stations: ~25)")
    lv.add_argument("--ev-delay-ms", type=float, help="the drones' EKF2_EV_DELAY: each position is moved along "
                    "its velocity to that age when sent, and not sent if over 100 ms later than that")
    a = ap.parse_args(argv)
    return {"replay": cmd_replay, "live": cmd_live}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
