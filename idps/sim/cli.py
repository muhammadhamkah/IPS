"""idps-sim run SCENARIO [--dest HOST:PORT] [--realtime] | idps-sim listen [--port P]"""
from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

from idps.protocol import packets as P


def cmd_run(a) -> int:
    from idps.sim.runner import FileSink, MultiSink, Simulation, UdpSink, write_calibration
    from idps.sim.scenario import load_scenario
    sc = load_scenario(a.scenario)
    if a.duration:
        sc.duration_s = a.duration
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    epoch = time.time_ns() if a.realtime else None
    sim = Simulation(sc, epoch_ns=epoch)
    if a.duration:                                   # stations size their frame count from the duration
        for st in sim.stations:
            st.n_frames = int((sc.duration_s - st.phase) / st.period) + 1
    sinks = [FileSink(out / "packets.idpsraw", sim.epoch_ns)]
    if a.dest:
        host, port = a.dest.rsplit(":", 1)
        sinks.append(UdpSink(host, int(port), a.realtime))
    write_calibration(sim, out / "calibration.yaml")
    print(f"scenario '{sc.name}': {len(sim.stations)} stations, {len(sc.drones)} drones, {sc.duration_s:g} s, "
          f"sync={sc.timing.sync}, sensor={sc.camera_model.sensor.name} @ {sc.camera_model.fps:g} fps"
          + (f", streaming to {a.dest}" + (" in real time" if a.realtime else "") if a.dest else ""))

    def progress(t, s):
        print(f"  t={t:6.1f} s  frames {s.frames:8d}  packets {s.packets:8d}  markers/frame "
              f"{s.markers_sent / max(1, s.frames):5.2f}", flush=True)

    t0 = time.time()
    stats = sim.run(MultiSink(sinks), progress=progress if not a.quiet else None,
                    truth_path=out / "truth.csv")
    seen = stats.true_detections / max(1, stats.frames)
    print(f"done in {time.time() - t0:.1f} s: {stats.frames} frames ({stats.frames_dropped} dropped), "
          f"{stats.packets} packets sent ({stats.packets_lost} lost), {seen:.2f} true markers per frame, "
          f"{stats.spurious} spurious blobs")
    print(f"wrote {out}/packets.idpsraw, truth.csv, calibration.yaml")
    return 0


def cmd_listen(a) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((a.host, a.port))
    sock.settimeout(0.2)
    print(f"listening on {a.host}:{a.port} (Ctrl+C to stop)")
    st: dict[int, dict] = {}
    t_end = time.time() + a.seconds if a.seconds else None
    t_report = time.time() + 1.0
    bad = 0
    try:
        while t_end is None or time.time() < t_end:
            try:
                data, _ = sock.recvfrom(65536)
            except socket.timeout:
                data = None
            now_ns = time.time_ns()
            if data:
                try:
                    h, body = P.decode(data)
                except P.ProtocolError:
                    bad += 1
                    continue
                s = st.setdefault(h.station_id, {"frames": 0, "markers": 0, "last_seq": None, "lost": 0,
                                                 "lat": [], "health": None, "drops": 0})
                if s["last_seq"] is not None:
                    gap = (h.seq - s["last_seq"] - 1) & 0xFFFFFFFF
                    s["lost"] += gap if gap < 1000 else 0
                s["last_seq"] = h.seq
                if h.msg_type == P.MSG_FRAME_OBS:
                    s["frames"] += body.part_index == 0
                    s["markers"] += len(body.markers)
                    s["drops"] += body.drops_since_last
                    s["lat"].append((now_ns - body.t_mid_ns) / 1e6)
                else:
                    s["health"] = body
            if time.time() >= t_report:
                t_report += 1.0
                _report(st, bad)
    except KeyboardInterrupt:
        pass
    _report(st, bad)
    return 0


def _report(st, bad):
    if not st:
        print("  (no packets)")
        return
    print(f"{'station':>8} {'fps':>6} {'markers/f':>9} {'lost pkts':>9} {'drops':>6} {'latency ms (p50)':>16}"
          + (f"   {bad} malformed" if bad else ""))
    for sid in sorted(st):
        s = st[sid]
        lat = sorted(s["lat"])
        p50 = f"{lat[len(lat) // 2]:.1f}" if lat else "-"
        print(f"{sid:>8} {s['frames']:>6} {s['markers'] / max(1, s['frames']):>9.2f} {s['lost']:>9} {s['drops']:>6} "
              f"{p50:>16}")
        s.update(frames=0, markers=0, lat=[])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="idps-sim", description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="simulate camera stations for a scenario")
    r.add_argument("scenario")
    r.add_argument("-o", "--out", default="out/sim", help="folder for packets, truth and calibration")
    r.add_argument("--dest", help="also stream UDP to HOST:PORT (engine default 127.0.0.1:47600)")
    r.add_argument("--realtime", action="store_true", help="send packets at real-time pace (with --dest)")
    r.add_argument("--duration", type=float, help="override the scenario duration (s)")
    r.add_argument("-q", "--quiet", action="store_true")
    li = sub.add_parser("listen", help="receive packets and print per-station statistics")
    li.add_argument("--host", default="127.0.0.1")
    li.add_argument("--port", type=int, default=47600)
    li.add_argument("--seconds", type=float)
    a = ap.parse_args(argv)
    return cmd_run(a) if a.cmd == "run" else cmd_listen(a)


if __name__ == "__main__":
    sys.exit(main())
