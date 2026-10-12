"""idps-sim run SCENARIO [--dest HOST:PORT] [--realtime] | idps-sim listen [--port P]
| idps-sim clocks [--stations N] [--profile P ...] | idps-sim px4 SCENARIO --manifest M --dest HOST:PORT"""
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
    files = "packets.idpsraw, truth.csv, calibration.yaml"
    if any(st.clock is not None for st in sim.stations):
        _write_clock_csv(out / "clocks.csv", [st.clock for st in sim.stations], sc.duration_s)
        files += ", clocks.csv"
    print(f"wrote {out}/{files}")
    return 0


def _write_clock_csv(path, traces, duration_s, rate_hz=10.0):
    """True clock error and the offset the sync daemon reports, per station, at rate_hz."""
    import csv

    import numpy as np
    t = np.arange(0, duration_s, 1.0 / rate_hz)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "station_id", "clock_error_us", "reported_offset_us"])
        for tr in traces:
            err = tr.at(t)
            for tt, e in zip(t, err):
                m = tr.measured_at(tt)
                w.writerow([f"{tt:.2f}", tr.station_id, f"{e * 1e6:.2f}", "" if not np.isfinite(m) else f"{m * 1e6:.2f}"])


def cmd_clocks(a) -> int:
    """Simulate clock synchronisation for N stations under each profile and compare."""
    import numpy as np

    from idps.core.noise import PRESETS
    from idps.sim.clocksync import PROFILES, simulate_clocks, skew_series, summarise
    names = a.profile or [p for p in PROFILES if p != "free_run"]
    for n in names:
        if n not in PROFILES:
            print(f"unknown profile {n!r}; known: {', '.join(PROFILES)}", file=sys.stderr)
            return 2
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    print(f"{a.stations} stations, {a.duration:g} s from power-on, drone speed {a.speed:g} m/s\n")
    hdr = (f"{'profile':22s} {'settled':>8s} {'skew p50':>9s} {'p95':>8s} {'p99':>8s} {'max':>9s} {'sigma':>8s} "
           f"{'error @':>8s}")
    print(hdr)
    print(f"{'':22s} {'(s)':>8s} {'(us)':>9s} {'(us)':>8s} {'(us)':>8s} {'(us)':>9s} {'(us)':>8s} "
          f"{f'{a.speed:g} m/s':>8s}")
    rows, all_traces = [], {}
    for n in names:
        tr = simulate_clocks(a.stations, a.duration, n, seed=a.seed)
        all_traces[n] = tr
        st = summarise(tr, a.duration, n, speed_mps=a.speed)
        rows.append(st)
        settled = "never" if st.converge_s is None else f"{st.converge_s:.0f}"
        print(f"{n:22s} {settled:>8s} {st.skew_p50_us:9.1f} {st.skew_p95_us:8.1f} {st.skew_p99_us:8.1f} "
              f"{st.skew_max_us:9.1f} {st.sigma_us:8.1f} {st.pos_err_mm_at_5mps * a.speed / 5:6.2f} mm")
    print("\nskew = difference between two stations' clocks (what triangulation sees); error = the timing "
          "part of the position error (p95) for a drone at that speed.")
    print("Planner error budget presets assume timing sigma: " + ", ".join(
        f"{k} {v.sigma_t_s * 1e6:g} us" for k, v in PRESETS.items()))
    for n, p in ((n, PROFILES[n]) for n in names):
        print(f"  {n}: {p.description}")
    import csv
    with open(out / "clocks_summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["profile", "settled_s", "skew_p50_us", "skew_p95_us", "skew_p99_us", "skew_max_us", "sigma_us",
                    f"pos_err_mm_at_{a.speed:g}mps"])
        for r in rows:
            w.writerow([r.profile, "" if r.converge_s is None else f"{r.converge_s:.1f}", f"{r.skew_p50_us:.2f}",
                        f"{r.skew_p95_us:.2f}", f"{r.skew_p99_us:.2f}", f"{r.skew_max_us:.2f}", f"{r.sigma_us:.2f}",
                        f"{r.pos_err_mm_at_5mps * a.speed / 5:.3f}"])
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(len(names), 1, figsize=(9, 2.4 * len(names)), squeeze=False)
        t = np.linspace(0, a.duration, 4000)
        for ax, n, r in zip(axs[:, 0], names, rows):
            S = skew_series(all_traces[n], t) * 1e6
            for srow in S[:12]:
                ax.plot(t, srow, lw=0.6)
            lim = max(3 * r.skew_p99_us, 1.0)
            ax.set_ylim(-lim, lim)
            ax.set_title(f"{n}: station clock minus the mean of all stations", fontsize=9, loc="left")
            ax.set_ylabel("us")
        axs[-1, 0].set_xlabel("time since power-on (s)")
        fig.tight_layout()
        fig.savefig(out / "clocks.png", dpi=90)
        plt.close(fig)
        extra = ", clocks.png"
    except ImportError:
        extra = ""
    print(f"\nwrote {out}/clocks_summary.csv{extra}")
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
    c = sub.add_parser("clocks", help="simulate clock synchronisation between stations and compare options")
    c.add_argument("--stations", type=int, default=16)
    c.add_argument("--duration", type=float, default=900.0, help="seconds from power-on")
    c.add_argument("--profile", action="append", help="clock profile (repeat to compare; default: all)")
    c.add_argument("--speed", type=float, default=5.0, help="drone speed for the error estimate (m/s)")
    c.add_argument("--seed", type=int, default=1)
    c.add_argument("-o", "--out", default="out/clocks")
    x = sub.add_parser("px4", help="closed loop: film the PX4 SITL drones (px4/sitl/run_sitl.sh) in real time "
                                   "and stream to the engine")
    x.add_argument("scenario", help="scenario for the cameras and models (its own drones are replaced by the show's)")
    x.add_argument("--manifest", required=True, help="show manifest.json (drone IDs, home spots, venue frame)")
    x.add_argument("--drones", type=int, help="film only the first N drones (one SITL instance each)")
    x.add_argument("--dest", default="127.0.0.1:5600", help="engine address (idps-engine live --port)")
    x.add_argument("--duration", type=float, default=120.0, help="seconds to run")
    x.add_argument("-o", "--out", default="out/loop", help="folder for calibration.yaml and truth.csv")
    a = ap.parse_args(argv)
    if a.cmd == "px4":
        from idps.sim.live import run_px4_loop
        return run_px4_loop(a)
    return {"run": cmd_run, "listen": cmd_listen, "clocks": cmd_clocks}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
