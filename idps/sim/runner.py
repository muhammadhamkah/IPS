"""Run a scenario: step every station's frames in true-time order, emit
FRAME_OBS + HEALTH packets to a sink, and record ground truth."""
from __future__ import annotations

import csv
import heapq
import math
import socket
import struct
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from idps.protocol import markercode
from idps.protocol import packets as P
from idps.sim.camera import SimStation
from idps.sim.scenario import Scenario
from idps.sim.trajectory import body_axes, tilt_deg

FAST_EPOCH_NS = 1_700_000_000_000_000_000      # fixed epoch in fast mode, so output is reproducible


# ---------------------------------------------------------------------- sinks
class MemorySink:
    def __init__(self):
        self.packets: list[tuple[float, bytes]] = []

    def send(self, t_send: float, data: bytes):
        self.packets.append((t_send, data))

    def close(self):
        pass


class FileSink:
    """Raw packet log: repeated [u64 send time ns since epoch][u32 length][packet bytes].
    The engine's replay reads the same format."""
    REC = struct.Struct("<QI")

    def __init__(self, path, epoch_ns: int):
        self.f = open(path, "wb")
        self.f.write(b"IDPSRAW1")
        self.epoch = epoch_ns

    def send(self, t_send: float, data: bytes):
        self.f.write(self.REC.pack(self.epoch + int(t_send * 1e9), len(data)) + data)

    def close(self):
        self.f.close()


def read_raw(path):
    with open(path, "rb") as f:
        if f.read(8) != b"IDPSRAW1":
            raise ValueError("not an IDPS raw packet log")
        while hdr := f.read(FileSink.REC.size):
            t, n = FileSink.REC.unpack(hdr)
            yield t, f.read(n)


class UdpSink:
    """Sends to host:port. In real-time mode packets leave at their simulated
    send time (capture + processing + network latency) against the wall clock."""

    def __init__(self, host: str, port: int, realtime: bool):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.realtime = realtime
        self.wall0 = time.monotonic()

    def send(self, t_send: float, data: bytes):
        if self.realtime:
            delay = self.wall0 + t_send - time.monotonic()
            if delay > 0:
                time.sleep(delay)
        self.sock.sendto(data, self.addr)

    def close(self):
        self.sock.close()


class MultiSink:
    def __init__(self, sinks):
        self.sinks = sinks

    def send(self, t, data):
        for s in self.sinks:
            s.send(t, data)

    def close(self):
        for s in self.sinks:
            s.close()


# ---------------------------------------------------------------------- run
@dataclass
class RunStats:
    frames: int = 0
    packets: int = 0
    packets_lost: int = 0
    frames_dropped: int = 0
    markers_sent: int = 0
    true_detections: int = 0
    spurious: int = 0
    per_station: dict = field(default_factory=dict)


class Simulation:
    def __init__(self, sc: Scenario, epoch_ns: int | None = None):
        self.sc = sc
        self.epoch_ns = FAST_EPOCH_NS if epoch_ns is None else epoch_ns
        base = np.random.default_rng(sc.seed)
        mk = sc.markers
        self.leds = [markercode.MarkerLed(d.marker_id, mk.chip_rate_hz, base.normal(0, mk.clock_ppm),
                                          base.uniform(0, markercode.CHIPS_PER_FRAME / mk.chip_rate_hz), mk.high, mk.low)
                     for d in sc.drones]
        self.stations = [SimStation(sid, c.position_m, c.R, c.lens_mm, sc.camera_model, sc.timing, sc.markers,
                                    np.random.default_rng([sc.seed, sid]), sc.duration_s)
                         for sid, c in zip(sc.station_ids, sc.cameras)]
        if sc.timing.clock_profile:
            from idps.sim.clocksync import simulate_clocks
            traces = simulate_clocks(len(self.stations), sc.duration_s, sc.timing.clock_profile, seed=sc.seed,
                                     station_ids=[st.id for st in self.stations],
                                     start_synced=sc.timing.clock_start != "power_on")
            for st, tr in zip(self.stations, traces):
                st.clock = tr
        self.net_rng = np.random.default_rng([sc.seed, 999])
        self.marker_drone = np.arange(len(sc.drones))

    # world state ---------------------------------------------------------
    STATE_DT = 1e-3            # drone state is precomputed on a 1 kHz grid, in 1 s blocks
    ATTITUDE_TAU_S = 0.08      # tilt response time constant
    ATTITUDE_HISTORY_S = 0.6   # lag history computed before each block (>> tau, so blocks join smoothly)

    def _block(self, b: int):
        """Positions/markers/up axes/tilt for all drones on block b's time grid."""
        if getattr(self, "_blk_id", None) == b:
            return self._blk
        sc, dt = self.sc, self.STATE_DT
        n = int(round(1.0 / dt))
        h = int(round(self.ATTITUDE_HISTORY_S / dt))
        tg = b + np.arange(-1 - h, n + 2) * dt             # history for the attitude lag, +1 sample each side
        D = len(sc.drones)
        P = np.stack([d.trajectory.position(tg) for d in sc.drones])            # (D, H+N+3, 3)
        A = np.zeros_like(P)
        A[:, 1:-1] = (P[:, 2:] - 2 * P[:, 1:-1] + P[:, :-2]) / (dt * dt)
        A[:, 0], A[:, -1] = A[:, 1], A[:, -2]
        # A drone tilts to accelerate, but its attitude cannot change instantly: the tilt follows
        # the commanded acceleration through a first-order lag (time constant ATTITUDE_TAU_S).
        # Without it, a waypoint path's step in acceleration would flip the tilt in one step and
        # jump an offset marker sideways.
        a = dt / (self.ATTITUDE_TAU_S + dt)
        for i in range(1, A.shape[1]):
            A[:, i] = A[:, i - 1] + a * (A[:, i] - A[:, i - 1])
        P, A, tg = P[:, h:], A[:, h:], tg[h:]
        M = np.empty_like(P)
        U = np.empty_like(P)
        tilt = np.empty(P.shape[:2])
        for k, d in enumerate(sc.drones):
            x, y, z = body_axes(A[k], d.yaw_deg)
            o = d.marker_offset_m
            M[k] = P[k] + o[0] * x + o[1] * y + o[2] * z
            U[k] = z
            tilt[k] = tilt_deg(A[k])
        self._blk_id, self._blk = b, (tg[0], P, M, U, tilt)
        return self._blk

    def state(self, t: float):
        """Drone centres (D, 3), marker positions (D, 3), body up axes (D, 3), tilt (D,)."""
        if not self.sc.drones:
            z = np.zeros((0, 3))
            return z, z, z, np.zeros(0)
        t0, P, M, U, tilt = self._block(int(np.floor(t)))
        x = (t - t0) / self.STATE_DT
        i = min(int(x), P.shape[1] - 2)
        f = x - i
        lerp = lambda A: A[:, i] * (1 - f) + A[:, i + 1] * f    # noqa: E731
        u = lerp(U)
        return lerp(P), lerp(M), u / np.linalg.norm(u, axis=1, keepdims=True), lerp(tilt)

    def led_levels(self, t0: float, t1: float) -> np.ndarray:
        """Mean drive level of every LED over [t0, t1] (vectorised over LEDs)."""
        if not self.leds:
            return np.zeros(0)
        if not hasattr(self, "_led_tab"):
            self._led_tab = (np.stack([l.pattern for l in self.leds]), np.array([l.rate for l in self.leds]),
                             np.array([l.phase for l in self.leds]), np.array([l.high for l in self.leds]),
                             np.array([l.low for l in self.leds]))
        pat, rate, ph, hi, lo = self._led_tab
        n = pat.shape[1]
        a, b = (t0 + ph) * rate, (t1 + ph) * rate
        ia, ib = np.floor(a).astype(np.int64), np.floor(b).astype(np.int64)
        rows = np.arange(len(pat))
        la = np.where(pat[rows, ia % n] > 0, hi, lo)
        lb = np.where(pat[rows, ib % n] > 0, hi, lo)
        fa = np.where(ia == ib, 1.0, (ib - a) / np.maximum(b - a, 1e-12))
        return la * fa + lb * (1 - fa)

    # main loop -----------------------------------------------------------
    def run(self, sink, progress=None, truth_rate_hz: float = 120.0, truth_path=None) -> RunStats:
        sc = self.sc
        stats = RunStats()
        net = sc.network
        heap = [(st.frame_time(0), i, 0) for i, st in enumerate(self.stations)]
        heapq.heapify(heap)
        seq = {st.id: 0 for st in self.stations}
        next_health = {st.id: 0.5 + 0.01 * i for i, st in enumerate(self.stations)}
        out_queue: list = []          # (t_send, counter, bytes) - reorders by network delay
        counter = 0
        last_progress = 0.0
        truth = TruthWriter(truth_path, self, truth_rate_hz) if truth_path else None

        def emit(t_send, data):
            nonlocal counter
            if self.net_rng.random() < net.loss:
                stats.packets_lost += 1
                return
            delay = max(0.0, self.net_rng.normal(net.latency_ms, net.jitter_ms)) * 1e-3
            heapq.heappush(out_queue, (t_send + delay, counter, data))
            counter += 1

        def flush(until):
            while out_queue and out_queue[0][0] <= until:
                t_send, _, data = heapq.heappop(out_queue)
                sink.send(t_send, data)
                stats.packets += 1

        while heap:
            t, i, k = heapq.heappop(heap)
            st = self.stations[i]
            if k + 1 < st.n_frames:
                heapq.heappush(heap, (st.frame_time(k + 1), i, k + 1))
            if t > sc.duration_s:
                continue
            if truth:
                truth.advance(t)
            stats.frames += 1
            ps = stats.per_station.setdefault(st.id, {"frames": 0, "dropped": 0, "markers": 0, "true": 0})
            ps["frames"] += 1
            if st.rng.random() < sc.camera_model.frame_drop_prob:
                st.dropped += 1
                stats.frames_dropped += 1
                ps["dropped"] += 1
                continue
            centres, mpos, ups, _ = self.state(t)
            half = st.exposure_s / 2
            levels = self.led_levels(t - half, t + half)
            markers, tr = st.observe(t, mpos, ups, levels, centres, self.marker_drone)
            n_true = sum(1 for x in tr if x[0] >= 0)
            stats.markers_sent += len(markers)
            stats.true_detections += n_true
            stats.spurious += len(markers) - n_true
            ps["markers"] += len(markers)
            ps["true"] += n_true
            obs = P.FrameObs(st.id, k, st.timestamp_ns(t, self.epoch_ns), int(st.exposure_s * 1e9), markers,
                             proc_us=int(800 + 40 * len(markers) + st.rng.exponential(200)),
                             sync_state=st.sync_state, threshold=sc.camera_model.threshold_dn,
                             drops_since_last=st.dropped, flags=P.FLAG_SIMULATED)
            st.dropped = 0
            t_send = t + half + sc.camera_model.proc_latency_ms * 1e-3
            pkts = P.encode_frame(obs, seq[st.id])
            seq[st.id] += len(pkts)
            for pkt in pkts:
                emit(t_send, pkt)
            if t >= next_health[st.id]:
                next_health[st.id] += 1.0
                emit(t_send, P.encode_health(st.id, seq[st.id], self.health(st, ps, t), P.FLAG_SIMULATED))
                seq[st.id] += 1
            flush(t)
            if progress and t - last_progress >= 1.0:
                last_progress = t
                progress(t, stats)
        flush(math.inf)
        if truth:
            truth.advance(sc.duration_s + 1e-9)
            truth.close()
        sink.close()
        return stats

    @staticmethod
    def _reported_offset_ns(st: SimStation, t: float) -> int:
        """What the station's sync daemon reports (its last measured offset), not the truth."""
        if st.clock is not None:
            m = st.clock.measured_at(t)
            return int(m * 1e9) if np.isfinite(m) else 0
        return int(st.clock_offset * 1e9)

    def health(self, st: SimStation, ps: dict, t: float) -> dict:
        cam = self.sc.cameras[self.sc.station_ids.index(st.id)]
        return {"fw": "idps-sim", "uptime_s": round(t, 1), "fps_measured": round(1.0 / st.period, 2),
                "frames_captured": ps["frames"], "frames_dropped": ps["dropped"],
                "proc_us_p50": 1400, "proc_us_p99": 2900, "cpu_temp_c": 55.0, "throttled": 0,
                "ptp_state": {"ptp": "SLAVE", "hw_trigger": "TRIGGER", "free_run": "NONE"}[self.sc.timing.sync],
                "ptp_offset_ns": self._reported_offset_ns(st, t), "exposure_ns": int(st.exposure_s * 1e9),
                "threshold": self.sc.camera_model.threshold_dn,
                "gimbal": {"state": "holding", "pan_deg": cam.pan_deg, "tilt_deg": cam.tilt_deg},
                "plan_camera_id": cam.id, "calib_version": "sim"}


class TruthWriter:
    """Ground truth CSV at a fixed rate: time, drone, marker id, reference point, marker, tilt, LED level."""

    def __init__(self, path, sim: Simulation, rate_hz: float):
        self.sim, self.dt = sim, 1.0 / rate_hz
        self.t_next = 0.0
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.f.write(f"# idps-sim ground truth; t_s is true time; t_ns = epoch_ns + t_s*1e9; epoch_ns = {sim.epoch_ns}\n")
        self.w.writerow(["t_s", "drone_id", "marker_id", "x", "y", "z", "marker_x", "marker_y", "marker_z",
                         "tilt_deg", "led_level"])

    def advance(self, t):
        while self.t_next <= t:
            tt = self.t_next
            centres, mpos, _, tilt = self.sim.state(tt)
            for d, c, m, tl, led in zip(self.sim.sc.drones, centres, mpos, tilt, self.sim.leds):
                self.w.writerow([f"{tt:.6f}", d.id, d.marker_id, *(f"{v:.5f}" for v in c), *(f"{v:.5f}" for v in m),
                                 f"{tl:.2f}", f"{float(led.level(tt)):.2f}"])
            self.t_next += self.dt

    def close(self):
        self.f.close()


def write_calibration(sim: Simulation, path) -> None:
    """True camera parameters and an 'estimated' calibration perturbed by the
    scenario's calibration error (what a real calibration would hand the engine)."""
    sc = sim.sc
    rng = np.random.default_rng([sc.seed, 4242])
    m = sc.camera_model
    out = {"format": "idps-calibration/1", "source": "idps-sim", "scenario": sc.name, "epoch_ns": sim.epoch_ns,
           "sensor": m.sensor.name, "width_px": m.sensor.width_px, "height_px": m.sensor.height_px, "cameras": []}
    for st, cam in zip(sim.stations, sc.cameras):
        rot_err = rng.normal(0, sc.calib_rot_mrad * 1e-3, 3)
        pos_err = rng.normal(0, sc.calib_pos_mm * 1e-3, 3)
        R_est = _rotvec_to_matrix(rot_err) @ st.R
        intr = {"f_px": st.f, "cx": st.cx, "cy": st.cy, "distortion": list(m.distortion)}
        out["cameras"].append({
            "station_id": st.id, "plan_camera_id": cam.id, **intr,
            "true": {"position_m": st.pos.tolist(), "R_world_to_cam": st.R.tolist(),
                     "clock_offset_s": float(st.clock.at(0.0)) if st.clock is not None else st.clock_offset,
                     "clock_drift": float(st.clock.rate[0]) if st.clock is not None else st.clock_drift},
            "estimate": {"position_m": (st.pos + pos_err).tolist(), "R_world_to_cam": R_est.tolist()},
        })
    Path(path).write_text(yaml.safe_dump(out, sort_keys=False, default_flow_style=None, width=120))


def _rotvec_to_matrix(r):
    th = float(np.linalg.norm(r))
    if th < 1e-15:
        return np.eye(3)
    k = r / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
