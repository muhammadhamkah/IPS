"""Clock synchronisation between camera stations, simulated message by message.

Every station (a Raspberry Pi + OV9281) timestamps its frames with its own system
clock. Triangulation needs those clocks to agree: a drone moving at v m/s seen by two
cameras whose clocks differ by dt lands about v * dt off. What matters is the *skew
between stations*; an offset shared by all stations only adds latency.

Model, per station:

* **Oscillator.** A crystal with a fixed frequency error (Pi boards: tens of ppm) plus a
  slow random-walk wander (temperature). Free running, the clock error grows by
  ppm x time: 20 ppm is 1.2 ms per minute.
* **Two-way exchange** (PTP Sync / Delay_Req, or NTP), every `sync_interval_s`:
  t1 master send, t2 slave receive, t3 slave send, t4 master receive. Each timestamp has
  noise from where it is taken: *software* timestamps on a Pi 3B: its Ethernet port is wired
  inside the board to the CPU through a USB 2.0 chip (LAN9514, 100 Mbit), so they carry USB polling
  quantisation (125 us microframes), interrupt latency and spikes when the CPU is busy
  with image processing; *hardware* timestamps (PHY/NIC, e.g. Pi 5, CM4, Intel i210)
  are good to tens of ns. The switch adds queueing delay variation. A fixed, unknown
  **path asymmetry** (send and receive paths through that chip differ) biases
  the measured offset by half of it, and no protocol can see that.
* **Servo** like linuxptp's PI servo: the first measurement steps the clock, then each
  measurement nudges phase (kp) and frequency (ki). The path delay goes through a
  moving median, like `delay_filter`.

The grandmaster is whatever the stations follow: the PC, one Pi, or a dedicated box.
Only the stations' agreement with each other matters for tracking, so a free-running
master is fine (macOS cannot run ptp4l; make a Pi or a small Linux box the master).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

US = 1e-6
FREQ_FIT_SAMPLES = 8          # start-up: samples used to measure the oscillator's frequency error


@dataclass
class TimestampNoise:
    """Error of one software or hardware timestamp (seconds)."""
    sigma_us: float = 0.0                 # gaussian part
    quantum_us: float = 0.0               # uniform 0..quantum (USB polling), mean subtracted
    spike_prob: float = 0.0               # chance of a late timestamp (IRQ latency, busy CPU)
    spike_mean_us: float = 0.0            # exponential spike size

    def draw(self, rng, n=None):
        e = rng.normal(0, self.sigma_us, n)
        if self.quantum_us:
            e = e + rng.uniform(0, self.quantum_us, n) - self.quantum_us / 2
        if self.spike_prob:
            hit = rng.random(n) < self.spike_prob
            e = e + np.where(hit, rng.exponential(max(self.spike_mean_us, 1e-9), n), 0.0)
        return e * US


@dataclass
class ClockProfile:
    """How the stations' clocks are kept in step."""
    name: str
    method: str                            # "none" | "ptp" | "ntp"
    sync_interval_s: float = 1.0
    slave_ts: TimestampNoise = field(default_factory=TimestampNoise)
    master_ts: TimestampNoise = field(default_factory=TimestampNoise)
    network_pdv_us: float = 2.0            # switch / link delay variation (1 sigma, one way)
    network_delay_us: float = 60.0         # one-way delay through the switch and adapters
    asymmetry_us: float = 0.0              # per-station fixed send/receive path asymmetry (1 sigma)
    kp: float = 0.7                        # PI servo gains (linuxptp: 0.7/0.3 hardware, 0.1/0.001 software)
    ki: float = 0.3
    delay_filter: int = 10                 # moving median of the measured path delay
    offset_filter: int = 1                 # NTP-style: best (lowest-delay) of the last N samples
    osc_ppm: float = 20.0                  # crystal frequency error (1 sigma)
    wander_ppb_per_sqrt_s: float = 1.0     # random-walk frequency wander (temperature)
    description: str = ""


PROFILES: dict[str, ClockProfile] = {
    # The planned prototype: Pi 3B, wired, PTP with software timestamps (ptp4l -S), 4 Hz sync.
    "pi3b_ptp_sw": ClockProfile(
        "pi3b_ptp_sw", "ptp", sync_interval_s=0.25,
        slave_ts=TimestampNoise(sigma_us=6, quantum_us=125, spike_prob=0.03, spike_mean_us=250),
        master_ts=TimestampNoise(sigma_us=6, quantum_us=125, spike_prob=0.01, spike_mean_us=150),
        network_pdv_us=3, asymmetry_us=15, kp=0.1, ki=0.001, wander_ppb_per_sqrt_s=2,
        description="Raspberry Pi 3B, Ethernet (100 Mbit port) into a gigabit switch, ptp4l with software "
                    "timestamps, a Pi as grandmaster"),
    # Same Pis, but the master has hardware timestamping (a Linux PC with an Intel i210, a CM4 / Pi 5).
    "pi3b_ptp_sw_hwmaster": ClockProfile(
        "pi3b_ptp_sw_hwmaster", "ptp", sync_interval_s=0.25,
        slave_ts=TimestampNoise(sigma_us=6, quantum_us=125, spike_prob=0.03, spike_mean_us=250),
        master_ts=TimestampNoise(sigma_us=0.05),
        network_pdv_us=3, asymmetry_us=15, kp=0.1, ki=0.001, wander_ppb_per_sqrt_s=2,
        description="Pi 3B stations with software timestamps; grandmaster with hardware timestamps"),
    # chrony / NTP over the same wiring: slower polling, best-of-N filtering.
    "pi3b_ntp": ClockProfile(
        "pi3b_ntp", "ntp", sync_interval_s=2.0,
        slave_ts=TimestampNoise(sigma_us=6, quantum_us=125, spike_prob=0.03, spike_mean_us=250),
        master_ts=TimestampNoise(sigma_us=6, quantum_us=125, spike_prob=0.01, spike_mean_us=150),
        network_pdv_us=3, asymmetry_us=15, kp=0.05, ki=0.0005, offset_filter=8, wander_ppb_per_sqrt_s=2,
        description="Pi 3B, chrony (NTP) on the wired network, 2 s polling"),
    # Hardware PTP end to end (Pi 5 / CM4 with PHY timestamping): the reference for an upgrade.
    "pi5_ptp_hw": ClockProfile(
        "pi5_ptp_hw", "ptp", sync_interval_s=0.125,
        slave_ts=TimestampNoise(sigma_us=0.02), master_ts=TimestampNoise(sigma_us=0.02),
        network_pdv_us=0.5, asymmetry_us=0.05, kp=0.7, ki=0.3, wander_ppb_per_sqrt_s=2,
        description="Pi 5 / CM4 stations with hardware PTP timestamps"),
    # Wi-Fi (2.4 GHz, Pi 3B): software timestamps through the radio, retries and power save.
    "pi3b_wifi_ntp": ClockProfile(
        "pi3b_wifi_ntp", "ntp", sync_interval_s=2.0,
        slave_ts=TimestampNoise(sigma_us=150, quantum_us=0, spike_prob=0.08, spike_mean_us=3000),
        master_ts=TimestampNoise(sigma_us=6, quantum_us=125, spike_prob=0.01, spike_mean_us=150),
        network_pdv_us=400, asymmetry_us=200, kp=0.05, ki=0.0005, offset_filter=8, wander_ppb_per_sqrt_s=2,
        description="Pi 3B on 2.4 GHz Wi-Fi with chrony"),
    "free_run": ClockProfile("free_run", "none", description="no synchronisation at all"),
}


@dataclass
class ClockTrace:
    """One station's clock error over time: local clock minus true time (seconds)."""
    station_id: int
    t: np.ndarray                          # event times (true time)
    offset: np.ndarray                     # true clock error at those times (after any correction)
    rate: np.ndarray                       # clock error rate (s/s) from each event to the next
    measured: np.ndarray                   # offset the servo measured at each event (what ptp4l prints)

    def at(self, t):
        """Clock error at true time(s) t."""
        t = np.asarray(t, float)
        k = np.clip(np.searchsorted(self.t, t, side="right") - 1, 0, len(self.t) - 1)
        return self.offset[k] + self.rate[k] * (t - self.t[k])

    def measured_at(self, t: float) -> float:
        k = int(np.clip(np.searchsorted(self.t, t, side="right") - 1, 0, len(self.t) - 1))
        return float(self.measured[k])


def simulate_clocks(n_stations: int, duration_s: float, profile: ClockProfile | str, seed: int = 1,
                    station_ids=None, start_synced: bool = False) -> list[ClockTrace]:
    """Simulate every station's clock against a grandmaster for duration_s.
    The master's own clock is true time (only agreement between stations matters).
    start_synced: begin from a converged state (a running system), not from power-on."""
    pf = PROFILES[profile] if isinstance(profile, str) else profile
    ids = list(station_ids) if station_ids is not None else list(range(1, n_stations + 1))
    out = []
    for sid in ids:
        rng = np.random.default_rng([seed, 7177, sid])
        f0 = rng.normal(0, pf.osc_ppm) * 1e-6
        theta0 = rng.uniform(-0.5, 0.5) if not start_synced else rng.normal(0, 20 * US)
        if pf.method == "none":
            t = np.array([0.0])
            out.append(ClockTrace(sid, t, np.array([theta0]), np.array([f0]), np.array([np.nan])))
            continue
        T = pf.sync_interval_s
        n = int(math.ceil(duration_s / T)) + 2
        t = np.arange(n) * T + rng.uniform(0, T)
        t[0] = 0.0
        asym = rng.normal(0, pf.asymmetry_us) * US
        wander = np.cumsum(rng.normal(0, pf.wander_ppb_per_sqrt_s * 1e-9 * math.sqrt(T), n))
        offs, rates, meas = np.empty(n), np.empty(n), np.full(n, np.nan)
        theta, adj = theta0, 0.0
        if start_synced:
            adj = f0                                       # servo already locked to the oscillator
        drift_int = adj
        delays: list[float] = []
        samples: list[tuple[float, float]] = []            # (measured offset, measured delay) for NTP filter
        stepped = start_synced
        d0 = pf.network_delay_us * US
        for k in range(n):
            if k:
                theta += rates[k - 1] * (t[k] - t[k - 1])
            # one two-way exchange: forward path carries +asym/2, reverse -asym/2
            df = d0 + asym / 2 + abs(rng.normal(0, pf.network_pdv_us * US))
            dr = d0 - asym / 2 + abs(rng.normal(0, pf.network_pdv_us * US))
            e1, e4 = pf.master_ts.draw(rng, 2)
            e2, e3 = pf.slave_ts.draw(rng, 2)
            # t2 - t1 = df + theta + e2 - e1 ;  t4 - t3 = dr - theta + e4 - e3
            fwd = df + theta + e2 - e1
            rev = dr - theta + e4 - e3
            delay = (fwd + rev) / 2
            delays.append(delay)
            if len(delays) > pf.delay_filter:
                delays.pop(0)
            raw = fwd - float(np.median(delays))
            m = raw
            if pf.offset_filter > 1 and stepped:           # NTP: trust the sample with the lowest delay
                samples.append([fwd - delay, delay, t[k]])
                samples = samples[-pf.offset_filter:]
                best = min(samples, key=lambda q: q[1])
                m = best[0]
            meas[k] = m
            if not stepped:
                # Start-up, like ptp4l/chrony: step to the first offset, then measure the
                # oscillator's frequency error over a few samples (a line fit, robust to the
                # noisy software timestamps) and step again with it, then lock the PI servo.
                if k == 0:
                    theta -= m
                    m = 0.0
                    fit = []
                    delays.clear()
                else:
                    fit.append((t[k], m))
                    m = 0.0
                    if len(fit) >= FREQ_FIT_SAMPLES:
                        ft, fm = np.array(fit).T
                        slope, icpt = np.polyfit(ft - ft[-1], fm, 1)
                        drift_int = slope
                        theta -= icpt
                        stepped = True
                        delays.clear()
                        samples.clear()
            # PI servo (linuxptp style), offsets in seconds, rate in s/s
            drift_int += pf.ki * m / T
            adj = pf.kp * m / T + drift_int
            if samples:                                    # older NTP samples: shift by the correction applied
                for q in samples:
                    q[0] -= pf.kp * m
            offs[k] = theta
            rates[k] = f0 + wander[k] - adj
        out.append(ClockTrace(sid, t, offs, rates, meas))
    return out


@dataclass
class SyncStats:
    profile: str
    stations: int
    converge_s: float | None               # when every station stays within 100 us of the others
    skew_p50_us: float
    skew_p95_us: float
    skew_p99_us: float
    skew_max_us: float
    offset_p95_us: float                   # vs the master (true time), after convergence
    sigma_us: float                         # 1-sigma of a station's clock vs the others (planner's sigma_t)
    pos_err_mm_at_5mps: float               # timing contribution to position error at 5 m/s (p95)


def skew_series(traces: list[ClockTrace], t: np.ndarray) -> np.ndarray:
    """Each station's clock error minus the mean of all stations, at times t: (S, T)."""
    E = np.stack([tr.at(t) for tr in traces])
    return E - E.mean(0, keepdims=True)


def summarise(traces: list[ClockTrace], duration_s: float, profile: str, settle_s: float | None = None,
              speed_mps: float = 5.0) -> SyncStats:
    t = np.linspace(0, duration_s, max(200, int(duration_s * 20)))
    S = skew_series(traces, t)
    worst = np.abs(S).max(0)                         # worst station at each time
    # settled: from then on the worst station's skew (30 s rolling median) never exceeds twice its
    # long-run level (the second half of the run), i.e. start-up is over
    win = max(1, int(30.0 / (t[1] - t[0])))
    roll = np.array([np.median(worst[max(0, i - win // 2):i + win // 2 + 1]) for i in range(len(worst))])
    level = float(np.mean(worst[len(t) // 2:]))
    above = np.nonzero(roll > 2 * level)[0]
    conv = 0.0 if not len(above) else (float(t[above[-1] + 1]) if above[-1] < len(t) - 2 * win else None)
    start = settle_s if settle_s is not None else (conv if conv is not None else duration_s / 2)
    sel = t >= start
    if sel.sum() < 10:
        sel = t >= t[len(t) // 2]
    # pairwise skew between stations (what the triangulation sees)
    Ss = S[:, sel]
    pair = np.abs(Ss[:, None, :] - Ss[None, :, :])
    iu = np.triu_indices(len(traces), 1)
    pv = pair[iu].ravel() if len(traces) > 1 else np.zeros(1)
    E = np.stack([tr.at(t[sel]) for tr in traces])
    q = lambda a, p: float(np.percentile(a, p)) / US                 # noqa: E731
    p95 = q(pv, 95)
    # a ray displaced by v*dt between two cameras moves the triangulated point by about half of it
    return SyncStats(profile, len(traces), conv, q(pv, 50), p95, q(pv, 99), float(pv.max()) / US,
                     q(np.abs(E), 95), float(np.std(Ss)) / US, speed_mps * p95 * US / 2 * 1e3)
