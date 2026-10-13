"""Closed loop with PX4 SITL: the simulated camera stations film the SITL drones.

Each SITL drone's true state (HIL_STATE_QUATERNION on run_sitl.sh's truth link, udp 14800+i)
places it in the venue at its home spot from the show manifest. The stations film it in real
time and stream to the positioning engine (`idps-engine live`), whose positions are what PX4
flies on. Nothing in the loop knows where a drone is except through the cameras.

    idps-sim px4 examples/scenarios/lab_three_drones.yaml --manifest out/show/manifest.json \\
        --dest 127.0.0.1:5600 -o out/loop
"""
from __future__ import annotations

import json
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

from idps.core.frames import Frame
from idps.sim import trajectory as traj
from idps.sim.runner import Simulation
from idps.sim.scenario import Scenario, SimDrone

R_EARTH = 6378137.0
TRUTH_PORT = 14800          # run_sitl.sh: instance i sends its true state to 14800 + i
SMOOTH_TAU_S = 0.3          # see SitlDrone._on_state


def quat_to_R(q) -> np.ndarray:
    """MAVLink quaternion (w, x, y, z), body FRD -> NED, as a rotation matrix."""
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class SitlDrone:
    """One SITL drone's true state, received on its truth link and converted to the simulator's
    world frame. Each sample goes to publish(snap), snap = (t, centre xyz, velocity xyz, marker
    offset xyz, body up xyz, tilt deg)."""

    def __init__(self, instance: int, drone_id: int, home_ned, t0: float, frame: Frame,
                 marker_offset_flu=(0.0, 0.0, 0.08)):
        self.instance, self.drone_id = instance, drone_id
        self.home = np.asarray(home_ned, float)
        self.t0, self.frame = t0, frame
        ox, oy, oz = marker_offset_flu
        self.offset_frd = np.array([ox, -oy, -oz])
        # NED -> world: world = W @ ned + w0
        self.w0 = frame.from_ned(np.zeros(3))
        self.W = np.stack([frame.from_ned(e) - self.w0 for e in np.eye(3)], 1)
        self.publish = lambda snap: None
        self._t = 0.0
        self.samples = 0
        self._p = self._v = None      # smoothed NED position and velocity
        self._origin = None
        self._stop = False

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()
        return self

    def stop(self):
        self._stop = True

    def _run(self):
        from pymavlink import mavutil
        m = mavutil.mavlink_connection(f"udpin:127.0.0.1:{TRUTH_PORT + self.instance}",
                                       source_system=254, source_component=196)
        while not self._stop:
            h = m.recv_match(type="HIL_STATE_QUATERNION", blocking=True, timeout=0.5)
            if h is not None:
                self._on_state(time.monotonic() - self.t0, h)

    def _on_state(self, t: float, h):
        """The message carries latitude and longitude in 1e-7 degrees: 1.1 cm steps, far coarser
        than what the cameras resolve. North and East therefore integrate the (1 cm/s) velocity and
        are pulled gently (SMOOTH_TAU_S) to the coarse position, so the drone the cameras film moves
        smoothly. Height comes in mm and is used directly. Every SITL drone starts at its own
        latitude/longitude origin, which is its home spot."""
        lat, lon, alt = h.lat * 1e-7, h.lon * 1e-7, h.alt * 1e-3
        if self._origin is None:
            self._origin = (lat, lon, alt)
        o = self._origin
        meas = self.home + [math.radians(lat - o[0]) * R_EARTH,
                            math.radians(lon - o[1]) * R_EARTH * math.cos(math.radians(o[0])),
                            -(alt - o[2])]
        v = np.array([h.vx, h.vy, h.vz], float) * 0.01
        if self._p is None:
            p = meas
        else:
            dt = min(max(t - self._t, 0.0), 0.1)
            p = self._p + 0.5 * (self._v + v) * dt
            p[:2] += min(1.0, dt / SMOOTH_TAU_S) * (meas[:2] - p[:2])
            p[2] = meas[2]
        self._p, self._v = p, v
        R = quat_to_R(h.attitude_quaternion)
        up = -R[:, 2]                                       # body up, NED
        tilt = math.degrees(math.acos(min(1.0, max(-1.0, -up[2]))))
        self._t = t
        self.publish(np.concatenate([[t], self.W @ p + self.w0, self.W @ v, self.W @ (R @ self.offset_frd),
                                     self.W @ up, [tilt]]))
        self.samples += 1


SNAP = 15                       # per drone: version, then the 14 numbers of a SitlDrone snap


def _receive(buf, specs, t0: float, origin: dict, marker_offset):
    """Truth process: one receiving thread per drone, each writing its latest snap to shared
    memory under a version count (odd while writing)."""
    sys.setswitchinterval(5e-4)                        # stamp each message as it arrives
    frame = Frame.from_dict(origin)
    arr = np.frombuffer(buf, float).reshape(-1, SNAP)
    for k, (instance, drone_id, home) in enumerate(specs):
        d = SitlDrone(instance, drone_id, home, t0, frame, marker_offset)
        d.publish = (lambda k: lambda snap: SitlWorld.write(arr, k, snap))(k)
        d.start()
    while True:
        time.sleep(1.0)


class SitlWorld:
    """All SITL drones, in the simulator's world frame (the drawing frame, Z up).

    The drones' states are received in a separate process. In the camera loop's own process,
    receiving threads would take Python's interpreter lock hundreds of times a second and hold
    the cameras up by tens to hundreds of milliseconds."""

    def __init__(self, rows: list[dict], t0: float, frame: Frame, marker_offset_flu=(0.0, 0.0, 0.08)):
        import multiprocessing as mp
        self.ids = [d["drone_id"] for d in rows]
        self.buf = mp.RawArray("d", len(rows) * SNAP)
        self.arr = np.frombuffer(self.buf, float).reshape(-1, SNAP)
        self._args = (self.buf, [(i, d["drone_id"], list(d["home_ned"])) for i, d in enumerate(rows)], t0,
                      frame.to_dict(), tuple(marker_offset_flu))
        self.proc = None

    @classmethod
    def attach(cls, buf, ids):
        """The same shared states, read from another process (camera workers)."""
        w = cls.__new__(cls)
        w.ids, w.buf, w.proc, w._args = list(ids), buf, None, None
        w.arr = np.frombuffer(buf, float).reshape(-1, SNAP)
        return w

    def start(self):
        import multiprocessing as mp
        self.proc = mp.Process(target=_receive, args=self._args, daemon=True)
        self.proc.start()
        return self

    @staticmethod
    def write(arr, k: int, snap):
        arr[k, 0] += 1
        arr[k, 1:] = snap
        arr[k, 0] += 1

    def wait(self, timeout_s: float = 20.0) -> list[int]:
        """Wait for every drone's first state; returns the IDs still missing."""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end and not (self.arr[:, 0] > 0).all():
            time.sleep(0.1)
        return [i for i, v in zip(self.ids, self.arr[:, 0]) if v <= 0]

    def stop(self):
        if self.proc is not None:
            self.proc.terminate()

    def snaps(self) -> np.ndarray:
        for _ in range(20):
            v = self.arr[:, 0].copy()
            s = self.arr[:, 1:].copy()
            if not (v % 2).any() and (self.arr[:, 0] == v).all():
                return s
        return s

    def state(self, t: float):
        """Drone centres (D, 3), marker positions (D, 3), body up axes (D, 3), tilt (deg, D), as
        Simulation.state gives them: each drone's latest sample, moved on to time t."""
        s = self.snaps()
        dt = np.clip(t - s[:, 0], -0.05, 0.05)[:, None]
        P = s[:, 1:4] + s[:, 4:7] * dt
        return P, P + s[:, 7:10], s[:, 10:13], s[:, 13]


class LiveSimulation(Simulation):
    """Simulation whose drones are the SITL drones, run against the wall clock."""

    def __init__(self, sc: Scenario, world: SitlWorld, epoch_ns: int, t0: float):
        super().__init__(sc, epoch_ns=epoch_ns)
        self.world, self.t0 = world, t0
        self.late = 0                                  # frames simulated more than 20 ms late
        self.lag = deque(maxlen=2400)                  # recent lag behind the wall clock (s)

    def state(self, t: float):
        return self.world.state(t)

    def pace(self, t: float):
        """Hold each frame until its exposure time on the wall clock."""
        wait = self.t0 + t - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        elif wait < -0.02:
            self.late += 1
        self.lag.append(max(0.0, -wait))


def live_scenario(sc: Scenario, manifest: dict, n: int | None = None, marker_offset=(0.0, 0.0, 0.08)):
    """The scenario's cameras and models, with the show's drones in place of its own: drone ID =
    marker ID = MAV_SYS_ID (D21), SITL instance i = drone i+1 (run_sitl.sh)."""
    frame = Frame.from_dict(manifest["venue"]["origin"])
    rows = sorted(manifest["drones"], key=lambda d: d["drone_id"])[:n]
    sc.drones = [SimDrone(d["drone_id"], d["drone_id"], traj.Hover(frame.from_ned(d["home_ned"])), 0.0,
                          tuple(marker_offset)) for d in rows]
    return sc, frame, rows


def _stations_subset(sc: Scenario, idx: list[int]) -> Scenario:
    """The scenario with only some of its camera stations (same station IDs, so each station's
    random streams, clock and pose are the same as in the full simulation)."""
    import copy
    sub = copy.copy(sc)
    sub.cameras = [sc.cameras[i] for i in idx]
    sub.station_ids = [sc.station_ids[i] for i in idx]
    return sub


def _film(sc: Scenario, world: SitlWorld, epoch_ns: int, t0: float, t_start: float, duration: float, dest: str,
          truth_path=None, report: bool = True):
    """Run some camera stations against the wall clock until the end (or SIGTERM)."""
    import signal

    from idps.sim.runner import UdpSink
    sc.duration_s = t_start + duration + 60.0
    sim = LiveSimulation(sc, world, epoch_ns, t0)
    sc.duration_s = t_start + duration
    for st in sim.stations:
        st.phase += t_start
        st.n_frames = int((sc.duration_s - st.phase) / st.period) + 1
    host, port = dest.rsplit(":", 1)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    def progress(t, s):
        if report and int(t) % 5 == 0:
            lag = np.array(sim.lag) * 1e3
            print(f"  t={t - t_start:6.1f} s  frames {s.frames}  markers/frame {s.true_detections / max(1, s.frames):.2f}"
                  f"  behind the clock (ms, last 2 s): median {np.median(lag):.1f}, p99 {np.percentile(lag, 99):.1f}, "
                  f"max {lag.max():.1f}", flush=True)

    try:
        stats = sim.run(UdpSink(host, int(port), realtime=False), progress=progress, truth_path=truth_path,
                        pace=sim.pace)
        if report:
            print(f"done: {stats.frames} frames, {stats.packets} packets, {sim.late} frames late", flush=True)
    except KeyboardInterrupt:
        if report:
            print("stopped", flush=True)


def _film_worker(scenario, manifest, drones, idx, buf, ids, epoch_ns, t0, t_start, duration, dest):
    from idps.sim.scenario import load_scenario
    sc = load_scenario(scenario)
    sc, _, _ = live_scenario(sc, json.loads(Path(manifest).read_text()), drones)
    _film(_stations_subset(sc, idx), SitlWorld.attach(buf, ids), epoch_ns, t0, t_start, duration, dest, report=False)


def run_px4_loop(a) -> int:
    """idps-sim px4: film the PX4 SITL drones in real time and stream to the engine. With
    --workers K the stations are shared out over K processes (a real install has one computer
    per camera)."""
    import multiprocessing as mp

    from idps.sim.runner import write_calibration
    from idps.sim.scenario import load_scenario
    man = json.loads(Path(a.manifest).read_text())
    sc = load_scenario(a.scenario)
    sc.duration_s = a.duration
    sc, frame, rows = live_scenario(sc, man, a.drones)
    t0 = time.monotonic()
    epoch_ns = time.time_ns()
    world = SitlWorld(rows, t0, frame).start()
    missing = world.wait()
    if missing:
        print(f"no state from SITL drone(s) {missing}: start them with px4/sitl/run_sitl.sh", file=sys.stderr)
        return 1
    sc.duration_s = a.duration + 60.0
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    write_calibration(LiveSimulation(sc, world, epoch_ns, t0), out / "calibration.yaml")
    # truth.csv's t_s counts from t0 (time.monotonic): the PC clock the show runs on (idps-show start)
    (out / "loop.json").write_text(json.dumps({"t0_monotonic_s": t0, "epoch_ns": epoch_ns,
                                               "drones": [d["drone_id"] for d in rows]}))
    k = max(1, min(getattr(a, "workers", 1) or 1, len(sc.cameras)))
    parts = [list(range(len(sc.cameras)))[i::k] for i in range(k)]
    t_start = time.monotonic() - t0 + 0.5 + 0.5 * k          # time for the workers to start
    procs = [mp.Process(target=_film_worker, args=(a.scenario, a.manifest, a.drones, idx, world.buf, world.ids,
                                                    epoch_ns, t0, t_start, a.duration, a.dest), daemon=True)
             for idx in parts[1:]]
    for p in procs:
        p.start()
    print(f"filming {len(rows)} SITL drone(s) {[d['drone_id'] for d in rows]} with {len(sc.cameras)} cameras "
          f"('{sc.name}') in {k} process(es), streaming to {a.dest} for {a.duration:g} s; calibration and truth in "
          f"{out}", flush=True)
    try:
        _film(_stations_subset(sc, parts[0]), world, epoch_ns, t0, t_start, a.duration, a.dest,
              truth_path=out / "truth.csv")
    finally:
        for p in procs:
            p.terminate()
        world.stop()
    return 0
