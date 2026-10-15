"""Closed loop with PX4 SITL (idps.sim.live): a SITL drone's true state -> the simulator's world
frame, and the cameras filming those drones -> the engine identifies them at their home spots."""
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from idps.core.frames import Frame
from idps.engine.calib import load_calibration
from idps.engine.engine import Engine
from idps.engine.ingest import Ingest
from idps.sim.live import R_EARTH, LiveSimulation, SitlDrone, SitlWorld, live_scenario, quat_to_R
from idps.sim.runner import MemorySink, write_calibration
from idps.sim.scenario import load_scenario

ROOT = Path(__file__).resolve().parents[1]


def _hil(n=0.0, e=0.0, d=0.0, vel=(0, 0, 0), q=(1, 0, 0, 0), lat0=47.0, lon0=8.0):
    lat = lat0 + math.degrees(n / R_EARTH)
    lon = lon0 + math.degrees(e / (R_EARTH * math.cos(math.radians(lat0))))
    return SimpleNamespace(lat=round(lat * 1e7), lon=round(lon * 1e7), alt=round(-d * 1e3), vx=round(vel[0] * 100),
                           vy=round(vel[1] * 100), vz=round(vel[2] * 100), attitude_quaternion=list(q))


def test_quaternion_matrix():
    a = math.radians(30)
    R = quat_to_R((math.cos(a / 2), math.sin(a / 2), 0, 0))           # roll 30°
    assert np.allclose(R, [[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def test_sitl_drone_state_in_world_frame():
    """Home spot, marker offset with tilt, NED -> a venue whose North is the drawing's +Y."""
    frame = Frame((1.0, 2.0), 90.0)                                    # origin (1, 2), North = drawing +Y
    d = SitlDrone(0, 7, [3.0, 4.0, 0.0], 0.0, frame)
    got = []
    d.publish = got.append
    d._on_state(0.0, _hil())                                          # first sample: the home spot
    a = math.radians(20)
    q = (math.cos(a / 2), math.sin(a / 2), 0, 0)                      # rolled 20° to the right
    for k in range(1, 400):                                           # 4 s climbing 1 m/s North at 100 Hz
        t = k * 0.01
        d._on_state(t, _hil(n=t, d=-0.5, vel=(1.0, 0, 0), q=q))
    s = got[-1]
    ned = np.array([3.0 + 3.99, 4.0, -0.5])
    assert np.allclose(s[1:4], frame.from_ned(ned), atol=3e-3)        # smoothed truth within quantisation
    assert np.allclose(s[4:7], [0, 1.0, 0], atol=1e-9)                # 1 m/s North = drawing +Y
    # marker 8 cm up the body axis; rolled right while facing North, so up leans East (drawing +X here)
    up_ned = -quat_to_R(q)[:, 2]
    assert np.allclose(s[7:10], frame.from_ned(0.08 * up_ned) - frame.from_ned(np.zeros(3)))
    assert math.isclose(s[13], 20.0, abs_tol=1e-6) and s[10] > 0


def test_sitl_drone_times_states_by_px4_clock_not_by_arrival():
    """States arriving late by a jittery delay are timed by PX4's own clock (which may run a
    few % fast), offset by the quickest delivery."""
    d = SitlDrone(0, 1, [0, 0, 0], 0.0, Frame())
    rng = np.random.default_rng(1)
    for k in range(1000):
        px4 = 100.0 + k * 0.01
        pc = 5.0 + k * 0.01 / 1.03                                    # PX4's clock 3 % fast
        got = d._pc_time(pc + 0.002 + rng.exponential(0.02), px4)
    assert abs(got - (pc + 0.002)) < 0.003


def test_sitl_world_extrapolates_and_reads_consistently():
    frame = Frame()
    rows = [{"drone_id": 1, "home_ned": [0, 0, 0]}, {"drone_id": 2, "home_ned": [1, 0, 0]}]
    w = SitlWorld(rows, 0.0, frame)
    assert w.wait(0.0) == [1, 2]
    SitlWorld.write(w.arr, 0, np.r_[1.0, 0, 0, 1, 2.0, 0, 0, 0, 0, 0.08, 0, 0, 1, 0])
    SitlWorld.write(w.arr, 1, np.r_[1.0, 1, 0, 1, 0, 0, 0, 0, 0, 0.08, 0, 0, 1, 0])
    assert w.wait(0.0) == []
    P, M, U, tilt = w.state(1.01)
    assert np.allclose(P[0], [0.02, 0, 1]) and np.allclose(M[1], [1, 0, 1.08])
    assert np.allclose(w.state(5.0)[0][0], [0.1, 0, 1])               # extrapolation capped at 50 ms


def test_sitl_world_films_a_late_frame_where_the_drone_was():
    """A frame rendered late (the host was busy) still sees each drone at the frame's time."""
    w = SitlWorld([{"drone_id": 1, "home_ned": [0, 0, 0]}], 0.0, Frame())
    for k in range(200):                                              # 2 s at 100 Hz, 1 m/s along x
        t = k * 0.01
        SitlWorld.write(w.arr, 0, np.r_[t, t, 0, 1, 1.0, 0, 0, 0, 0, 0.08, 0, 0, 1, 0])
    P, M, _, _ = w.state(1.555)                                       # 0.43 s behind the newest snap
    assert np.allclose(P[0], [1.555, 0, 1]) and np.allclose(M[0], [1.555, 0, 1.08])
    assert np.allclose(w.state(0.2)[0][0], [1.31, 0, 1])              # older than the ring: from its oldest


def test_cameras_filming_sitl_drones_feed_the_engine(tmp_path):
    """Three drones standing on their home spots, filmed through LiveSimulation (unpaced): the
    engine identifies each by its drone ID, at its spot."""
    man = {"venue": {"origin": {"xy": [0, 0], "north_deg": 0}},
           "drones": [{"drone_id": i + 1, "home_ned": h} for i, h in
                      enumerate([[0.0, 1.5, 0.0], [1.3, -0.75, 0.0], [-1.3, -0.75, 0.0]])]}
    sc = load_scenario(ROOT / "examples/scenarios/lab_three_drones.yaml")
    sc.duration_s = 6.0
    sc, frame, rows = live_scenario(sc, man)
    w = SitlWorld(rows, 0.0, frame)
    for k, d in enumerate(rows):
        h = frame.from_ned(d["home_ned"])
        SitlWorld.write(w.arr, k, np.r_[0.0, h, 0, 0, 0, 0, 0, 0.08, 0, 0, 1, 0])
    sim = LiveSimulation(sc, w, epoch_ns=1_700_000_000_000_000_000, t0=0.0)
    sink = MemorySink()
    sim.run(sink)
    write_calibration(sim, tmp_path / "calibration.yaml")
    eng = Engine(load_calibration(tmp_path / "calibration.yaml"), frame=frame)
    ing = Ingest()
    outs = []
    for _, pkt in sink.packets:
        for f in ing.feed(pkt):
            outs += eng.feed(f)
    outs += eng.flush()
    last = {o.marker_id: o for o in outs}
    assert sorted(last) == [1, 2, 3]
    for d in rows:
        want = np.array(d["home_ned"]) + [0, 0, -0.08]                  # the marker, 8 cm above the spot
        assert np.linalg.norm(last[d["drone_id"]].ned - want) < 0.005


def test_explain_run_tells_a_simulator_stall_from_late_positions(tmp_path):
    """A drone off its file after its simulator stood still, another after its positions were held up."""
    import json
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "px4" / "sitl"))
    from explain_run import explain
    show0, t0, epoch = 100.0, 50.0, 1_000_000_000_000
    (tmp_path / "start.txt").write_text(f"show time 0 = PC clock (time.monotonic) {show0} s\n")
    (tmp_path / "loop.json").write_text(json.dumps({"t0_monotonic_s": t0, "epoch_ns": epoch, "drones": [1, 2]}))
    t = np.arange(0, 30, 0.01)
    rows, clk, pos = [], [], []
    for d in (1, 2):
        err = np.zeros_like(t)
        hit = 10.0 if d == 1 else 20.0                     # drone 1 off at 10 s, drone 2 at 20 s
        err += 0.06 * np.exp(-((t - hit - 0.3) / 0.15) ** 2)
        rows += [(d, s, e, 0.0, 0.0) for s, e in zip(t, err)]
        lag = np.where((d == 1) & (t > hit - 0.2), 0.08, 0.0)   # drone 1's clock stood still 80 ms
        clk += [(d, show0 + s, show0 + s - 0.5 - g) for s, g in zip(t, lag)]
        age = np.where((d == 2) & (np.abs(t - (hit - 0.3)) < 0.05), 120.0, 22.0)   # drone 2: held up 100 ms
        for s, a in zip(t[::2], age[::2]):
            sent = show0 + s
            pos.append((int(epoch + (sent - a / 1000 - t0) * 1e9), d, a))
    np.savetxt(tmp_path / "track.csv", np.array(rows), delimiter=",", header="drone_id,show_t,dn,de,dd", comments="")
    np.savetxt(tmp_path / "clock.csv", np.array(clk), delimiter=",", header="drone_id,pc_s,px4_s", comments="")
    with open(tmp_path / "positions.csv", "w") as f:
        f.write("t_ns,marker_id,n,e,d,vn,ve,vd,n_cams,sigma_mm,age_ms\n")
        for tn, d, a in pos:
            f.write(f"{tn},{d},0,0,0,0,0,0,4,1.0,{a:.1f}\n")
    text = "\n".join(explain(tmp_path))
    lines = text.splitlines()
    i1 = next(i for i, ln in enumerate(lines) if "drone(s) 1 " in ln)
    i2 = next(i for i, ln in enumerate(lines) if "drone(s) 2 " in ln)
    assert "PX4 simulator stood still: drone 1 80 ms" in lines[i1 + 1]
    assert "positions late or missing: drone 2 up to 120 ms old" in lines[i2 + 1]
