import math
from pathlib import Path

import numpy as np
import pytest

from idps.protocol import packets as P
from idps.sim import trajectory as T
from idps.sim.runner import FileSink, MemorySink, Simulation, read_raw, write_calibration
from idps.sim.scenario import load_scenario

ROOT = Path(__file__).resolve().parents[1]
LAB = ROOT / "examples" / "scenarios" / "lab_three_drones.yaml"


def short(path=LAB, seconds=1.0, **edit):
    sc = load_scenario(path)
    sc.duration_s = seconds
    for k, v in edit.items():
        obj, attr = k.split("__")
        setattr(getattr(sc, obj), attr, v)
    return sc


def test_circle_tilt_matches_physics():
    v, r = 2.0, 2.0
    c = T.Circle([0, 0, 1], r, v)
    acc = c.acceleration(np.array([0.3, 1.7]))
    assert np.linalg.norm(acc[:, :2], axis=1) == pytest.approx(v * v / r, rel=1e-3)
    assert T.tilt_deg(acc) == pytest.approx(math.degrees(math.atan(v * v / r / T.G)), abs=0.05)


def test_line_peak_speed_and_bounds():
    ln = T.Line([0, 0, 1], [10, 0, 1], speed=4.0)
    t = np.linspace(0, 20, 4001)
    p = ln.position(t)
    assert p[:, 0].min() >= -1e-9 and p[:, 0].max() <= 10 + 1e-9
    assert np.abs(ln.velocity(t)[:, 0]).max() == pytest.approx(4.0, rel=1e-3)


def test_fixed_yaw_body_axes():
    x, y, z = T.body_axes(np.array([[3.0, 0.0, 0.0]]), yaw_deg=0.0)
    assert x[0, 1] == pytest.approx(0) and y[0] == pytest.approx([0, 1, 0])     # pitching about Y keeps yaw
    assert z[0, 0] > 0                                                           # thrust tilts towards +X


def test_run_is_deterministic_and_packets_decode(tmp_path):
    a, b = MemorySink(), MemorySink()
    n = len(short().cameras)
    Simulation(short()).run(a)
    Simulation(short()).run(b)
    assert [p for _, p in a.packets] == [p for _, p in b.packets]
    frames = health = 0
    for _, pkt in a.packets:
        h, body = P.decode(pkt)
        assert h.flags & P.FLAG_SIMULATED
        if h.msg_type == P.MSG_FRAME_OBS:
            frames += 1
            assert body.sync_state == P.SYNC_PTP
        else:
            health += 1
            assert "plan_camera_id" in body
    assert frames > n * 110 and health == n            # ~120 frames/s/station, health at 0.5 s


def test_noise_free_observations_hit_true_projection():
    sc = short(camera_model__centroid_floor_px=0.0, camera_model__shot_noise_k=0.0,
               camera_model__false_positive_rate=0.0, camera_model__static_reflections=0,
               camera_model__frame_drop_prob=0.0, camera_model__distortion=(0, 0, 0, 0, 0),
               camera_model__saturation_dn=10**9)
    sim = Simulation(sc)
    for st in sim.stations:                       # a station that sees a marker at this moment
        t = st.frame_time(10)
        centres, mpos, ups, _ = sim.state(t)
        obs, truth = st.observe(t, mpos, ups, np.ones(len(mpos)), centres, sim.marker_drone)
        if obs:
            break
    assert obs, "some camera should see at least one marker"
    for m, (i, u, v) in zip(obs, truth):
        Xc = st.R @ (mpos[i] - st.pos)
        assert m.x == pytest.approx(st.cx + st.f * Xc[0] / Xc[2], abs=1e-6)
        assert m.y == pytest.approx(st.cy + st.f * Xc[1] / Xc[2], abs=1e-6)


def test_occlusion_and_merging():
    sc = short()
    sim = Simulation(sc)
    st = sim.stations[0]
    # put a second drone exactly on the ray from the camera to the first marker
    target = np.array([0.0, 0.0, 1.0])
    blocker = st.pos + 0.5 * (target - st.pos)
    centres = np.array([target - [0, 0, 0.08], blocker])
    mpos = np.array([target, blocker + [0, 0, 0.08]])
    ups = np.array([[0, 0, 1.0], [0, 0, 1.0]])
    occ = st._occluded(mpos, centres, np.array([0, 1]))
    assert occ[0] and not occ[1]
    merged = st._merge([[100, 100, 500, 300, 9, 0, 0], [101, 100.5, 400, 200, 9, 0, 1], [300, 300, 50, 30, 4, 0, 2]])
    assert len(merged) == 2 and merged[0][5] & P.MFLAG_ELONGATED
    assert merged[0][0] == pytest.approx((100 * 500 + 101 * 400) / 900)


def test_hw_trigger_aligns_capture_times():
    sim = Simulation(short(timing__sync="hw_trigger"))
    t = [st.frame_time(5) for st in sim.stations]
    assert np.ptp(t) < 50e-6


def test_binned_sensor_mode_pi3b():
    sc = short(ROOT / "examples" / "scenarios" / "lab_pi3b_wifi.yaml")
    sim = Simulation(sc)
    st = sim.stations[0]
    assert (st.W, st.H) == (640, 400) and st.f == pytest.approx(sc.cameras[0].lens_mm * 1e-3 / 6e-6)
    s = sim.run(MemorySink())
    assert s.true_detections > 0


def test_files_truth_calibration_and_raw_log(tmp_path):
    sc = short(seconds=0.5)
    sim = Simulation(sc)
    sink = FileSink(tmp_path / "p.idpsraw", sim.epoch_ns)
    stats = sim.run(sink, truth_path=tmp_path / "truth.csv")
    write_calibration(sim, tmp_path / "cal.yaml")
    recs = list(read_raw(tmp_path / "p.idpsraw"))
    assert len(recs) == stats.packets
    assert all(P.decode(b)[0].station_id in range(1, len(sc.cameras) + 1) for _, b in recs)
    assert all(t1 <= t2 for (t1, _), (t2, _) in zip(recs, recs[1:]))     # log is in send order
    lines = (tmp_path / "truth.csv").read_text().splitlines()
    assert lines[1].startswith("t_s,") and len(lines) - 2 == 3 * 61          # 3 drones at 120 Hz for 0.5 s
    import yaml
    cal = yaml.safe_load((tmp_path / "cal.yaml").read_text())
    c0 = cal["cameras"][0]
    Rt, Re = np.array(c0["true"]["R_world_to_cam"]), np.array(c0["estimate"]["R_world_to_cam"])
    ang = math.acos(min(1.0, (np.trace(Re @ Rt.T) - 1) / 2))
    assert 0 < ang < 1e-3                                   # perturbed by ~0.1 mrad


def test_clock_sync_profiles():
    """Disciplined clocks: software PTP on a Pi 3B keeps stations within tens of us of each
    other; hardware PTP within ~1 us; free running drifts apart by ppm x time."""
    from idps.sim.clocksync import simulate_clocks, summarise
    sw = summarise(simulate_clocks(8, 300, "pi3b_ptp_sw", seed=3), 300, "pi3b_ptp_sw")
    hw = summarise(simulate_clocks(8, 300, "pi5_ptp_hw", seed=3), 300, "pi5_ptp_hw")
    free = summarise(simulate_clocks(8, 300, "free_run", seed=3), 300, "free_run")
    wifi = summarise(simulate_clocks(8, 300, "pi3b_wifi_ntp", seed=3), 300, "pi3b_wifi_ntp")
    assert 10 < sw.skew_p95_us < 200 and sw.converge_s is not None and sw.converge_s < 120
    assert hw.skew_p95_us < 5
    assert free.skew_p95_us > 1e4 and wifi.skew_p95_us > 5 * sw.skew_p95_us
    assert sw.pos_err_mm_at_5mps < 0.5                       # timing costs well under 1 mm at 5 m/s
    # deterministic for a seed
    a = simulate_clocks(3, 60, "pi3b_ptp_sw", seed=5)[1]
    b = simulate_clocks(3, 60, "pi3b_ptp_sw", seed=5)[1]
    assert np.array_equal(a.offset, b.offset)


def test_path_asymmetry_is_invisible_to_ptp():
    """A fixed send/receive path difference biases the clock by half of it, and the servo cannot
    see it: the offset it reports averages ~0 while the true clock error does not."""
    from dataclasses import replace

    from idps.sim.clocksync import PROFILES, TimestampNoise, simulate_clocks
    pf = replace(PROFILES["pi5_ptp_hw"], asymmetry_us=0.0, slave_ts=TimestampNoise(0.01),
                 master_ts=TimestampNoise(0.01), network_pdv_us=0.01)
    biased = replace(pf, asymmetry_us=40.0)
    for prof, expect_bias in ((pf, False), (biased, True)):
        tr = simulate_clocks(4, 60, prof, seed=2, start_synced=True)
        true_err = np.array([np.mean(t.offset[len(t.t) // 2:]) for t in tr]) * 1e6
        reported = np.array([np.nanmean(t.measured[len(t.t) // 2:]) for t in tr]) * 1e6
        assert np.all(np.abs(reported) < 1.0)
        assert (np.abs(true_err).max() > 2.0) == expect_bias


def test_simulation_timestamps_follow_simulated_clocks(tmp_path):
    from idps.sim.clocksync import simulate_clocks  # noqa: F401  (profile wiring)
    from idps.sim.cli import main
    sc = load_scenario(ROOT / "examples" / "scenarios" / "lab_pi3b_ethernet.yaml")
    sc.duration_s = 2.0
    sim = Simulation(sc)
    assert all(st.clock is not None for st in sim.stations)
    sink = MemorySink()
    sim.run(sink)
    stations = {s.id: s for s in sim.stations}
    errs, offsets = [], []
    for _, pkt in sink.packets:
        h, body = P.decode(pkt)
        if h.msg_type == P.MSG_FRAME_OBS and body.part_index == 0:
            s = stations[body.station_id]
            t = s.phase + body.frame_id * s.period
            errs.append((body.t_mid_ns - sim.epoch_ns) * 1e-9 - t - float(s.clock.at(t)))
        elif h.msg_type == P.MSG_HEALTH:
            offsets.append(body["ptp_offset_ns"])
    assert abs(np.mean(errs)) < 3e-6 and np.std(errs) < 30e-6      # only the frame-timestamp jitter remains
    assert offsets and any(o != 0 for o in offsets)                    # ptp4l-style reported offsets
    assert main(["clocks", "--stations", "4", "--duration", "60", "--profile", "pi3b_ptp_sw",
                 "--profile", "pi5_ptp_hw", "-o", str(tmp_path)]) == 0
    assert (tmp_path / "clocks_summary.csv").read_text().count("\n") == 3
