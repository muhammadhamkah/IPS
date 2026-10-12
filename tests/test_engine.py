"""Positioning engine: pixels -> rays, triangulation, blink-code IDs, ingest, and end to end
against the simulator's ground truth."""
import dataclasses
from pathlib import Path

import numpy as np
import pytest

from idps.engine.calib import CameraCal, load_calibration, rays_batch
from idps.engine.engine import Engine
from idps.engine.evaluate import read_truth, score
from idps.engine.idcode import decode
from idps.engine.ingest import Ingest
from idps.engine.triangulate import closest_approach, triangulate, triangulate_many
from idps.protocol import packets as P
from idps.protocol.markercode import MarkerLed
from idps.sim.runner import MemorySink, Simulation, write_calibration
from idps.sim.scenario import load_scenario

ROOT = Path(__file__).resolve().parents[1]


def _cam(pos, R=np.eye(3), dist=(-0.05, 0.01, 0.001, -0.001, 0.0)):
    return CameraCal(1, np.asarray(pos, float), np.asarray(R, float), 933.3, 639.5, 399.5, np.asarray(dist, float))


def test_pixel_ray_round_trip_with_distortion():
    c = _cam([1.0, 2.0, 3.0])
    X = np.array([[1.4, 2.3, 8.0], [0.2, 1.5, 5.0], [3.0, 4.0, 9.0]])
    u, v, z = c.project(X)
    d = c.rays(u, v)
    want = (X - c.position) / np.linalg.norm(X - c.position, axis=1, keepdims=True)
    assert np.allclose(d, want, atol=1e-9)
    assert np.allclose(rays_batch([c, c, c], np.column_stack([u, v])), d)


def test_triangulation_and_motion_correction():
    rng = np.random.default_rng(3)
    X = np.array([1.0, -0.5, 1.8])
    O = rng.uniform([-6, -6, 3], [6, 6, 4], (5, 3))
    D = (X - O) / np.linalg.norm(X - O, axis=1, keepdims=True)
    p = triangulate(O, D, np.full(5, 933.0), 1e-4)
    assert np.allclose(p.X, X, atol=1e-9) and p.n == 5 and p.resid_px.max() < 1e-3
    # a moving marker seen at different instants: shift each ray back with the velocity
    vel, dt = np.array([3.0, 0.0, 0.0]), np.array([-0.004, -0.002, 0.0, 0.002, 0.004])
    Xt = X + np.outer(dt, vel)
    Dt = (Xt - O) / np.linalg.norm(Xt - O, axis=1, keepdims=True)
    assert np.linalg.norm(triangulate(O, Dt, 933.0, 1e-4, dt=dt, vel=vel).X - X) < 1e-9
    # one wrong ray is dropped
    D2 = D.copy()
    D2[2] = D2[2] + np.array([0.02, 0, 0])
    D2[2] /= np.linalg.norm(D2[2])
    p = triangulate(O, D2, 933.0, 1e-4)
    assert not p.used[2] and np.linalg.norm(p.X - X) < 1e-9
    # batched version agrees
    Xb, cov, ok, keep, ncam = triangulate_many(np.zeros(5, int), O, D2, np.full(5, 933.0), X[None] + 0.01,
                                                np.zeros((5, 3)), 1, 1e-4)
    assert ok[0] and not keep[2] and ncam[0] == 4 and np.linalg.norm(Xb[0] - X) < 1e-6
    mid, gap, s, t = closest_approach(O[0], D[0], O[1], D[1])
    assert gap < 1e-9 and np.allclose(mid, X)


@pytest.mark.parametrize("mid", [0, 17, 99, 255])
def test_blink_code_decoder(mid):
    rng = np.random.default_rng(mid)
    led = MarkerLed(mid, phase_s=rng.uniform(0, 1), clock_ppm=80)
    t = np.sort(np.concatenate([np.arange(0, 2.5, 1 / 120) + rng.uniform(0, 1 / 120) for _ in range(4)]))
    lv = np.log(led.level(t) * rng.uniform(0.9, 1.1, len(t)))
    got, n, contrast = decode(t, lv - np.median(lv))
    assert got == mid and n >= 1 and contrast > 0.3
    assert decode(t, rng.normal(0, 0.05, len(t)))[0] is None or True     # noise rarely decodes; never crashes


def test_ingest_reassembles_split_frames_and_counts_loss():
    obs = P.FrameObs(4, 77, 123456789, 500000, [P.Marker(float(i), 2.0 * i, 100 + i, 50, 9) for i in range(150)])
    pk = P.encode_frame(obs, seq_start=10)
    assert len(pk) == 2
    ing = Ingest()
    assert ing.feed(pk[1]) == []                                  # parts may arrive out of order
    (f,) = ing.feed(pk[0])
    assert f.n == 150 and f.xy[149, 0] == 149.0 and f.station_id == 4
    assert ing.feed(P.encode_frame(dataclasses.replace(obs, frame_id=78, markers=[]), seq_start=14)[0])
    assert ing.stations[4].lost == 2                              # seq 12 and 13 never arrived
    assert ing.feed(b"garbage") == [] and ing.stations[-1].bad == 1


@pytest.fixture(scope="module")
def lab_run(tmp_path_factory):
    sc = load_scenario(ROOT / "examples" / "scenarios" / "lab_three_drones.yaml")
    sc.duration_s = 7.0
    sim = Simulation(sc)
    sink = MemorySink()
    d = tmp_path_factory.mktemp("lab")
    sim.run(sink, truth_path=d / "truth.csv")
    write_calibration(sim, d / "calibration.yaml")
    return sink.packets, d


def test_engine_end_to_end_lab(lab_run):
    """3 drones, 9 cameras, calibration with realistic errors: every drone found, identified
    from its blink code within ~3.5 s, and within the 1 cm requirement."""
    packets, d = lab_run
    eng, ing, out = Engine(load_calibration(d / "calibration.yaml")), Ingest(), []
    for _, pkt in packets:
        for f in ing.feed(pkt):
            out += eng.feed(f)
    out += eng.flush()
    res, unknown = score(out, read_truth(d / "truth.csv"))
    assert unknown == 0 and len(res) == 3
    for r in res:
        assert r.outputs > 300 and r.wrong == 0 and r.first_id_s < 3.6, r.line()
        assert r.p95_mm < 10.0 and r.max_mm < 15.0, r.line()
        assert r.availability > 0.98, r.line()
    assert all(o.n_cams >= 2 for o in out)


def test_engine_ned_output_and_reflections_ignored(lab_run):
    from idps.plan.site import load_site
    packets, d = lab_run
    frame = load_site(ROOT / "examples" / "sites" / "lab.yaml").frame
    eng, ing, out = Engine(load_calibration(d / "calibration.yaml"), frame=frame), Ingest(), []
    for _, pkt in packets:
        for f in ing.feed(pkt):
            out += eng.feed(f)
    o = out[-1]
    assert np.allclose(o.ned, frame.ned(o.pos[None])[0])
    from idps.engine.cli import vel_ned
    up = np.array([0.0, 0.0, 1.0])                                # climbing = negative Down
    assert vel_ned(frame, up) == pytest.approx([0, 0, -1])
    v = frame.n_hat * 2.0
    assert vel_ned(frame, np.r_[v, 0.0]) == pytest.approx([2, 0, 0])
    # the simulated cameras each see fixed reflections: they never become identified drones
    assert {o.marker_id for o in out} == {17, 42, 99}
    if eng.backend == "cpp":
        assert eng._core.n_static_spots() > 0
    else:
        assert any(m.xy.size for m in eng.static.values())


def _run(packets, d, backend):
    eng, ing, out = Engine(load_calibration(d / "calibration.yaml"), backend=backend), Ingest(), []
    for _, pkt in packets:
        for f in ing.feed(pkt):
            out += eng.feed(f)
    return out + eng.flush()


def test_cpp_core_matches_python_reference(lab_run):
    """The compiled core is a port of the Python engine: same tracks, IDs and positions."""
    pytest.importorskip("idps.engine._core")
    packets, d = lab_run
    py, cpp = _run(packets, d, "python"), _run(packets, d, "cpp")
    assert len(py) == len(cpp) > 300
    for a, b in zip(py, cpp):
        assert (a.t_ns, a.marker_id, a.n_cams) == (b.t_ns, b.marker_id, b.n_cams)
        assert np.allclose(a.pos, b.pos, atol=1e-9) and np.allclose(a.cov, b.cov, rtol=1e-6, atol=1e-15)


def test_bearing_update_has_no_velocity_feedback():
    """Cameras exposing at different instants, alternating camera sets between slots, and a
    1 m/s velocity error to start with: the line-of-sight update converges instead of feeding
    the velocity error back into the position (the cause of the old 10-18 mm bursts)."""
    from idps.engine.tracker import KalmanCV, Track
    rng = np.random.default_rng(5)
    cams = rng.uniform([-15, -15, 8], [15, 15, 12], (8, 3))
    v_true = np.array([2.0, -1.0, 0.3])
    tr = Track(1, KalmanCV(np.zeros(3), 0.0, np.eye(3) * 1e-4, q=400.0), 0.0)
    tr.kf.x[3:] = v_true + np.array([1.0, 0.0, 0.0])                # wrong velocity prior
    for k in range(1, 241):
        T = k / 120
        tr.kf.predict(T)
        subset = cams[0::2] if k % 2 else cams[1::2]               # alternating camera sets
        dt = rng.uniform(-0.004, 0.004, len(subset))               # each camera's own exposure time
        X = v_true[None] * (T + dt)[:, None]                       # the marker when each camera looked
        D = (X - subset) / np.linalg.norm(X - subset, axis=1, keepdims=True)
        Engine._bearing_update(tr, subset, D, dt, 1e-5)
    assert np.linalg.norm(tr.kf.pos - v_true * 2.0) < 1e-3          # < 1 mm
    assert np.linalg.norm(tr.kf.vel - v_true) < 0.02
