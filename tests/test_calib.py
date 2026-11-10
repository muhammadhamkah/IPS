"""Camera calibration (idps.calib): the printable board, a lens from checkerboard pictures, and the
cameras' poses from one LED moved through the simulated lab."""
from pathlib import Path

import numpy as np
import pytest

from idps.calib.board import board_svg
from idps.calib.lens import Lens, load_lens, nominal_lens
from idps.engine.calib import CameraCal, load_calibration
from idps.sim import trajectory as traj
from idps.sim.runner import FileSink, MemorySink, Simulation, write_calibration
from idps.sim.scenario import SimDrone, load_scenario

ROOT = Path(__file__).resolve().parents[1]


def test_board_is_true_size():
    svg = board_svg(10, 7, 25.0)
    assert svg.count("<rect") == 1 + 35 and 'width="270.0mm"' in svg and 'height="195.0mm"' in svg


def _picture(cam: CameraCal, R, t, cols, rows, sq, ss=2):
    """A camera's picture of the board (its corner at t, turned by R, in the camera frame)."""
    W, H = cam.width, cam.height
    o = (np.arange(ss) + 0.5) / ss - 0.5
    u, v = np.meshgrid((np.arange(W)[:, None] + o[None]).ravel(), (np.arange(H)[:, None] + o[None]).ravel())
    xn, yn = cam.normalised(u, v)
    d = np.stack([xn, yn, np.ones_like(xn)], -1)
    n = R[:, 2]                                              # the board's plane: (X - t) . n = 0
    s = (t @ n) / (d @ n)
    Xb = (d * s[..., None] - t) @ R                          # in board coordinates
    bx, by = Xb[..., 0], Xb[..., 1]                           # squares span [-sq, (cols-1) sq] x [-sq, (rows-1) sq]:
    hit = s > 0                                              # the inner corners are at 0, sq, 2 sq, ...
    inside = hit & (bx > -sq) & (bx < (cols - 1) * sq) & (by > -sq) & (by < (rows - 1) * sq)
    black = ((np.floor(bx / sq) + np.floor(by / sq)) % 2 == 0) & inside
    m = 0.6 * sq                                             # white margin of the sheet
    border = hit & ~inside & (bx > -sq - m) & (bx < (cols - 1) * sq + m) & (by > -sq - m) & (by < (rows - 1) * sq + m)
    img = np.where(black, 25.0, np.where(inside | border, 215.0, 90.0))
    return img.reshape(H, ss, W, ss).mean((1, 3)).astype(np.uint8)


def test_lens_from_checkerboard_pictures(tmp_path):
    pytest.importorskip("cv2")
    from scipy.spatial.transform import Rotation
    from idps.calib.lens import calibrate_lens
    true = CameraCal(1, np.zeros(3), np.eye(3), 466.0, 322.0, 197.0, np.array([-0.05, 0.01, 0.0, 0.0, 0.0]), 640, 400)
    rng = np.random.default_rng(3)
    cols, rows, sq = 10, 7, 0.025
    imgs = []
    for k in range(14):
        R = Rotation.from_euler("xyz", rng.uniform([-35, -35, -20], [35, 35, 20]), degrees=True).as_matrix()
        centre = np.array([rng.uniform(-0.12, 0.12), rng.uniform(-0.07, 0.07), rng.uniform(0.28, 0.45)])
        t = centre - R @ np.array([(cols - 2) * sq / 2, (rows - 2) * sq / 2, 0])
        imgs.append(_picture(true, R, t, cols, rows, sq))
    lens = calibrate_lens(imgs, cols, rows, sq * 1000, station_id=1, log=lambda *a: None)
    assert lens.images_used >= 10 and lens.rms_px < 0.15
    assert lens.f_px == pytest.approx(true.f_px, rel=0.004)
    assert (lens.cx, lens.cy) == pytest.approx((true.cx, true.cy), abs=1.5)
    assert lens.distortion[0] == pytest.approx(-0.05, abs=0.01)
    back = load_lens(lens.save(tmp_path / "lens_1.yaml"))
    assert back.f_px == pytest.approx(lens.f_px, abs=1e-3) and back.station_id == 1 and back.images_used == lens.images_used
    assert nominal_lens(79.0).f_px == pytest.approx(640 / np.tan(np.radians(39.5)))


class _Wand(traj.Trajectory):
    """The LED: still on each measured point for 3 s, then wandering through the room."""

    def __init__(self, points, way, speed=0.8, rest_s=3.0, move_s=2.0):
        self.pts, self.rest_s, self.move_s = np.asarray(points, float), rest_s, move_s
        self.t_pts = len(points) * (rest_s + move_s)
        self.w = traj.Waypoints(way, speed)

    def position(self, t):
        t = np.asarray(t, float)
        out = np.empty(t.shape + (3,))
        for idx in np.ndindex(t.shape):
            tt = max(float(t[idx]), 0.0)
            if tt >= self.t_pts:
                out[idx] = self.w.position(np.array(tt - self.t_pts))
                continue
            k, r = divmod(tt, self.rest_s + self.move_s)
            k = int(k)
            nxt = self.pts[k + 1] if k + 1 < len(self.pts) else self.w.position(np.array(0.0))
            s = 0.0 if r < self.rest_s else 0.5 * (1 - np.cos(np.pi * (r - self.rest_s) / self.move_s))
            out[idx] = self.pts[k] + s * (nxt - self.pts[k])
        return out


POINTS = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [-1.0, -1.0, 0.0]]


@pytest.fixture(scope="module")
def wand_run(tmp_path_factory):
    sc = load_scenario(ROOT / "examples" / "scenarios" / "lab_three_drones.yaml")
    sc.duration_s = 50.0
    way = np.random.default_rng(1).uniform([-2.4, -2.4, 0.3], [2.4, 2.4, 2.4], (40, 3))
    sc.drones = [SimDrone(1, 5, _Wand(POINTS, way), marker_offset_m=(0.0, 0.0, 0.0))]
    sim = Simulation(sc)
    sink = MemorySink()
    d = tmp_path_factory.mktemp("wand")
    sim.run(sink, truth_path=d / "truth.csv")
    write_calibration(sim, d / "sim.yaml")
    rec = FileSink(d / "room.idpsraw", sim.epoch_ns)
    for t, pkt in sink.packets:
        rec.send(t, pkt)
    rec.close()
    return sink.packets, d, load_calibration(d / "sim.yaml", "true")


def _errors(cams, true):
    pos = max(float(np.linalg.norm(c.position - true[s].position)) for s, c in cams.items()) * 1000
    rot = max(float(np.arccos(np.clip((np.trace(c.R @ true[s].R.T) - 1) / 2, -1, 1))) for s, c in cams.items()) * 1000
    return pos, rot


def test_room_from_one_moving_led(wand_run):
    """Every camera's position within 2 mm and direction within 0.2 mrad of the truth, from one
    blinking LED: fixed reflections and false dots ignored, the cameras' different exposure times
    bridged, the world frame from four rest points."""
    pytest.importorskip("cv2")
    pytest.importorskip("scipy")
    from idps.calib.solve import solve_room
    from idps.calib.tracks import frames_from_packets, observations, single_dot_tracks
    packets, d, true = wand_run
    tracks, _ = single_dot_tracks(frames_from_packets(packets))
    assert all(tr.fixed_spots >= 1 for tr in tracks.values())           # the simulator's fixed reflections
    obs = observations(tracks, max_points=700)
    assert len(obs.rest) == 4
    lenses = {s: Lens(c.f_px, c.cx, c.cy, list(c.distortion), c.width, c.height, s) for s, c in true.items()}
    res = solve_room(obs, lenses, refs=POINTS, log=lambda *a: None)
    assert sorted(res.cams) == sorted(true) and not res.failed and res.frame == "rest points"
    pos, rot = _errors(res.cams, true)
    assert pos < 2.0 and rot < 0.2, (pos, rot)
    assert res.rms_all_px < 0.15 and max(res.ref_residual_mm) < 1.0

    # lenses known only roughly (focal length 3 % off, no distortion): refined here
    rough = {s: Lens(c.f_px * 1.03, 639.5, 399.5, [0.0] * 5, c.width, c.height, s) for s, c in true.items()}
    res2 = solve_room(obs, rough, refs=POINTS, refine_lens=True, log=lambda *a: None)
    pos2, rot2 = _errors(res2.cams, true)
    assert pos2 < 5.0 and rot2 < 0.5, (pos2, rot2)
    assert all(abs(c.f_px / true[s].f_px - 1) < 0.002 for s, c in res2.cams.items())

    # two cameras only, the world frame from where they were planned to stand and look
    two = sorted(true)[1:3]
    tr2 = {s: tracks[s] for s in two}
    res3 = solve_room(observations(tr2, max_points=700), lenses, planned={s: (true[s].position, true[s].R[2]) for s in two},
                      log=lambda *a: None)
    pos3, rot3 = _errors(res3.cams, true)
    assert res3.frame == "planned camera positions" and pos3 < 2.0 and rot3 < 0.3, (pos3, rot3)


def test_calib_room_command_writes_a_file_the_engine_reads(wand_run, tmp_path, capsys):
    pytest.importorskip("cv2")
    pytest.importorskip("scipy")
    import yaml
    from idps.calib.cli import main
    from idps.engine.engine import Engine
    from idps.engine.ingest import Ingest
    packets, d, true = wand_run
    for s, c in true.items():
        Lens(c.f_px, c.cx, c.cy, list(c.distortion), c.width, c.height, s).save(tmp_path / f"lens_{s}.yaml")
    (tmp_path / "points.yaml").write_text(yaml.safe_dump({"points": [{"xyz": p} for p in POINTS]}))
    out = tmp_path / "calibration.yaml"
    args = ["room", str(d / "room.idpsraw"), "--points", str(tmp_path / "points.yaml"), "--max-points", "700", "-o", str(out)]
    for s in true:
        args += ["--lens", str(tmp_path / f"lens_{s}.yaml")]
    assert main(args) == 0
    text = capsys.readouterr().out
    assert "world frame from rest points" in text and "4 rest points" in text
    cal = load_calibration(out)
    pos, rot = _errors(cal, true)
    assert pos < 2.0 and rot < 0.2
    # the engine, with this calibration, puts the LED where it was
    eng, ing, outp = Engine(cal), Ingest(), []
    for _, pkt in packets[: len(packets) // 3]:
        for f in ing.feed(pkt):
            outp += eng.feed(f)
    rest = [o.pos for o in outp if 5.5 < (o.t_ns - eng.t0_ns) * 1e-9 < 7.5]      # resting on the second point
    assert rest and np.linalg.norm(np.mean(rest, axis=0) - np.array(POINTS[1])) < 0.003


def test_calib_record_saves_what_the_stations_send(wand_run, tmp_path, capsys):
    import socket
    import threading
    import time
    from idps.calib.cli import main
    from idps.sim.runner import read_raw
    packets, _, _ = wand_run
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    out = tmp_path / "rec.idpsraw"
    rc = []
    th = threading.Thread(target=lambda: rc.append(main(["record", "-o", str(out), "--port", str(port), "--seconds", "2.5"])))
    th.start()
    time.sleep(0.5)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sent = [pkt for _, pkt in packets[20000:20600]]
    for k, pkt in enumerate(sent):
        tx.sendto(pkt, ("127.0.0.1", port))
        if k % 50 == 0:
            time.sleep(0.01)
    th.join(10)
    assert rc == [0]
    got = [pkt for _, pkt in read_raw(out)]
    assert len(got) > 0.9 * len(sent) and set(got) <= set(sent)
    assert "frames with the LED: station" in capsys.readouterr().out
