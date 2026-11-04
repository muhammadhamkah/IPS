"""The camera station (edge/): dot detector, packets, the idps-station program, and the whole chain
from camera images to drone positions."""
import shutil
import socket
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest

from idps.protocol import packets as P
from idps.station.detect import ELONGATED, EDGE, SATURATED, DetectParams, detect
from idps.station.render import SensorNoise, Spot, read_frames, render, render_sparse, spot_flux_for_peak, write_frames

ROOT = Path(__file__).resolve().parents[1]
S = pytest.importorskip("idps.station._station", reason="station C++ module not built (pip install -e .)")


def _frame(rng, n=30, bits=8, W=640, H=400, peak=(80, 240)):
    k = 4 if bits == 10 else 1
    spots = [Spot(rng.uniform(5, W - 6), rng.uniform(5, H - 6), spot_flux_for_peak(rng.uniform(*peak) * k, 1.2))
             for _ in range(n)]
    noise = SensorNoise(bits=bits, black=4 * k, read_noise=1.0 * k, dn_per_electron=0.25 * k)
    return spots, render(W, H, spots, noise, rng)


@pytest.mark.parametrize("bits", [8, 10])
def test_cpp_detector_matches_the_python_reference(bits):
    rng = np.random.default_rng(bits)
    p = dict(threshold=60, threshold_low=30, saturation=255) if bits == 8 else \
        dict(threshold=240, threshold_low=120, saturation=1023)
    det = S.Detector(p)
    for _ in range(4):
        _, img = _frame(rng, bits=bits)
        ref, got = detect(img, DetectParams(**p)), det.detect(img)
        assert len(ref) == len(got) > 20
        for a, b in zip(ref, got):
            assert (a.x, a.y, a.sum) == pytest.approx(b[:3], abs=1e-6)
            assert (a.peak, a.area, a.flags) == tuple(b[3:])


def test_centroids_are_within_a_few_hundredths_of_a_pixel():
    rng = np.random.default_rng(5)
    det = S.Detector({"threshold": 60, "threshold_low": 30})
    errs = []
    for _ in range(6):
        spots, img = _frame(rng, n=12, peak=(100, 230))
        dots = det.detect(img)
        for s in spots:
            d = min(dots, key=lambda b: (b[0] - s.x) ** 2 + (b[1] - s.y) ** 2)
            e = np.hypot(d[0] - s.x, d[1] - s.y)
            if e < 1.0:                     # (two spots that ran into each other are a separate case)
                errs.append(e)
    errs = np.array(errs)
    assert len(errs) > 60
    assert np.median(errs) < 0.05 and np.percentile(errs, 95) < 0.12


def test_flags_area_limit_and_empty_frames():
    det = S.Detector({"threshold": 60, "threshold_low": 30, "max_area": 60})
    noise = SensorNoise(read_noise=0.0, dn_per_electron=0.0)
    rng = np.random.default_rng(0)
    img = render(200, 120, [Spot(1.0, 60.0, spot_flux_for_peak(150, 1.2)),          # on the edge
                            Spot(60.0, 60.0, spot_flux_for_peak(1500, 1.2)),         # clipped
                            Spot(120.0, 60.0, spot_flux_for_peak(150, 1.2)),         # two spots ...
                            Spot(123.5, 60.0, spot_flux_for_peak(150, 1.2)),         # ... run together
                            Spot(170.0, 60.0, spot_flux_for_peak(5000, 6.0))],       # too big: a lamp
                 noise, rng)
    dots = sorted(det.detect(img), key=lambda d: d[0])
    assert [d[5] for d in dots] == [EDGE, SATURATED, ELONGATED]
    assert dots[2][0] == pytest.approx(121.75, abs=0.1)        # the merged dot sits between the two
    assert det.detect(np.full((120, 200), 4, np.uint8)) == []


def test_cpp_packets_are_byte_for_byte_the_python_ones():
    for n in (0, 2, 100):                                       # 100: split over two packets
        dots = [(10.25 + i, 20.5, 1234.7 + i, 200, 9, i % 4) for i in range(n)]
        cpp = S.encode_frame(3, 10482, 58322100000, 500000, dots, 5, proc_us=1400, sync_state=1, threshold=60,
                             drops=2, flags=P.FLAG_SIMULATED)
        obs = P.FrameObs(3, 10482, 58322100000, 500000, [P.Marker(x, y, int(s), pk, a, f) for x, y, s, pk, a, f in dots],
                         proc_us=1400, sync_state=1, threshold=60, drops_since_last=2, flags=P.FLAG_SIMULATED)
        assert cpp == P.encode_frame(obs, 5)
    body = {"fw": "x", "fps_measured": 119.98}
    import json
    js = json.dumps(body, separators=(",", ":"))
    assert S.encode_health(3, 9, js) == P.encode_health(3, 9, body)


def test_raw10_unpacking():
    vals = np.arange(10, dtype=np.uint16).reshape(1, 10) * 101 % 1024    # 10 pixels: 2 groups + 2 left over
    packed = bytearray()
    for g in range(0, 10, 4):
        grp = [int(v) for v in vals[0, g:g + 4]]
        grp += [0] * (4 - len(grp))                                      # a short last group still takes 5 bytes
        packed += bytes(v >> 2 for v in grp)
        packed.append(sum((v & 3) << (2 * i) for i, v in enumerate(grp)))
    packed += b"\0" * 3                                                  # row stride padding
    out = S.unpack_raw10(bytes(packed), 10, 1, len(packed))
    assert (out == vals).all()


def test_frame_files_and_the_sparse_renderer(tmp_path):
    rng = np.random.default_rng(2)
    _, a = _frame(rng)
    _, b = _frame(rng)
    f = write_frames(tmp_path / "f.idpsfrm", [a, b])
    assert (read_frames(f) == np.stack([a, b])).all()
    spots = [Spot(50.3, 40.6, spot_flux_for_peak(150, 1.2)), Spot(52.0, 41.0, spot_flux_for_peak(150, 1.2)),
             Spot(300.0, 200.0, spot_flux_for_peak(150, 1.2))]
    img = render_sparse(640, 400, spots, SensorNoise(read_noise=0, dn_per_electron=0), rng)
    full = render(640, 400, spots, SensorNoise(read_noise=0, dn_per_electron=0), rng)
    assert (img == full).all()                  # overlapping spots add up exactly as in a full render


def test_images_through_the_station_detector_give_the_same_positions(tmp_path):
    """The whole chain: simulated cameras render images, the station's C++ detector finds the dots,
    the engine turns them into identified drone positions within millimetres."""
    from idps.engine.calib import load_calibration
    from idps.engine.engine import Engine
    from idps.engine.evaluate import read_truth, score
    from idps.engine.ingest import Ingest
    from idps.sim.runner import MemorySink, Simulation, write_calibration
    from idps.sim.scenario import load_scenario
    from idps.station.filming import film_with_station_detector
    sc = load_scenario(ROOT / "examples" / "scenarios" / "lab_three_drones.yaml")
    sc.duration_s = 6.0
    sim = Simulation(sc)
    film_with_station_detector(sim)
    sink = MemorySink()
    sim.run(sink, truth_path=tmp_path / "truth.csv")
    write_calibration(sim, tmp_path / "calib.yaml")
    eng, ing, out = Engine(load_calibration(tmp_path / "calib.yaml")), Ingest(), []
    for _, pkt in sink.packets:
        for f in ing.feed(pkt):
            out += eng.feed(f)
    res, _ = score(out + eng.flush(), read_truth(tmp_path / "truth.csv"))
    assert len(res) == 3
    for r in res:
        assert r.wrong == 0 and r.first_id_s < 3.6, r.line()
        assert r.p95_mm < 10.0 and r.availability > 0.98, r.line()


# ------------------------------------------------------------------ the idps-station program
@pytest.fixture(scope="module")
def station_exe(tmp_path_factory):
    if shutil.which("cmake") is None:
        pytest.skip("needs cmake")
    build = tmp_path_factory.mktemp("edge-build")
    subprocess.run(["cmake", "-S", str(ROOT / "edge"), "-B", str(build), "-DCMAKE_BUILD_TYPE=Release"], check=True,
                   capture_output=True)
    subprocess.run(["cmake", "--build", str(build), "-j4"], check=True, capture_output=True)
    return build / "idps-station"


def test_station_sends_one_packet_per_frame_and_health(station_exe):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(0.5)
    port = rx.getsockname()[1]
    proc = subprocess.Popen([str(station_exe), "run", "--source", "synthetic", "--dots", "5",
                             "--engine", f"127.0.0.1:{port}", "--id", "7"], stdout=subprocess.PIPE)
    frames, health, seqs = [], [], []
    try:
        end = time.time() + 15.0                            # the first packet (it prepares its frames first)
        while time.time() < end:
            try:
                h, body = P.decode(rx.recv(2048))
            except socket.timeout:
                continue
            if not seqs:
                end = time.time() + 2.5                         # then 2.5 s of packets
            seqs.append(h.seq)
            (frames if h.msg_type == P.MSG_FRAME_OBS else health).append(body)
    finally:
        proc.terminate()
        proc.wait(5)
    assert len(frames) > 200 and health                     # 120 fps, and HEALTH every second
    assert all(f.station_id == 7 and len(f.markers) == 5 and f.flags & P.FLAG_SIMULATED for f in frames)
    assert np.diff(seqs).tolist() == [1] * (len(seqs) - 1)
    period = np.median(np.diff([f.t_mid_ns for f in frames])) * 1e-9
    assert period == pytest.approx(1 / 120, rel=0.01)
    assert abs(time.time_ns() - frames[-1].t_mid_ns) < 2e9    # stamped in the system time base
    assert health[-1]["fps_measured"] > 110 and health[-1]["frames_dropped"] == 0 and health[-1]["send_errors"] == 0


def test_station_snapshot_and_bench(station_exe, tmp_path):
    rng = np.random.default_rng(4)
    spots, img = _frame(rng, n=6)
    frames = write_frames(tmp_path / "f.idpsfrm", [img])
    out = subprocess.run([str(station_exe), "snapshot", "--source", f"file:{frames}", "--skip", "2",
                          "--out", str(tmp_path / "s.pgm")], capture_output=True, text=True, check=True)
    pgm = (tmp_path / "s.pgm").read_bytes()
    header = b"P5\n640 400\n255\n"
    assert pgm.startswith(header) and np.frombuffer(pgm[len(header):], np.uint8).reshape(400, 640).tolist() == img.tolist()
    assert "6 dots" in out.stdout or "dots:" in out.stdout
    b = subprocess.run([str(station_exe), "bench", "--source", "synthetic", "--seconds", "2", "--bench-dots", "0,50"],
                       capture_output=True, text=True)
    assert "1. Dot detection" in b.stdout and "frame rate" in b.stdout and ("GO" in b.stdout)
