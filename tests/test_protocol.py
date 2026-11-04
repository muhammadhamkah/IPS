import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from idps.protocol import markercode as MC
from idps.protocol import packets as P

ROOT = Path(__file__).resolve().parents[1]
VEC = ROOT / "protocol" / "vectors"


def test_golden_vectors_match_encoder():
    import importlib.util
    spec = importlib.util.spec_from_file_location("mv", ROOT / "protocol" / "make_vectors.py")
    mv = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mv)
    for name, data in mv.vectors():
        assert (VEC / f"{name}.bin").read_bytes() == data, name


def test_decode_two_markers():
    h, f = P.decode((VEC / "frame_two_markers.bin").read_bytes())
    assert (h.station_id, h.seq, h.msg_type) == (3, 5, P.MSG_FRAME_OBS)
    assert h.flags & P.FLAG_SIMULATED
    assert f.frame_id == 10482 and f.t_mid_ns == 58322100000 and f.sync_state == P.SYNC_PTP
    assert f.markers[0].x == pytest.approx(621.42, abs=1e-4)
    assert f.markers[1].mflags == P.MFLAG_SATURATED | P.MFLAG_ELONGATED


def test_split_frame_parts():
    parts = sorted(VEC.glob("frame_split_part*.bin"))
    decoded = [P.decode(p.read_bytes()) for p in parts]
    assert [len(f.markers) for _, f in decoded] == [89, 11]
    assert all(f.n_markers_total == 100 for _, f in decoded)
    assert decoded[0][0].flags & P.FLAG_PART and not decoded[0][0].flags & P.FLAG_LAST_PART
    assert decoded[1][0].flags & P.FLAG_LAST_PART
    assert [h.seq for h, _ in decoded] == [0, 1]
    assert len(parts[0].read_bytes()) == P.MAX_UDP_PAYLOAD


def test_empty_frame_and_health():
    h, f = P.decode((VEC / "frame_empty.bin").read_bytes())
    assert f.markers == [] and h.payload_len == 32
    h, body = P.decode((VEC / "health.bin").read_bytes())
    assert h.msg_type == P.MSG_HEALTH and body["fps_measured"] == 119.98


@pytest.mark.parametrize("mutate", ["magic", "version", "length", "truncate", "type"])
def test_malformed_packets_rejected(mutate):
    b = bytearray((VEC / "frame_two_markers.bin").read_bytes())
    if mutate == "magic":
        b[0] ^= 1
    elif mutate == "version":
        b[4] = 9
    elif mutate == "length":
        b[12] += 1
    elif mutate == "truncate":
        b = b[:-3]
    else:
        b[5] = 77
    with pytest.raises(P.ProtocolError):
        P.decode(bytes(b))


@pytest.mark.skipif(shutil.which("gcc") is None, reason="needs gcc")
def test_c_header_reads_python_vectors(tmp_path):
    exe = tmp_path / "check"
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Werror", "-o", str(exe), str(ROOT / "tests" / "c" / "check_vectors.c")],
                   check=True)
    r = subprocess.run([str(exe), str(VEC / "frame_two_markers.bin")], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout


def test_marker_code_roundtrip_and_distance():
    words = []
    for mid in range(256):
        c = MC.chips(mid)
        assert len(c) == MC.CHIPS_PER_FRAME == 30
        assert MC.decode_chips(c) == mid
        words.append(int("".join(map(str, MC.codeword_bits(mid))), 2))
        data = c[len(MC.SYNC):]
        runs = np.diff(np.flatnonzero(np.diff(np.r_[-1, data, -1]) != 0))
        assert runs.max() <= 2                      # sync (run of 3) can never appear in data
    dmin = min(bin(a ^ b).count("1") for i, a in enumerate(words) for b in words[i + 1:])
    assert dmin >= 3                                # every single-bit error is detected... and correctable
    bad = MC.chips(17).copy()
    bad[8] ^= 1
    bad[9] ^= 1                                     # flip one Manchester bit -> CRC must fail
    assert MC.decode_chips(bad) is None


def test_led_mean_level_over_exposure():
    led = MC.MarkerLed(5, chip_rate_hz=30.0, high=1.0, low=0.4)
    # first chip is HIGH (sync); an exposure entirely inside it averages to HIGH
    assert led.mean_level(0.001, 0.002) == pytest.approx(1.0)
    # an exposure straddling the 3rd->4th chip boundary (HIGH->LOW) averages between
    tb = 3 / 30.0
    assert led.mean_level(tb - 0.0005, tb + 0.0005) == pytest.approx(0.7)
