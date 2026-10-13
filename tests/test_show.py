"""Shows: reading Skybrush exports, placing them in the venue, checking and exporting."""
import io
import json
import sys
import types
import zipfile
from pathlib import Path

import numpy as np
import pytest

from idps.plan.model import Plan
from idps.plan.site import load_site
from idps.show.check import Limits, check_show
from idps.show.demo import write_demo_skyc
from idps.show.export import export_px4
from idps.show.model import track_from_points
from idps.show.placement import Placement
from idps.show.readers import load_show
from idps.show.runner import TYPE_MASK_POS_VEL_ACC_YAW, TrajectoryPlayer

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def lab():
    return load_site(ROOT / "examples" / "sites" / "lab.yaml"), Plan.load(ROOT / "examples" / "plans" / "lab.yaml").cameras


def test_bezier_segments_position_velocity_acceleration():
    tr = track_from_points("d", [[0, [0, 0, 0], []], [2, [4, 0, 0], [[1, 0, 0], [3, 0, 0]]], [3, [4, 2, 0], []]])
    assert tr.sample([0, 1, 2, 2.5, 3]) == pytest.approx(np.array([[0, 0, 0], [2, 0, 0], [4, 0, 0], [4, 1, 0], [4, 2, 0]]))
    assert tr.sample([1], 1)[0] == pytest.approx([2.25, 0, 0])          # cubic: 3*(1,2,1)/4 / 2 s
    assert tr.sample([2.5], 1)[0] == pytest.approx([0, 2, 0])           # straight segment
    with pytest.raises(ValueError):
        track_from_points("d", [[0, [0, 0, 0], [[1, 1, 1]]]])


@pytest.mark.parametrize("layout", ["inline", "folders"])
def test_skyc_both_layouts(tmp_path, layout):
    show = load_show(write_demo_skyc(tmp_path / "s.skyc", layout=layout, n=6))
    assert show.fmt == "skyc" and len(show.drones) == 6 and show.duration == pytest.approx(26.0)
    assert show.drones[0].sample([0.0])[0] == pytest.approx([1.5, 0, 0])


def test_skybrush_csv_zip(tmp_path):
    p = tmp_path / "show.zip"
    with zipfile.ZipFile(p, "w") as z:
        for k in range(2):
            rows = "Time [msec],x [m],y [m],z [m],Red,Green,Blue\n" + "".join(
                f"{t * 100},{k * 2.0},{0.0},{t * 0.1:.2f},255,{t * 10},0\n" for t in range(11))
            z.writestr(f"Drone {k + 1}.csv", rows)
    show = load_show(p)
    assert show.fmt == "csv" and len(show.drones) == 2 and show.duration == pytest.approx(1.0)
    d = show.drones[1]
    assert d.sample([0.5])[0] == pytest.approx([2.0, 0, 0.5])
    assert d.colour([0.55])[0] == pytest.approx([255, 50, 0])


def test_csv_zip_from_a_mac_keeps_drone_numbers(tmp_path):
    """A zipped folder from macOS: Drone 1..12 inside a folder, plus __MACOSX debris.
    Drone N must keep drone ID N (not 1, 10, 11, 12, 2, ...)."""
    p = tmp_path / "flyable.zip"
    with zipfile.ZipFile(p, "w") as z:
        for k in range(1, 13):
            z.writestr(f"flyable/Drone {k}.csv", f"Time [msec],x [m],y [m],z [m],Red,Green,Blue\n0,{k},0,0,255,0,0\n"
                                                 f"1000,{k},0,1,255,0,0\n")
            z.writestr(f"__MACOSX/flyable/._Drone {k}.csv", "\x00\x05junk")
    show = load_show(p)
    assert [d.name for d in show.drones] == [f"Drone {k}" for k in range(1, 13)]
    assert [d.sample([0.0])[0][0] for d in show.drones] == list(range(1, 13))


def test_placement_enu_to_ned():
    pl = Placement()                                             # Blender +X East, +Y North, +Z up
    assert pl.to_ned([1, 2, 3]) == pytest.approx([2, 1, -3])
    assert np.allclose(pl.from_ned(pl.to_ned([[1, 2, 3]])), [[1, 2, 3]])
    turned = Placement(rotate_deg=90)                            # North becomes East
    assert turned.to_ned([0, 1, 0]) == pytest.approx([0, 1, 0], abs=1e-12)


def test_demo_show_passes_and_bad_shows_are_caught(tmp_path, lab):
    site, cams = lab
    show = load_show(write_demo_skyc(tmp_path / "s.skyc"))
    rep = check_show(show, site, cams)
    assert rep.ok and not rep.findings and rep.stats["covered_fraction"] == pytest.approx(1.0)
    # moved 2 m North: drones leave the area and lose coverage
    rep = check_show(show, site, cams, Placement(offset_ned=(2.0, 0, 0)))
    kinds = {f.kind for f in rep.findings}
    assert not rep.ok and {"outside", "coverage"} <= kinds
    # too fast for a cautious airframe, and too close for a 1.5 m separation rule
    rep = check_show(show, site, cams, limits=Limits(max_speed_h_mps=1.0, min_separation_m=1.5, min_home_spacing_m=1.0))
    assert {"speed", "separation"} <= {f.kind for f in rep.findings}
    # two drones on the same path collide
    from idps.show.model import Show
    a = track_from_points("A", [[0, [0, 0, 0], []], [4, [0, 0, 1.5], []]])
    b = track_from_points("B", [[0, [0.3, 0, 0], []], [4, [0.3, 0, 1.5], []]])
    rep = check_show(Show([a, b]), site, cams)
    assert {"separation", "home"} <= {f.kind for f in rep.findings}
    # above the ceiling: higher than the cameras
    hi = track_from_points("C", [[0, [0, 0, 0], []], [6, [0, 0, 3.5], []]])
    rep = check_show(Show([hi]), site, cams, limits=Limits(max_speed_up_mps=5))
    assert any(f.kind == "outside" and "ceiling" in f.message for f in rep.findings)


def test_export_and_player(tmp_path, lab):
    site, cams = lab
    show = load_show(write_demo_skyc(tmp_path / "s.skyc", n=4))
    rep = check_show(show, site, cams)
    man = json.loads(export_px4(show, tmp_path / "out", report=rep).read_text())
    assert man["format"] == "idps-show/1" and len(man["drones"]) == 4 and man["check"]["ok"]
    assert [d["drone_id"] for d in man["drones"]] == [1, 2, 3, 4] and man["drones"][0]["file"] == "drone_001.json"
    doc = json.loads((tmp_path / "out" / man["drones"][0]["file"]).read_text())
    assert doc["format"] == "idps-traj/1" and doc["frame"] == "NED" and doc["rate_hz"] == 20 and doc["drone_id"] == 1
    pl = TrajectoryPlayer(tmp_path / "out" / man["drones"][0]["file"])
    sp = pl.setpoint(10.0)
    truth = Placement().to_ned(show.drones[0].sample([10.0]))[0]
    assert sp.pos == pytest.approx(truth, abs=0.02)                  # 20 Hz samples, linear in between
    assert sp.pos[2] < 0                                            # in the air: D negative
    f = sp.mavlink_fields(123)
    assert f["coordinate_frame"] == 1 and f["type_mask"] == TYPE_MASK_POS_VEL_ACC_YAW == 2048 and f["yaw"] == 0.0
    # streaming: sends at 50 Hz from the start time to the end, via a fake MAVLink link
    sent = []
    fake = types.SimpleNamespace(target_system=1, target_component=1,
                                 mav=types.SimpleNamespace(set_position_target_local_ned_send=lambda *a, **k: sent.append(k)))
    clock = {"t": 1000.0}
    pl.run(fake, start_epoch_s=1000.0, clock=lambda: clock["t"],
           sleep=lambda s: clock.__setitem__("t", clock["t"] + 0.02))
    assert len(sent) == pytest.approx((pl.duration + 2.0) * 50, abs=3)
    assert sent[0]["z"] == pytest.approx(0.0, abs=1e-6)              # starts on its home spot


def test_blender_venue_script_runs(lab, monkeypatch):
    """The generated script runs against a stand-in for Blender's bpy and builds the venue."""
    from idps.show.blender import blender_script, venue_geometry
    site, cams = lab
    geo = venue_geometry(site, cams)
    assert geo["origin"] == [0.0, 0.0, 0.0] and len(geo["cameras"]) == len(cams)
    assert geo["north"][1] > 0 and abs(geo["north"][0]) < 1e-9                # North = Blender +Y
    made = []

    class Objects(list):                       # coll.objects: iterable, with link()
        def link(self, ob):
            made.append(ob)

    class NS(types.SimpleNamespace):
        pass

    coll = NS(name="IDPS venue", objects=Objects())
    mesh = NS(from_pydata=lambda v, e, f: None)
    data = NS(collections=NS(get=lambda n: coll, new=lambda n: coll),
              meshes=NS(new=lambda n: mesh),
              curves=NS(new=lambda n, k: NS(body="", size=1)),
              objects=NS(new=lambda n, d: NS(name=n), remove=lambda *a, **k: None))
    scene = NS(collection=NS(children=["IDPS venue"]), unit_settings=NS(system=""))
    bpy = NS(data=data, context=NS(scene=scene))
    monkeypatch.setitem(sys.modules, "bpy", bpy)
    out = io.StringIO()
    monkeypatch.setattr(sys, "stdout", out)
    exec(compile(blender_script(site, cams), "idps_venue.py", "exec"), {})
    names = [o.name for o in made]
    assert "Performance area" in names and "Proven coverage" in names and "Origin N (north)" in names
    assert sum(n.startswith("Camera ") for n in names) == len(cams)


def test_cli_check_export_blender(tmp_path):
    from idps.show.cli import main
    show = tmp_path / "demo.skyc"
    assert main(["demo", "-o", str(show), "--drones", "4"]) == 0
    site, plan = str(ROOT / "examples/sites/lab.yaml"), str(ROOT / "examples/plans/lab.yaml")
    assert main(["check", str(show), "--site", site, "--plan", plan]) == 0
    assert main(["check", str(show), "--site", site, "--plan", plan, "--offset", "2,0,0"]) == 1
    assert main(["export", str(show), "--site", site, "--plan", plan, "--offset", "2,0,0", "-o", str(tmp_path / "x")]) == 1
    assert not (tmp_path / "x" / "manifest.json").exists()                     # refused
    assert main(["export", str(show), "--site", site, "--plan", plan, "-o", str(tmp_path / "ok")]) == 0
    assert (tmp_path / "ok" / "drone_004.json").exists()
    assert main(["blender-venue", "--site", site, "--plan", plan, "-o", str(tmp_path / "v.py")]) == 0
    assert main(["play", str(tmp_path / "ok" / "drone_001.json"), "--dry-run"]) == 0


def test_fleet_sends_positions_as_vision_with_heading_north(tmp_path, lab):
    """IDPS positions go to PX4 as ODOMETRY marked VISION (EKF2 ignores MOCAP), heading 0."""
    from idps.show.fleet import Fix, Fleet, odometry_fields, parse_links
    site, cams = lab
    show = load_show(write_demo_skyc(tmp_path / "s.skyc", n=3))
    man = export_px4(show, tmp_path / "out", report=check_show(show, site, cams))
    sent = {1: [], 3: []}
    links = {k: types.SimpleNamespace(mav=types.SimpleNamespace(odometry_send=lambda k=k, **f: sent[k].append(f)))
             for k in sent}
    fleet = Fleet(man, links)
    assert fleet.send_positions({1: Fix(np.array([1.0, 2.0, -0.5]))}, now_s=12.5) == 1      # drone 3 not seen
    o = sent[1][0]
    assert not sent[3] and (o["x"], o["y"], o["z"]) == (1.0, 2.0, -0.5) and o["time_usec"] == 12_500_000
    assert o["q"] == [1.0, 0.0, 0.0, 0.0] and o["estimator_type"] == 3 and o["frame_id"] == 1    # North, VISION, NED
    assert o["pose_covariance"][20] > 0 and np.isnan(o["pose_covariance"][15])
    assert odometry_fields([0, 0, 0], 0, yaw_rad=np.pi)["q"][3] == pytest.approx(1.0)
    assert parse_links(["2=udpout:10.0.0.2:14550"]) == {2: "udpout:10.0.0.2:14550"}
    with pytest.raises(ValueError):
        Fleet(man, {9: None})


def test_trj_file_layout_and_crc(tmp_path):
    """The .trj the PX4 module reads: fixed layout, PX4's CRC-32, corruption detected."""
    import struct
    from idps.show.trj import HEADER_SIZE, SAMPLE, crc32, from_bytes, read_trj
    show = load_show(write_demo_skyc(tmp_path / "s.skyc", n=2))
    man = json.loads(export_px4(show, tmp_path / "out").read_text())
    d = man["drones"][1]
    b = (tmp_path / "out" / d["trj"]).read_bytes()
    t = read_trj(tmp_path / "out" / d["trj"])
    assert b[:8] == b"IDPSTRJ1" and struct.unpack_from("<H", b, 12)[0] == 2 == t.drone_id
    assert struct.unpack_from("<I", b, 20)[0] == len(t.pos) and len(b) == HEADER_SIZE + len(t.pos) * SAMPLE.itemsize
    assert d["trj_crc32"] == crc32(b) and d["trj_bytes"] == len(b)
    doc = json.loads((tmp_path / "out" / d["file"]).read_text())
    S = np.asarray(doc["samples"])
    assert np.allclose(t.pos, S[:, 1:4], atol=1e-4) and np.allclose(t.rgb, S[:, 10:13], atol=0.51)

    def px4_crc32(data, c=0):                    # NuttX/PX4 crc32part, bit by bit
        for byte in data:
            c ^= byte
            for _ in range(8):
                c = (c >> 1) ^ 0xEDB88320 if c & 1 else c >> 1
        return c
    assert crc32(b[:300]) == px4_crc32(b[:300])
    bad = bytearray(b)
    bad[HEADER_SIZE + 5] ^= 0xFF
    with pytest.raises(ValueError, match="corrupt"):
        from_bytes(bytes(bad))
    with pytest.raises(ValueError, match="truncated"):
        from_bytes(b[:-40])


class FakeDrone:
    """Answers IDPS show commands like the PX4 module: ACK with state and file CRC."""

    def __init__(self, did, crc, refuse=None):
        self.did, self.crc, self.refuse, self.state = did, crc, refuse, 1
        self.target_system, self.target_component = did, 1
        self.inbox, self.cmds, self.beats = [], [], 0
        self.mav = types.SimpleNamespace(command_int_send=self._cmd, heartbeat_send=self._hb)

    def _hb(self, *a):
        self.beats += 1

    def _cmd(self, target, comp, frame, command, cur, auto, p1, p2, p3, p4, x, y, z):
        self.cmds.append((target, command, p1, p2, p3, p4, x, y))
        action = int(p1)
        if action == 5:
            return
        ok = action != 2 or self.refuse is None
        if action == 2 and ok:
            self.state = 2
        if action == 3:
            self.state = 5
        if not ok:
            self.inbox.append(self._msg("STATUSTEXT", text=f"IDPS show: {self.refuse}"))
        self.inbox.append(self._msg("COMMAND_ACK", command=0, result=0 if ok else 2, progress=self.state,
                                    result_param2=self.crc - (1 << 32) if self.crc >= 1 << 31 else self.crc))

    def _msg(self, kind, **f):
        m = types.SimpleNamespace(**f)
        m.get_type, m.get_srcSystem = (lambda: kind), (lambda: self.did)
        return m

    def recv_match(self, type=None, blocking=True, timeout=None):
        kinds = type if isinstance(type, list) else [type]
        for i, m in enumerate(self.inbox):
            if m.get_type() in kinds:
                return self.inbox.pop(i)
        return None


def test_onboard_commands_start_all_or_none(tmp_path):
    from idps.show import onboard as ob
    show = load_show(write_demo_skyc(tmp_path / "s.skyc", n=3))
    man = export_px4(show, tmp_path / "out")
    crcs = {d["drone_id"]: d["trj_crc32"] for d in json.loads(man.read_text())["drones"]}
    fakes = {k: FakeDrone(k, c) for k, c in crcs.items()}
    drones = ob.drones_from_manifest(man, fakes)
    assert {k: d.crc for k, d in drones.items()} == crcs
    r = ob.command(fakes[2], 2, ob.LOAD, crcs[2])
    assert r.ok and r.state == "ready" and r.crc == crcs[2] and fakes[2].beats == 1
    tgt, cmd, p1, p2, p3, p4, x, y = fakes[2].cmds[-1]
    assert (tgt, cmd, p1, p2) == (2, 0, 1.0, 2.0) and (int(p3) << 16 | int(p4)) == crcs[2]
    assert ob.start_time_fields(12_345_678_901) == (12345, 678901)
    clock = lambda: 1_000_000_000_000                                       # noqa: E731  (1000 s, in ns)
    t, rep = ob.start_all(drones, lead_s=10, clock=clock)
    assert t == 1_010_000_000 and all(r.ok for r in rep.values())
    assert {f.cmds[-1][6:] for f in fakes.values()} == {(1010, 0)}           # one start instant for all
    fakes[3].refuse = "not on its home spot"
    _, rep = ob.start_all(drones, lead_s=10, clock=clock)
    assert not rep[3].ok and rep[3].text == "not on its home spot"
    assert all(f.cmds[-1][2] == ob.ABORT for f in fakes.values())           # nobody flies a partial show
    with pytest.raises(ValueError, match="not in the show"):
        ob.drones_from_manifest(man, {9: FakeDrone(9, 0)})
    sync = ob.ClockSync({1: fakes[1]}, hz=50, clock=clock).start()
    import time
    time.sleep(0.15)
    sync.stop()
    syncs = [c for c in fakes[1].cmds if c[2] == ob.SYNC]
    assert syncs and syncs[0][6:] == (1000, 0)


def test_drone_ids_from_names():
    """The show's "Drone 7" is flown by the drone with ID 7 (its MAV_SYS_ID and marker ID)."""
    from idps.show.export import drone_ids
    from idps.show.model import Show
    tr = lambda n: track_from_points(n, [[0, [0, 0, 0], []], [1, [0, 0, 1], []]])     # noqa: E731
    assert drone_ids(Show([tr("Drone 7"), tr("Drone 12"), tr("drone_3")])) == [7, 12, 3]
    assert drone_ids(Show([tr("Alpha"), tr("Bravo")])) == [1, 2]                         # no numbers: in order
    with pytest.raises(ValueError, match="same drone ID"):
        drone_ids(Show([tr("Drone 2"), tr("Copy of Drone 2")]))
    with pytest.raises(ValueError, match="1..255"):
        drone_ids(Show([tr("Drone 300")]))


def test_preflight_each_drone_on_its_own_spot(tmp_path, lab):
    from idps.show.fleet import Fleet
    site, cams = lab
    show = load_show(write_demo_skyc(tmp_path / "s.skyc", n=4))
    fleet = Fleet(export_px4(show, tmp_path / "out"), {k: object() for k in (1, 2, 3, 4)})
    homes = fleet.homes
    assert fleet.preflight({k: h for k, h in homes.items()}) == []                       # all in place
    swapped = {1: homes[2], 2: homes[1], 3: homes[3], 4: homes[4]}
    probs = fleet.preflight(swapped)
    assert any("Drone 1: on Drone 2's spot" in p for p in probs) and len(probs) == 2
    probs = fleet.preflight({1: homes[1], 2: homes[2], 3: homes[3] + [1.0, 0, 0], 9: homes[4]})
    assert any(p.startswith("Drone 3:") and "m from its spot" in p for p in probs)
    assert any(p.startswith("Drone 4: not seen") for p in probs)
    assert any(p.startswith("Marker 9:") for p in probs)
    one_link = Fleet(tmp_path / "out" / "manifest.json", {1: object()})
    assert any("Drone 2: no link" in p for p in one_link.preflight(dict(homes)))


def _xtjz(drone_id, pos_cm, rgb, encoding=1, signed=False):
    """Build an XTJZ file (idps.show.xtjz) for tests."""
    import struct
    pos_cm, rgb = np.asarray(pos_cm, int), np.asarray(rgb, int)

    def rle(vals, wide):
        out, k = bytearray(), 0
        d = np.diff(np.r_[0, vals])
        while k < len(d):
            n = 1
            while k + n < len(d) and d[k + n] == d[k] and n < 256:
                n += 1
            dv = int(d[k]) if wide else (int(d[k]) + 128) % 256 - 128          # colours wrap around
            out += bytes([n - 1]) + struct.pack("<h" if wide else "<b", dv)
            k += n
        return bytes(out)

    body = b""
    for ch, vals, wide in [(1, pos_cm[:, 0], True), (2, pos_cm[:, 1], True), (3, pos_cm[:, 2], True),
                           (5, rgb[:, 0], False), (6, rgb[:, 1], False), (7, rgb[:, 2], False)]:
        data = (np.asarray(vals, "<i2" if wide else "u1").tobytes() if encoding == 1 else rle(vals, wide))
        body += struct.pack("<IBBH", len(data), ch, encoding, 8) + data
    head = b"XTJZ" + struct.pack("<HHI", 140, 1, len(body))
    head += (bytes(range(128)) if signed else struct.pack("<IHH", drone_id, 20, len(pos_cm)).ljust(128, b"\0"))
    return head + body


def test_xtjz_reader_raw_and_run_length(tmp_path):
    """Per-drone XTJZ files: plain and signed headers, raw and run-length channels, gzipped or not."""
    import gzip

    from idps.show.readers import load_show
    from idps.show.xtjz import read_xtjz
    n = 41
    t = np.arange(n) / 10.0
    pos = np.column_stack([np.round(100 * np.sin(t)), np.round(-250 + 30 * t), np.full(n, -300)])   # cm, NED
    rgb = np.column_stack([np.linspace(0, 255, 18 * n).round(), np.zeros(18 * n), np.full(18 * n, 128)])
    a = read_xtjz(_xtjz(7, pos, rgb, 1), "x.dat")
    b = read_xtjz(gzip.compress(_xtjz(7, pos, rgb, 2, signed=True)), "7.dat.gz")
    assert a.drone_id == 7 and not a.signed and b.drone_id == 7 and b.signed     # signed: ID from the name
    assert a.rate_hz == b.rate_hz == 10 and np.allclose(a.pos_ned, pos / 100) and np.array_equal(a.pos_ned, b.pos_ned)
    assert np.array_equal(a.rgb, rgb) and np.array_equal(b.rgb, rgb) and np.isclose(a.t_rgb[18], a.t[1])
    for k in (3, 1):
        (tmp_path / f"{k}.dat.gz").write_bytes(gzip.compress(_xtjz(k, pos + 50 * k, rgb, 2, signed=True)))
    show = load_show(tmp_path)
    assert show.fmt == "xtjz" and [d.name for d in show.drones] == ["Drone 1", "Drone 3"]
    p0 = show.drones[0].sample([0.0])[0]                        # show frame: x East, y North, z up
    assert np.allclose(p0, [(pos[0, 1] + 50) / 100, (pos[0, 0] + 50) / 100, -(pos[0, 2] + 50) / 100])
    assert np.allclose(show.drones[1].colour([0.0])[0], rgb[0])


def test_check_measures_acceleration_on_sampled_paths():
    """A sampled path is straight between samples: the check still sees its acceleration."""
    from idps.show.check import _accel
    from idps.show.model import track_from_samples
    t = np.arange(0, 4, 0.05)
    P = np.column_stack([np.zeros_like(t), np.zeros_like(t), 0.5 * 3.0 * t ** 2])
    tr = track_from_samples("d", t, P, smooth=False)
    a = _accel(tr, np.array([1.0, 2.0, 3.0]))
    assert np.allclose(a[:, 2], 3.0, atol=1e-6)
    # the default spline through the same samples carries velocity and acceleration itself
    sm = track_from_samples("d", t, P)
    q = np.array([1.03, 2.0, 2.97])
    assert np.allclose(sm.sample(q), P[0] * 0 + np.c_[0 * q, 0 * q, 1.5 * q ** 2], atol=1e-3)
    assert np.allclose(sm.sample(q, 1)[:, 2], 3.0 * q, atol=0.02) and np.allclose(sm.sample(q, 2)[:, 2], 3.0, atol=0.2)
