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
    Drone N must become slot N (not 1, 10, 11, 12, 2, ...)."""
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
    assert man["format"] == "idps-show/1" and len(man["slots"]) == 4 and man["check"]["ok"]
    slot = json.loads((tmp_path / "out" / man["slots"][0]["file"]).read_text())
    assert slot["format"] == "idps-traj/1" and slot["frame"] == "NED" and slot["rate_hz"] == 20
    pl = TrajectoryPlayer(tmp_path / "out" / man["slots"][0]["file"])
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
    assert (tmp_path / "ok" / "slot_04.json").exists()
    assert main(["blender-venue", "--site", site, "--plan", plan, "-o", str(tmp_path / "v.py")]) == 0
    assert main(["play", str(tmp_path / "ok" / "slot_01.json"), "--dry-run"]) == 0


def test_fleet_player_sends_positions_and_setpoints(tmp_path, lab):
    """PC-side flying (no companion): ODOMETRY with yaw 0 for seen drones, setpoints for all."""
    from idps.show.fleet import Fix, FleetPlayer, odometry_fields, parse_links
    site, cams = lab
    show = load_show(write_demo_skyc(tmp_path / "s.skyc", n=3))
    man = export_px4(show, tmp_path / "out", report=check_show(show, site, cams))

    def link():
        log = {"odo": [], "sp": []}
        mav = types.SimpleNamespace(odometry_send=lambda **k: log["odo"].append(k),
                                    set_position_target_local_ned_send=lambda *a, **k: log["sp"].append(k))
        return types.SimpleNamespace(target_system=1, target_component=1, mav=mav, log=log)

    links = {1: link(), 3: link()}
    fleet = FleetPlayer(man, links)
    seen = {1: Fix(np.array([1.0, 2.0, -0.5]))}                          # slot 3 not seen this tick
    clock = {"t": 500.0}
    fleet.run(500.0, positions=lambda: seen, clock=lambda: clock["t"],
              sleep=lambda s: clock.__setitem__("t", clock["t"] + 0.02))
    n = len(links[1].log["sp"])
    assert n == pytest.approx((fleet.t_end + 2.0) * 50, abs=3) and len(links[3].log["sp"]) == n
    assert len(links[1].log["odo"]) == n and not links[3].log["odo"]
    o = links[1].log["odo"][0]
    assert (o["x"], o["y"], o["z"]) == (1.0, 2.0, -0.5) and o["q"] == [1.0, 0.0, 0.0, 0.0]   # facing North
    assert o["frame_id"] == 1 and o["pose_covariance"][20] > 0 and np.isnan(o["pose_covariance"][15])
    assert odometry_fields([0, 0, 0], 0, yaw_rad=np.pi)["q"][3] == pytest.approx(1.0)
    assert parse_links(["2=udpout:10.0.0.2:14550"]) == {2: "udpout:10.0.0.2:14550"}
    with pytest.raises(ValueError):
        FleetPlayer(man, {9: link()})
