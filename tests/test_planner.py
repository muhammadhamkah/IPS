from pathlib import Path

import numpy as np
import pytest

from idps.plan.candidates import generate_candidates
from idps.plan.model import Plan, install_angles
from idps.plan.planner import evaluate, optimise
from idps.plan.site import load_site, site_from_dict

ROOT = Path(__file__).resolve().parents[1]
LAB = ROOT / "examples" / "sites" / "lab.yaml"


def small_site(**over):
    d = {
        "name": "test",
        "volume": {"min": [-2, -2, 0], "max": [2, 2, 2]},
        "lenses_mm": [2.8],
        "requirements": {"min_cameras": 3, "p95_mm": 10, "noise": "tight"},
        "visibility": {"max_range_m": 15},
        "evaluation": {"spacing_m": 1.0, "inset_m": 0.25},
        "trusses": [{"id": "T1", "start": [-3, 0], "end": [3, 0], "height_m": 3.0,
                     "usable": [[0, 2], [4, 6]]}],
        "tripod_zones": [{"id": "N", "rect": [-3, 2.5, 3, 3], "height_m": [1.5, 2.5]},
                         {"id": "S", "rect": [-3, -3, 3, -2.5], "height_m": [1.5, 2.5]}],
        "no_go_zones": [[-1, 2.5, 1, 3]],
        "candidates": {"truss_step_m": 1, "tripod_step_m": 1, "tripod_heights_m": [2.0],
                       "tripod_elevations_deg": [-20, 0], "azimuth_step_deg": 45},
        "optimiser": {"max_cameras": 20},
    }
    d.update(over)
    return site_from_dict(d)


def test_candidates_respect_usable_segments_and_no_go():
    site = small_site()
    c = generate_candidates(site)
    truss = c.mount_type == "truss"
    s = c.along_m[truss]
    assert ((s <= 2 + 1e-9) | (s >= 4 - 1e-9)).all()          # nothing in the 2..4 m gap
    assert np.allclose(c.pos[truss, 2], 3.0 - site.candidates.truss_drop_m)
    trip = c.pos[c.mount_type == "tripod"]
    in_no_go = (trip[:, 0] >= -1) & (trip[:, 0] <= 1) & (trip[:, 1] >= 2.5)
    assert not in_no_go.any()
    assert set(np.round(trip[:, 2], 3)) == {2.0}


def test_site_warnings():
    site = small_site(trusses=[{"id": "low", "start": [0, 0], "end": [1, 0], "height_m": 1.0}])
    assert any("below the flight ceiling" in w for w in site.warnings())


def test_cameras_always_above_the_drones():
    """Decision D17: drones never fly higher than a camera. No candidate sits below the
    volume top, none looks up, and short masts / low trusses are left out with a warning."""
    from idps.plan.candidates import generate_candidates
    site = small_site(trusses=[{"id": "low", "start": [-3, 0], "end": [3, 0], "height_m": 1.5},
                               {"id": "high", "start": [-3, 1], "end": [3, 1], "height_m": 3.0}],
                      tripod_zones=[{"id": "short", "rect": [-3, 2.5, 3, 3], "height_m": [1.0, 1.8]},
                                    {"id": "tall", "rect": [-3, -3, 3, -2.5], "height_m": [1.0, 4.0]}],
                      candidates={"truss_step_m": 1, "tripod_step_m": 1, "tripod_heights_m": [1.5, 3.0],
                                  "tripod_elevations_deg": [20, 0, -20], "azimuth_step_deg": 90})
    c = generate_candidates(site)
    assert (c.pos[:, 2] >= site.volume.z_max - 1e-9).all()
    assert (c.el <= 0).all()
    assert set(c.mount_id) == {"high", "tall"}
    assert sorted(set(c.pos[c.mount_id == "tall", 2])) == [2.0, 3.0]      # from the ceiling up
    w = " ".join(site.warnings())
    assert "truss low" in w and "tripod zone short" in w
    site2 = small_site(trusses=[], tripod_zones=[{"id": "short", "rect": [-3, 2.5, 3, 3], "height_m": [1, 1.8]}])
    with pytest.raises(ValueError, match="above the drones"):
        generate_candidates(site2)


def test_optimise_small_site_meets_requirements_and_roundtrips(tmp_path):
    site = small_site()
    plan, ev = optimise(site, log=lambda *a: None)
    assert plan.cameras
    assert ev.summary["fraction_meeting_all_requirements"] == pytest.approx(1.0)
    assert (ev.n_cams >= 3).all()
    ids = [c.id for c in plan.cameras]
    assert ids == sorted(ids) and len(set(ids)) == len(ids)
    plan.save(tmp_path / "plan.yaml")
    plan.write_csv(tmp_path / "cameras.csv")
    back = Plan.load(tmp_path / "plan.yaml")
    ev2 = evaluate(site, back.cameras)
    assert np.allclose(ev2.p95_mm, ev.p95_mm)


def test_removing_a_camera_is_reported_as_critical():
    site = small_site()
    plan, ev = optimise(site, log=lambda *a: None)
    k = int(np.argmax(ev.critical))
    ev2 = evaluate(site, plan.cameras[:k] + plan.cameras[k + 1:])
    assert ev2.meets.sum() == ev.meets.sum() - ev.critical[k]


@pytest.mark.parametrize("az,base,pan", [(0, 0, 0), (30, 45, -15), (350, 0, -10), (200, 180, 20)])
def test_install_angles(az, base, pan):
    b, p, t = install_angles(az, -30)
    assert np.isclose(b, base) and np.isclose(p, pan) and t == -30


def test_lab_example_loads_and_plans():
    site = load_site(LAB)
    assert not site.warnings()
    plan, ev = optimise(site, log=lambda *a: None)
    assert ev.summary["fraction_meeting_all_requirements"] == pytest.approx(1.0)


def test_report_writes(tmp_path):
    pytest.importorskip("matplotlib")
    from idps.plan.report import write_report
    site = small_site()
    plan, ev = optimise(site, log=lambda *a: None)
    write_report(tmp_path / "r.html", site, plan, ev)
    html = (tmp_path / "r.html").read_text()
    assert "Camera install table" in html and "data:image/png;base64" in html


def test_polygon_volume_with_offset_tripod_zone():
    site = small_site(
        volume={"polygon": [[0, 0], [8, 0], [8, 4], [4, 4], [4, 8], [0, 8]], "z": [0, 3]},
        trusses=[],
        tripod_zones=[{"id": "ring", "offset_m": [1, 2], "height_m": [2, 3], "edges": [0, 3, 4, 5]}],
        no_go_zones=[],
        visibility={"max_range_m": 20},
        evaluation={"spacing_m": 1.5, "inset_m": 0.25},
        candidates={"tripod_step_m": 1.5, "tripod_heights_m": [2.5], "tripod_elevations_deg": [-20, 0],
                    "azimuth_step_deg": 45},
    )
    assert np.isclose(site.volume.floor_area_m2, 48)
    from idps.coverage.volume import distance_to_polygon
    from idps.plan.candidates import tripod_positions
    xy = tripod_positions(site, site.tripod_zones[0])
    d, e, _ = distance_to_polygon(xy, site.volume.polygon)
    assert (d >= 1 - 1e-6).all() and (d <= 2 * 1.5).all()   # corner points sit on the bisector
    assert not np.isin(e, [1, 2]).any()                          # excluded edges get no tripods
    assert not site.volume.contains_xy(xy).any()
    plan, ev = optimise(site, log=lambda *a: None)
    assert ev.summary["fraction_meeting_all_requirements"] == pytest.approx(1.0)
    assert all("edge E" in c.install_note for c in plan.cameras)


def test_design_margin_is_used_and_validated():
    site = small_site(design={"min_cameras": 4, "p95_mm": 7})
    plan, ev = optimise(site, log=lambda *a: None)
    assert (ev.n_cams >= 4).all()
    assert ev.summary["design_margin"] == {"min_cameras": 4, "p95_mm": 7.0}
    with pytest.raises(ValueError):
        small_site(design={"min_cameras": 2})


def test_min_spacing_between_mounts():
    site = small_site(candidates={"truss_step_m": 0.5, "tripod_step_m": 0.5, "tripod_heights_m": [2.0],
                                  "tripod_elevations_deg": [-20, 0], "azimuth_step_deg": 45,
                                  "max_cameras_per_mount": 1, "min_camera_spacing_m": 2.0})
    plan, ev = optimise(site, log=lambda *a: None)
    P = np.array([c.position_m for c in plan.cameras])
    d = np.linalg.norm(P[:, None] - P[None], axis=-1)
    np.fill_diagonal(d, np.inf)
    assert d.min() >= 2.0 - 1e-9


def test_measurement_chains_truss_and_corner_tripod():
    from idps.plan.measure import chains, describe
    from idps.plan.model import PlannedCamera
    poly = [[0, 0], [10, 0], [10, 6], [0, 6]]
    trusses = [{"id": "T1", "start": [-1, 7], "end": [11, 7]}]
    cams = [PlannedCamera("C1", "truss", "T1", [2, 7, 9], 0, -60, 2.8, along_truss_m=3.0),
            PlannedCamera("C2", "truss", "T1", [5, 7, 9], 0, -60, 2.8, along_truss_m=6.0),
            PlannedCamera("C3", "tripod", "s", [-2, -2, 3], 45, 0, 2.8),       # corner, beyond V0
            PlannedCamera("C4", "tripod", "s", [4, -1.5, 3], 90, 0, 2.8)]
    chs = {c.ref: c for c in chains(trusses, poly, cams)}
    t = chs["T1"]
    assert [s for s, _, _ in t.stations] == [3.0, 6.0] and t.segments() == [3.0, 3.0]
    e0 = chs["E0 (V0->V1)"]
    assert e0.stations[0][0] == pytest.approx(-2.0) and e0.stations[0][2] == pytest.approx(2.0)   # 2 before V0, 2 out
    assert e0.stations[1][0] == pytest.approx(4.0) and e0.stations[1][2] == pytest.approx(1.5)
    assert "before V0" in describe(e0)


def test_cli_writes_measurements(tmp_path):
    from idps.plan.cli import main
    assert main(["optimise", str(LAB), "-o", str(tmp_path)]) == 0
    rows = (tmp_path / "measurements.csv").read_text().splitlines()
    assert rows[0].startswith("reference,measure_from") and len(rows) > 1
    assert "Install measurements" in (tmp_path / "report.html").read_text()


def _short_venue():
    d = {
        "name": "short", "volume": {"polygon": [[-10, -5], [-10, 7], [17.5, 7], [17.5, -5]], "z": [0, 10]},
        "lenses_mm": [2.8], "requirements": {"min_cameras": 3, "p95_mm": 10, "noise": "tight"},
        "design": {"min_cameras": 4, "p95_mm": 7}, "visibility": {"max_range_m": 80},
        "evaluation": {"spacing_m": [2, 2, 2], "inset_m": 0.5},
        "trusses": [{"id": "T1", "start": [-9.5, 7.5], "end": [17.5, 7.5], "height_m": 11.5},
                    {"id": "T2", "start": [-11, -5.5], "end": [18, -5.5], "height_m": 11.5}],
        "candidates": {"truss_step_m": 3, "tripod_step_m": 3, "tripod_heights_m": [3, 10, 18],
                       "truss_elevations_deg": [-90, -60, -35],          # steep only: top layer comes up short
                       "tripod_elevations_deg": [0, -15, -30, -50], "azimuth_step_deg": 30,
                       "max_cameras_per_mount": 1, "min_camera_spacing_m": 1.5},
        "optimiser": {"max_cameras": 200},
    }
    return d


def test_advisor_recommendations_are_real_improvements():
    from idps.plan.advisor import Advisor, apply
    d = _short_venue()
    plan, ev = optimise(site_from_dict(d), log=lambda *a: None)
    d["tripod_zones"] = [{"id": "s", "offset_m": [1, 3], "height_m": [2, 20]}]
    site = site_from_dict(d)
    adv = Advisor(site, plan.cameras)
    h = adv.health()
    assert h.failing > 0 and h.worst_camera is not None
    recs = adv.recommend(top=6)
    assert recs and recs[0].points_fixed > 0
    for r in recs:                                    # every claim checks out when applied
        e2 = evaluate(site, apply(plan.cameras, r))
        assert int((~e2.meets).sum()) == r.failing_after
        assert r.detail
    kinds = {r.kind for r in recs}
    assert "add" in kinds
    add = next(r for r in recs if r.kind == "add")
    assert "tripod" in add.detail and " m out from the edge" in add.detail
    P = np.array([c.position_m for c in plan.cameras])
    assert np.linalg.norm(P - np.array(add.new_camera.position_m), axis=1).min() >= 1.5 - 1e-9   # spacing kept


def test_auto_fix_reaches_full_coverage():
    from idps.plan.advisor import auto_fix
    d = _short_venue()
    plan, _ = optimise(site_from_dict(d), log=lambda *a: None)
    d["tripod_zones"] = [{"id": "s", "offset_m": [1, 3], "height_m": [2, 20]}]
    site = site_from_dict(d)
    steps, cams = auto_fix(site, plan.cameras)
    assert steps and steps[-1].failing_after == 0
    assert all(a.failing_after == b.failing_before for a, b in zip(steps, steps[1:]))
    assert evaluate(site, cams).summary["fraction_meeting_all_requirements"] == pytest.approx(1.0)


def test_reaim_prefers_servo_move_from_existing_base():
    from idps.plan.advisor import SERVO_PAN_LIMIT_DEG, Advisor
    d = _short_venue()
    plan, _ = optimise(site_from_dict(d), log=lambda *a: None)
    adv = Advisor(site_from_dict(d), plan.cameras)
    for r in adv.recommend(top=10, kinds=("reaim",)):
        old = next(c for c in plan.cameras if c.id == r.camera_id)
        if r.new_camera.base_heading_deg == old.base_heading_deg:
            assert abs(r.new_camera.pan_deg) <= SERVO_PAN_LIMIT_DEG and "No physical move" in r.detail
        else:
            assert "Turn the gimbal base" in r.detail


def test_shallow_truss_tilts_cover_the_top_layer():
    """Cameras hang just above the drones (D17); without near-level tilts the top of the volume
    falls above every view cone. The default tilts include them."""
    d = _short_venue()
    d["candidates"].pop("truss_elevations_deg", None)
    d["candidates"]["tripod_elevations_deg"] = [0, -15, -30, -50]
    plan, ev = optimise(site_from_dict(d), log=lambda *a: None)
    assert ev.summary["fraction_meeting_all_requirements"] == pytest.approx(1.0)


def test_fine_check_finds_gaps_between_grid_samples():
    """The coarse planning grid can miss a thin gap; the dense re-check after planning finds it,
    and the planner re-plans until it passes."""
    from idps.plan.planner import fine_check, fine_check_text
    d = {"name": "gap", "volume": {"polygon": [[-15, -8], [15, -8], [15, 8], [-15, 8]], "z": [0, 8]},
         "lenses_mm": [2.8], "requirements": {"min_cameras": 4, "p95_mm": 10, "noise": "tight"},
         "design": {"min_cameras": 4, "p95_mm": 7}, "visibility": {"max_range_m": 80},
         "evaluation": {"spacing_m": [2, 2, 2], "inset_m": 0.5},
         "trusses": [{"id": "T1", "start": [-15.5, 8], "end": [15.5, 8], "height_m": 11.5},
                     {"id": "T2", "start": [-15.5, -8], "end": [15.5, -8], "height_m": 11.5}],
         "candidates": {"truss_step_m": 3, "truss_elevations_deg": [-90, -60, -35], "azimuth_step_deg": 30,
                        "max_cameras_per_mount": 1, "min_camera_spacing_m": 1.5},
         "optimiser": {"max_cameras": 200}}
    plan, ev = optimise(site_from_dict(d), log=lambda *a: None)
    fc = plan.summary["fine_check"]
    assert fc["spacing_m"] <= 0.5 and fc["points"] > 10 * ev.summary["evaluation_points"]
    # steep tilts leave thin gaps up top that the 2 m grid misses: the fine check finds them (also by
    # zooming to 1 cm at view edges), the planner re-plans with them, and says so when no mount can help
    assert plan.summary["optimiser"]["fine_check_rounds"] > 1
    assert fc["zoom_failing"] > 0 and fc["resolution_m"] == 0.01 and not fc["guaranteed"]
    assert "falls short" in fine_check_text(fc) and fc["worst_spots"]
    # with near-level tilts allowed, the same venue is proven down to 1 cm
    d["candidates"]["truss_elevations_deg"] = [-90, -60, -35, -15, 0]
    plan2, _ = optimise(site_from_dict(d), log=lambda *a: None)
    fc2 = plan2.summary["fine_check"]
    assert fc2["guaranteed"] and fc2["failing"] == 0 and "proven" in fine_check_text(fc2)
    lab = load_site(ROOT / "examples" / "sites" / "lab.yaml")
    ok = fine_check(lab, Plan.load(ROOT / "examples" / "plans" / "lab.yaml").cameras)
    assert ok["failing"] == 0 and "every point meets" in fine_check_text(ok)


def test_guaranteed_visibility_is_conservative():
    """A ball counted as fully seen really is: random points inside it are all visible."""
    from idps.coverage.fisher import contributions, guaranteed_visible
    site = load_site(ROOT / "examples" / "sites" / "lab.yaml")
    cams = Plan.load(ROOT / "examples" / "plans" / "lab.yaml").cameras
    pos = np.array([c.position_m for c in cams])
    R = np.array([c.R for c in cams])
    f = np.array([site.sensor.f_px(c.lens_mm) for c in cams])
    rng = np.random.default_rng(0)
    centres = rng.uniform([-3, -3, 0.2], [3, 3, 3], size=(300, 3))
    r = 0.3
    sure = guaranteed_visible(pos, R, f, centres, site.sensor, site.limits, r)
    assert sure.any()
    for _ in range(20):
        u = rng.normal(size=(len(centres), 3))
        u *= (r * rng.uniform(0, 1, (len(centres), 1)) ** (1 / 3)) / np.linalg.norm(u, axis=1, keepdims=True)
        vis, _ = contributions(pos, R, f, centres + u, site.sensor, site.requirements.noise, site.limits)
        assert not (sure & ~vis).any()
