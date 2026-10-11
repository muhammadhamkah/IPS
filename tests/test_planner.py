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
    assert any("inside the flight volume" in w for w in site.warnings())


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
