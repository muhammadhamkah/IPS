"""Charging pads: batched take-off to the air grid and back (idps.show.pads)."""
from pathlib import Path

import numpy as np
import pytest

from idps.show.pads import PadLayout, check_sequence, landing_legs, launch_legs, smallest_safe_gap
from idps.show.trj import Trj, from_bytes, to_bytes

ROOT = Path(__file__).resolve().parents[1]


def lay(gap=8.0, climb=1.0, xfer=1.0):
    return PadLayout(np.array([[-3.2, -1.0], [-3.2, 0.0]]), 0.2, (1, 2, 3, 4), gap, 60, climb, xfer)


def layered_grid(ids):
    """batch b at b m, rows along North (batch 1 farthest from the pads), 1.2 m along East."""
    P = []
    for d in ids:
        b, col = (d - 1) % 4 + 1, (d - 1) // 4
        P.append([(2.5 - b) * 0.9, (col - 0.5) * 1.2, -float(b)])
    return np.array(P)


def test_drone_id_to_pad_slot_batch():
    L = lay()
    assert L.pad_slot(1) == (1, 1) and L.pad_slot(4) == (1, 4) and L.pad_slot(5) == (2, 1)
    assert L.batch(6) == 2 and L.layer_m(7) == 3
    assert np.allclose(L.home_ned(1), [-3.1, -1.1, 0]) and np.allclose(L.home_ned(8), [-3.3, 0.1, 0])


def test_launch_climbs_straight_up_then_flies_level():
    L = lay()
    g = np.array([1.0, 0.0, -2.0])
    legs = launch_legs(L, 2, g, t_launch=10.0)
    assert legs.t0 == 18.0                                       # batch 2 goes one gap after the launch
    top = legs.at([legs.t0 + legs.durations[0]])[0]
    assert np.allclose(top, [-3.1, -0.9, -2.0])                  # straight up over its pad slot
    assert np.allclose(legs.at([legs.t_end + 5])[0], g)
    land = landing_legs(L, 2, g, t_leave0=100.0, n_batches=4)
    assert land.t0 == 100.0 + 2 * 8.0                            # batch 2 of 4 leaves two gaps after batch 4
    assert np.allclose(land.at([land.t_end])[0], [-3.1, -0.9, 0.0])     # down onto its pad slot


def test_sequence_passes_with_a_safe_gap_and_names_the_smallest():
    ids = list(range(1, 9))
    g = layered_grid(ids)
    assert check_sequence(lay(gap=8.0), ids, g, g)[0] == []
    problems, _ = check_sequence(lay(gap=5.0, climb=0.8), ids, g, g)
    assert any(k == "downwash" for k, *_ in problems)            # 0.2 m slots: the next batch climbs under
    gap = smallest_safe_gap(lay(climb=0.8), ids, g, g)
    assert gap is not None and 5.0 < gap <= 10.0


def test_wrong_layer_height_is_reported():
    ids = [1, 2]
    g = layered_grid(ids)
    g[1, 2] = -1.5                                               # batch 2 should be at 2 m
    problems, _ = check_sequence(lay(), ids, g, g)
    assert problems and problems[0][0] == "layer" and "batch 2" in problems[0][4]


def test_trj_header_carries_the_batch_and_timing():
    n = 3
    t = Trj(6, 20.0, 0.0, np.array([-3.1, -0.9, 0.0]), 0.0, np.zeros((n, 3)), np.zeros((n, 3)), np.zeros((n, 3)),
            np.zeros((n, 3)), batch=2, n_batches=4, batch_gap_s=8.0, climb_speed_mps=1.0, transfer_speed_mps=1.0,
            hold_timeout_s=60)
    r = from_bytes(to_bytes(t))
    assert (r.batch, r.n_batches, r.batch_gap_s, r.climb_speed_mps, r.hold_timeout_s) == (2, 4, 8.0, 1.0, 60.0)
    old = from_bytes(to_bytes(Trj(1, 20.0, 0.0, np.zeros(3), 0.0, *[np.zeros((n, 3))] * 4)))
    assert old.batch == 0                                        # no pads: the show starts on the ground


def _corner_site():
    return {"origin": {"xy": [0, 0], "north_deg": 90},
            "volume": {"polygon": [[-5, -4], [5, -4], [5, 4], [-5, 4]], "z": [0, 5.5]},
            "pads": {"centres": [[-4.0, 3.0], [-3.0, 3.0]]}}


def test_pads_face_the_area_and_step_forward_before_the_long_leg():
    """A pad's front (slots 1-2, the first batches) faces where its drones fly: the middle of the
    area. After climbing, each drone first steps CLEAR_M forward over the pad's empty slots, so a
    long leg's slow start never leaves it over the slot whose batch climbs next."""
    from idps.show.pads import CLEAR_M, launch_legs, pads_from_site, slot_offset_ne
    lay = pads_from_site(_corner_site())
    assert lay.facing_deg == pytest.approx(np.degrees(np.arctan2(3.5, -3.0)) % 360, abs=1e-3)    # to the SE
    a = np.radians(lay.facing_deg)
    fwd = np.array([np.cos(a), np.sin(a)])
    assert slot_offset_ne(1, 0.2, lay.facing_deg) @ fwd == pytest.approx(0.1)            # slot 1 at the front
    assert slot_offset_ne(3, 0.2, lay.facing_deg) @ fwd == pytest.approx(-0.1)
    L = launch_legs(lay, 3, [0.0, 0.0, -3.0])
    home, top, clear, grid = (np.asarray(p) for p in L.points)
    assert np.allclose(top, [home[0], home[1], -3.0]) and np.allclose(clear[:2] - top[:2], fwd * CLEAR_M)
    assert L.durations[1] == pytest.approx(1.5)
    # the example: pads along the South edge face North, as before
    import yaml
    ex = pads_from_site(yaml.safe_load(open(ROOT / "examples/sites/show_8x8.yaml")))
    assert ex.facing_deg == 0.0 and np.allclose(ex.home_ned(1), [-3.1, -2.1, 0])


def test_corner_pads_show_passes_the_take_off_and_landing_checks():
    """The example show built for pads in a corner: no downwash or crowding over the pads at a 5 s gap."""
    import importlib.util
    from idps.show.pads import check_sequence, pads_from_site
    spec = importlib.util.spec_from_file_location("lfp", ROOT / "examples/shows/layers_from_pads.py")
    lfp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(lfp)
    site = _corner_site()
    lay = pads_from_site(site)
    lay.batch_gap_s, lay.climb_speed_mps, lay.transfer_speed_mps = 5.0, 1.5, 1.5
    ch = lfp.build(site, 8)
    t, P, _ = ch.arrays()
    ned = np.stack([P[:, :, 1], P[:, :, 0], -P[:, :, 2]], -1)                # show frame (E, N, up) -> NED
    problems, _ = check_sequence(lay, list(range(1, 9)), ned[:, 0], ned[:, -1])
    assert not problems, problems


def test_trj_keeps_the_pad_facing():
    from idps.show.trj import Trj, from_bytes, to_bytes
    n = 3
    t = Trj(1, 10.0, 0.0, np.zeros(3), 0.0, *[np.zeros((n, 3))] * 4, batch=1, n_batches=4, batch_gap_s=5,
            climb_speed_mps=1.5, transfer_speed_mps=1.5, hold_timeout_s=60, pad_facing_deg=129.81)
    assert from_bytes(to_bytes(t)).pad_facing_deg == pytest.approx(129.81)


def test_no_take_off_before_every_drone_is_tracked_on_its_own_slot(tmp_path):
    """idps-show start --positions: the engine's live log must show every drone on its own spot
    (within 0.1 m: the next slot is 0.2 m away), seen in the last second."""
    import json
    import time
    from idps.show.fleet import Fleet, seen_in_log
    man = tmp_path / "manifest.json"
    man.write_text(json.dumps({"format": "idps-show/1", "drones": [
        {"drone_id": 1, "home_ned": [0.1, -0.1, 0]}, {"drone_id": 2, "home_ned": [0.1, 0.1, 0]}]}))
    log = tmp_path / "positions.csv"
    now = time.time_ns()
    log.write_text("t_ns,marker_id,n,e,d\n"
                   f"{now - 5_000_000_000},2,0.1,0.1,-0.08\n"            # seen, but 5 s ago
                   f"{now - 100_000_000},1,0.1,0.1,-0.08\n")             # drone 1 on drone 2's slot
    seen = seen_in_log(log, now_ns=now)
    assert set(seen) == {1}
    fleet = Fleet(man, {})
    fleet.links = {1: None, 2: None}
    problems = fleet.preflight(seen, tol_m=0.1)
    assert problems == ["Drone 1: on Drone 2's spot (home N 0.10 E -0.10)",
                        "Drone 2: not seen (missing, marker off, or blocked)"]
    with open(log, "a") as f:
        f.write(f"{now},1,0.1,-0.1,-0.08\n{now},2,0.12,0.09,-0.08\n")
    assert fleet.preflight(seen_in_log(log, now_ns=now), tol_m=0.1) == []
