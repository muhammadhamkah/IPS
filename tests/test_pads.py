"""Charging pads: batched take-off to the air grid and back (idps.show.pads)."""
import numpy as np

from idps.show.pads import PadLayout, check_sequence, landing_legs, launch_legs, smallest_safe_gap
from idps.show.trj import Trj, from_bytes, to_bytes


def lay(gap=8.0, climb=1.0):
    return PadLayout(np.array([[-3.2, -1.0], [-3.2, 0.0]]), 0.2, (1, 2, 3, 4), gap, 60, climb, 1.0)


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
    assert np.allclose(land.at([land.t_end])[0], [-3.1, -0.9, -0.3])


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
