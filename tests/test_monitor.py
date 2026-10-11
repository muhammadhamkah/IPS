"""The brain: pattern detection, station rules, operator decisions, and fixes."""
import numpy as np
import pytest

from idps.monitor.brain import Brain, BrainSettings
from idps.monitor.model import (BLIND_SPOT, CAMERA_MOVED, CAMERA_OFFLINE, FIX_ACCEPTED, LIVED_WITH, OPEN, RESOLVED,
                                StationStatus, TrackSample)
from idps.plan.model import Plan
from idps.plan.site import load_site


@pytest.fixture(scope="module")
def lab():
    return load_site("examples/sites/lab.yaml"), Plan.load("examples/plans/lab.yaml").cameras


def _feed(brain, t, drones, spot, bad_ids, cams, dt=0.1):
    """One tick: drones in bad_ids sit at `spot` seen by one camera; the rest are elsewhere and fine."""
    samples = []
    for d in drones:
        if d in bad_ids:
            samples.append(TrackSample(t, d, np.asarray(spot) + 0.1 * d, frozenset(cams[:1])))
        else:
            samples.append(TrackSample(t, d, np.array([-2.0, 2.0, 1.5]), frozenset(cams)))
    return brain.feed(samples, [StationStatus(t, c, True, 120.0, 0.0) for c in cams])


def _loss_burst(brain, t0, drone, spot, cams, dur=0.6):
    """One drone loses tracking at `spot` for `dur` s, then recovers. Returns the end time."""
    t = t0
    out = []
    while t < t0 + dur:
        out += _feed(brain, t, [1, 2, 3], spot, {drone}, cams)
        t += 0.1
    out += _feed(brain, t, [1, 2, 3], spot, set(), cams)
    return t + 0.1, out


def test_one_off_loss_is_not_a_pattern(lab):
    site, cams = lab
    brain = Brain(site, cams)
    ids = [c.id for c in cams]
    t, ch = _loss_burst(brain, 0.0, 1, [1, 1, 1.5], ids)
    for _ in range(30):
        ch += _feed(brain, t, [1, 2, 3], None, set(), ids)
        t += 0.1
    assert len(brain.events) == 1 and not brain.issues and not ch


def test_repeated_losses_from_several_drones_raise_one_issue(lab):
    site, cams = lab
    brain = Brain(site, cams)
    ids = [c.id for c in cams]
    t, changes = 0.0, []
    for k in range(6):
        t, ch = _loss_burst(brain, t, 1 + k % 2, [1, 1, 1.5], ids)
        changes += ch
        t += 1.0
    issues = [i for i in brain.issues.values() if i.kind == BLIND_SPOT]
    assert len(issues) == 1 and ("new" in [c for _, c in changes])
    iss = issues[0]
    assert np.linalg.norm(iss.location[:2] - [1.15, 1.15]) < 1.0 and iss.count >= 5 and iss.drones == {1, 2}
    assert iss.severity == "critical"          # seen by 1 camera: tracking lost, not merely degraded
    assert brain.pending() == [iss]


def test_camera_offline_raised_and_resolved(lab):
    from idps.plan.advisor import Advisor
    site, cams = lab
    brain = Brain(site, cams)
    ids = [c.id for c in cams]
    worst = Advisor(site, cams).health().worst_camera       # the camera whose loss hurts most
    ids.remove(worst)
    ids.insert(1, worst)
    t = 0.0
    for _ in range(5):
        brain.feed([], [StationStatus(t, c, True, 120.0) for c in ids])
        t += 0.1
    ch = []
    for _ in range(15):                           # C002 goes quiet for 1.5 s
        ch += brain.feed([], [StationStatus(t, c, c != ids[1], 120.0) for c in ids])
        t += 0.1
    off = [i for i in brain.issues.values() if i.kind == CAMERA_OFFLINE]
    assert len(off) == 1 and off[0].cameras == [ids[1]] and off[0].severity == "critical"
    brain.analyse_pending()
    assert off[0].fixes and off[0].fixes[0].kind == "reaim"          # stop-gap: re-aim a neighbour
    ch = brain.feed([], [StationStatus(t, c, True, 120.0) for c in ids])
    assert off[0].state == RESOLVED and (off[0], "resolved") in ch


def test_knocked_camera_restore(lab):
    site, cams = lab
    brain = Brain(site, cams)
    ids = [c.id for c in cams]
    brain.feed([], [StationStatus(0.0, c, True, 120.0, 3.0 if c == ids[2] else 0.02) for c in ids])
    moved = [i for i in brain.issues.values() if i.kind == CAMERA_MOVED]
    assert len(moved) == 1 and moved[0].action == "restore" and moved[0].severity == "critical"
    brain.accept(moved[0].id)
    brain.feed([], [StationStatus(0.1, c, True, 120.0, 0.02) for c in ids])
    assert moved[0].state == RESOLVED


def test_live_with_suppresses_until_it_gets_worse(lab):
    site, cams = lab
    brain = Brain(site, cams, BrainSettings(window_s=600))
    ids = [c.id for c in cams]
    t = 0.0
    for k in range(6):                            # about one loss every 10 s
        t, _ = _loss_burst(brain, t, 1 + k % 2, [1, 1, 1.5], ids)
        t += 9.0
    iss = next(i for i in brain.issues.values() if i.kind == BLIND_SPOT)
    brain.live_with(iss.id)
    assert iss.state == LIVED_WITH and iss.lived_rate > 0
    changes = []
    for k in range(4):                            # same rate: stays quiet
        t, ch = _loss_burst(brain, t, 1 + k % 2, [1, 1, 1.5], ids)
        changes += ch
        t += 9.0
    assert iss.state == LIVED_WITH and not [c for c in changes if c[1] == "worse"]
    for k in range(30):                           # much more often: alert again
        t, ch = _loss_burst(brain, t, 1 + k % 3, [1, 1, 1.5], ids)
        changes += ch
        t += 0.3
    assert iss.state == OPEN and (iss, "worse") in changes


def test_accepted_fix_confirmed_when_quiet(lab):
    site, cams = lab
    brain = Brain(site, cams, BrainSettings(quiet_s=5.0, min_passes=10))
    ids = [c.id for c in cams]
    t = 0.0
    for k in range(6):
        t, _ = _loss_burst(brain, t, 1 + k % 2, [1, 1, 1.5], ids)
    iss = next(i for i in brain.issues.values() if i.kind == BLIND_SPOT)
    brain.accept(iss.id)
    assert iss.state == FIX_ACCEPTED and not brain.events
    ch = []
    for _ in range(80):                            # drones fly through the spot, all well seen
        s = [TrackSample(t, d, np.array([1.1, 1.1, 1.5]), frozenset(ids)) for d in (1, 2)]
        ch += brain.feed(s, [StationStatus(t, c, True, 120.0) for c in ids])
        t += 0.1
    assert iss.state == RESOLVED and (iss, "resolved") in ch
    assert "Fix confirmed" in iss.history[-1][1]


def test_simulated_obstruction_found_and_fix_helps(lab):
    """End to end: a simulated show, an obstruction hung where drones fly, the brain finds the
    pattern by itself, and its recommendation reduces the problem spots it measured."""
    from idps.monitor.simsource import SimShow
    site, cams = lab
    show = SimShow(site, cams, n_drones=6, seed=4)
    brain = Brain(site, cams)
    for _ in range(100):
        brain.feed(*show.step(0.1))
    assert not brain.issues                      # the plan is good: nothing to report
    show.obstruct()
    for _ in range(1200):
        brain.feed(*show.step(0.1))
        if brain.pending():
            break
    iss = brain.pending()[0]
    assert iss.kind == BLIND_SPOT and iss.count >= 5
    brain.analyse_pending()
    assert iss.fixes, iss.fix_text
    assert iss.fixes[-1].failing_after < iss.fixes[0].failing_before


def test_segments_hit_boxes():
    from idps.sim.camera import segments_hit_boxes
    box = [((4, -1, -1), (5, 1, 1))]
    hit = segments_hit_boxes([0, 0, 0], [[10, 0, 0], [10, 5, 0], [1, 0, 0], [10, 0, 0.5]], box)
    assert hit.tolist() == [True, False, False, True]
    assert segments_hit_boxes([0, 0, 0], [[0, 0, 10]], [((-1, -1, 4), (1, 1, 5))]).tolist() == [True]


def test_advisor_focus_counts_whole_volume(lab):
    """A focused fix must not break coverage elsewhere: points outside the focus still count."""
    from idps.plan.advisor import Advisor, Focus
    site, cams = lab
    blocked = frozenset({"C004", "C005", "C007"})
    adv = Advisor(site, cams, focus=Focus((0.5, 0.5, 1.5), 2.0, blocked, 3))
    assert adv.mask.sum() < len(adv.pts)
    k = next(i for i, c in enumerate(cams) if c.id == "C004")
    inside = np.nonzero(adv.mask)[0]
    assert not adv.vis[k, inside].any()          # blocked camera removed inside the focus only
    assert adv.vis[k, ~adv.mask].any()
    for r in adv.recommend(top=3):
        assert r.failing_after <= r.failing_before
