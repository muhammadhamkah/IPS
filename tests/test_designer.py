"""Show designer: scenes on a wall, drones spread evenly over a shape, safe flights between scenes."""
from pathlib import Path

import numpy as np
import pytest
import yaml

from idps.show.designer import (RATE_HZ, Design, DesignError, Scene, hungarian, plan_move, sample_mask, shape_points,
                                _closest, _move)

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def site():
    return yaml.safe_load((ROOT / "examples" / "sites" / "show_8x8.yaml").read_text())


def test_hungarian_finds_the_cheapest_assignment():
    rng = np.random.default_rng(3)
    import itertools
    for n in (1, 2, 5, 7):
        c = rng.random((n, n))
        best = min(sum(c[i, p[i]] for i in range(n)) for p in itertools.permutations(range(n)))
        got = hungarian(c)
        assert sorted(got) == list(range(n)) and c[np.arange(n), got].sum() == pytest.approx(best)


def test_drones_are_spread_evenly_over_a_shape():
    """20 places on a filled disc and on a ring: all on the shape, none bunched (the closest two
    are at least half as far apart as an even spread would be)."""
    yy, xx = np.mgrid[0:200, 0:200]
    r = np.hypot(xx - 100, yy - 100)
    for mask, even in ((r < 80, np.sqrt(np.pi * 80 ** 2 / 20)), ((r > 70) & (r < 80), 2 * np.pi * 75 / 20)):
        p = sample_mask(mask, 20)
        assert mask[(199 - p[:, 1]).astype(int), p[:, 0].astype(int)].all()
        gaps = np.linalg.norm(p[:, None] - p[None], axis=2)[np.triu_indices(20, 1)]
        assert gaps.min() > 0.5 * even
    with pytest.raises(DesignError, match="too small"):
        sample_mask(r < 2, 20)


def test_shapes_text_and_images(tmp_path):
    for kind in ("grid", "circle", "heart"):
        uv, rgb = shape_points(kind, 20)
        assert uv.shape == (20, 2) and rgb is None and np.ptp(uv, axis=0).max() == pytest.approx(1.0)
    uv, _ = shape_points("text", 20, text="HI")
    assert uv.shape == (20, 2) and np.ptp(uv[:, 0]) > 0.9             # wider than tall, both letters used
    assert (uv[:, 0] < -0.1).sum() >= 6 and (uv[:, 0] > 0.1).sum() >= 6
    with pytest.raises(DesignError, match="type the text"):
        shape_points("text", 20, text=" ")
    # an image: a red square on white, left half; a blue one on the right. The places take its colours.
    from PIL import Image
    a = np.full((120, 200, 3), 255, np.uint8)
    a[30:90, 20:80] = (255, 0, 0)
    a[30:90, 120:180] = (0, 0, 255)
    path = tmp_path / "two.png"
    Image.fromarray(a).save(path)
    uv, rgb = shape_points("image", 20, image=str(path))
    left = uv[:, 0] < 0
    assert 8 <= left.sum() <= 12 and (rgb[left] == (255, 0, 0)).all() and (rgb[~left] == (0, 0, 255)).all()
    with pytest.raises(DesignError, match="no shape found"):
        Image.fromarray(np.full((50, 50, 3), 200, np.uint8)).save(tmp_path / "flat.png")
        shape_points("image", 20, image=str(tmp_path / "flat.png"))


def test_a_move_that_crosses_is_made_safe():
    """Two rows that swap sides, each drone to the place opposite (given, not chosen): flown
    straight they meet in the middle; planned, they keep the minimum distance."""
    n = 6
    start = np.array([[x, 0.0, 3.0] for x in np.arange(n) * 1.0])
    target = start[::-1].copy()
    assert _closest(_move(start, target, 4.0, np.zeros(n)))[0] < 0.1
    order, delays, bows, worst = plan_move(start, target, 4.0, 0.8, assign=False, z_range=(0.5, 9.0))
    assert list(order) == list(range(n)) and worst >= 0.8 - 1e-3
    assert (delays > 0).any() or np.abs(bows).max() > 0
    # with the places free to choose nobody needs to move at all
    order, delays, bows, worst = plan_move(start, target, 4.0, 0.8, assign=True)
    assert list(order) == list(range(n))[::-1] and not delays.any() and not bows.any()


def test_a_design_compiles_to_a_safe_show(site, tmp_path):
    """Grid -> text -> circle with a wave -> heart: starts and ends in the pads' layered grid, the
    drones keep the minimum distance all the way, stay in the area, and the show file is read
    back and passes the take-off and landing checks from the charging pads."""
    d = Design(n=12, scenes=[Scene("grid"), Scene("text", text="HI"), Scene("circle", wave_m=0.25, hold_s=6),
                             Scene("heart", spin_deg=45, rgb=(255, 0, 80))])
    c = d.compile(site)
    assert [f for f in c.findings if f[0] == "error"] == []
    assert c.stats["min_separation_m"] >= 0.8 - 1e-3 and c.stats["max_accel_mps2"] < 4.0
    assert c.P.shape == (12, len(c.t), 3) and np.allclose(c.P[:, 0], c.P[:, -1])      # back where it began
    assert [s[1] for s in c.steps].count("scene") == 6 and c.steps[-2][0] == "back to the grid"
    # the wave: the circle's drones go up and down by the amplitude, and are back level when it ends
    t0, t1 = c.scene_span(2)
    k0, k1 = int(round(t0 * RATE_HZ)), int(round(t1 * RATE_HZ))
    z = c.P[:, k0:k1 + 1, 2]
    assert 0.2 < np.abs(z - z[:, :1]).max() <= 0.25 + 1e-6 and np.allclose(z[:, -1], z[:, 0], atol=1e-6)
    # colours: the heart's colour while it holds
    t0, t1 = c.scene_span(3)
    assert (c.C[:, int((t0 + 1) * RATE_HZ)] == (255, 0, 80)).all()
    path = c.write_csv_zip(tmp_path / "show.zip")
    from idps.show.check import check_show
    from idps.show.pads import fit_layers, pads_from_site
    from idps.show.placement import Placement
    from idps.show.readers import load_show
    from idps.show.export import drone_ids
    from idps.plan.site import load_site
    show = load_show(path)
    assert len(show.drones) == 12 and show.duration == pytest.approx(c.t[-1], abs=0.1)
    pads = pads_from_site(site)
    rep = check_show(show, load_site(ROOT / "examples" / "sites" / "show_8x8.yaml"), None, Placement(), pads=pads)
    kinds = {f.kind for f in rep.findings if f.severity == "error"}
    assert kinds <= {"downwash", "separation"}, kinds          # (a wall stacks drones; IDPS's own spacing is 1 m)
    assert rep.stats_pads["take-off closest"] >= 0.5


def test_what_cannot_be_built_is_said(site):
    with pytest.raises(DesignError, match="charging pads"):
        Design(n=4, scenes=[Scene("grid")]).compile({k: v for k, v in site.items() if k != "pads"})
    with pytest.raises(DesignError, match="charging pads"):
        Design(n=40, scenes=[Scene("grid")]).compile(site)
    c = Design(n=12, scenes=[Scene("grid", width_m=1.0)]).compile(site)          # 12 drones in 1 m
    assert any("m apart: make it" in f[1] for f in c.findings) and not c.ok
    c = Design(n=12, scenes=[Scene("grid", height_m=20.0)]).compile(site)
    assert any("above 5.5 m" in f[1] for f in c.findings)
    c = Design(n=12, scenes=[Scene("grid", east_m=30.0)]).compile(site)
    assert any("leave the performance area" in f[1] for f in c.findings)
    c = Design(n=12, scenes=[Scene("text", text="")]).compile(site)               # the other scenes still build
    assert any("type the text" in f[1] for f in c.findings) and len(c.t) > 10


def test_a_design_is_saved_and_opened(tmp_path, site):
    d = Design(n=12, min_sep_m=0.9, scenes=[Scene("text", text="A", rgb=(1, 2, 3), wave_m=0.2, depth="tilt", depth_m=1.0),
                                             Scene("circle", facing_deg=90.0, hold_s=4)])
    d.save(tmp_path / "d.yaml")
    back = Design.load(tmp_path / "d.yaml")
    assert back == d and np.allclose(back.compile(site).P, d.compile(site).P)
    (tmp_path / "x.yaml").write_text("name: venue\n")
    with pytest.raises(DesignError, match="not a show design"):
        Design.load(tmp_path / "x.yaml")
