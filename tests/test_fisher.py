import numpy as np

from idps.core.camera import SENSORS, rotation_from_az_el
from idps.core.noise import NoiseBudget
from idps.coverage.fisher import VisibilityLimits, contributions, trace_inverse, unpack
from idps.coverage.volume import volume_grid


def test_trace_inverse_matches_numpy():
    rng = np.random.default_rng(1)
    A = rng.normal(size=(50, 3, 3))
    S = A @ np.transpose(A, (0, 2, 1)) + 0.1 * np.eye(3)
    packed = np.stack([S[:, 0, 0], S[:, 0, 1], S[:, 0, 2], S[:, 1, 1], S[:, 1, 2], S[:, 2, 2]], -1)
    assert np.allclose(trace_inverse(packed), np.trace(np.linalg.inv(S), axis1=1, axis2=2))
    assert np.allclose(unpack(packed), S)


def test_single_camera_is_singular():
    pts = np.array([[10.0, 0, 0]])
    vis, inf = contributions(np.zeros((1, 3)), rotation_from_az_el(0, 0)[None], 1000.0, pts,
                             SENSORS["OV9281"], NoiseBudget(), VisibilityLimits())
    assert vis[0, 0]
    assert np.isinf(trace_inverse(inf.sum(0)))[0]


def test_two_orthogonal_cameras_match_analytic_error():
    # Two cameras 10 m from the point along X and Y: each constrains the two
    # axes perpendicular to its ray with sigma = r * sigma_ang.
    noise = NoiseBudget(sigma_px=0.1, sigma_rot_rad=0.0, sigma_pos_m=0.0, sigma_t_s=0.0)
    f = 1000.0
    pts = np.array([[0.0, 0.0, 0.0]])
    pos = np.array([[-10.0, 0, 0], [0, -10.0, 0]])
    R = np.stack([rotation_from_az_el(0, 0), rotation_from_az_el(90, 0)])
    vis, inf = contributions(pos, R, f, pts, SENSORS["OV9281"], noise, VisibilityLimits())
    assert vis.all()
    s = 10.0 * 0.1 / f                       # lateral sigma per camera, m
    # x: seen only by camera 2; y: only by camera 1; z: by both
    expected = s ** 2 + s ** 2 + s ** 2 / 2
    assert np.isclose(trace_inverse(inf.sum(0))[0], expected, rtol=1e-9)


def test_visibility_limits():
    R = rotation_from_az_el(0, 0)[None]
    pts = np.array([[5.0, 0, 0], [-5.0, 0, 0], [200.0, 0, 0], [5.0, 50.0, 0]])
    vis, _ = contributions(np.zeros((1, 3)), R, 933.0, pts, SENSORS["OV9281"], NoiseBudget(),
                           VisibilityLimits(max_range_m=120))
    assert vis[0].tolist() == [True, False, False, False]


def test_volume_grid_includes_takeoff_layer_and_boundary():
    pts, (xs, ys, zs) = volume_grid([-5, -5, 0], [5, 5, 3], 2.0, inset=0.5)
    assert np.isclose(zs[0], 0.2) and np.isclose(zs[-1], 2.5)
    assert np.isclose(xs[0], -4.5) and np.isclose(xs[-1], 4.5)
    assert len(pts) == len(xs) * len(ys) * len(zs)
