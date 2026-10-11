import numpy as np
import pytest

from idps.core.camera import SENSORS, az_el_from_direction, direction_from_az_el, project, rotation_from_az_el


@pytest.mark.parametrize("az,el", [(0, 0), (90, 0), (37, -20), (200, 45), (0, -90), (123, 90)])
def test_rotation_is_proper_and_looks_along_aim(az, el):
    R = rotation_from_az_el(az, el)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(R), 1.0)
    assert np.allclose(R[2], direction_from_az_el(az, el))
    assert abs(R[0, 2]) < 1e-12          # image x axis stays horizontal (no roll)


def test_down_axis_points_down_for_level_camera():
    R = rotation_from_az_el(0, 0)
    assert np.allclose(R[1], [0, 0, -1])  # OpenCV +Y down
    assert np.allclose(R[0], [0, -1, 0])  # +X right when looking along world +X


def test_project_centre_and_offset():
    R = rotation_from_az_el(0, 0)
    pos = np.zeros(3)
    u, v, z = project(R, pos, 1000.0, np.array([[10.0, 0, 0], [10.0, -1.0, 0.5]]))
    assert np.allclose([u[0], v[0], z[0]], [0, 0, 10])
    assert np.isclose(u[1], 100.0) and np.isclose(v[1], -50.0)


def test_az_el_roundtrip():
    for az, el in [(10, 5), (-120, -30), (179, 60)]:
        a, e = az_el_from_direction(direction_from_az_el(az, el))
        assert np.isclose(a, az) and np.isclose(e, el)


def test_sensor_fov():
    s = SENSORS["OV9281"]
    assert np.isclose(s.f_px(2.8), 933.33, atol=0.01)
    assert 68 < s.hfov_deg(2.8) < 70
