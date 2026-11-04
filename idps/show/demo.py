"""A small demo show for testing and rehearsal: drones take off from a grid on the floor,
rise into a ring, turn it once, and land back on their spots. Written as a Skybrush-style
.skyc (show.json with the trajectories inside), in the default ENU show axes."""
from __future__ import annotations

import json
import math
import zipfile
from pathlib import Path

import numpy as np


def demo_points(n=8, radius=2.4, height=1.8, centre=(0.0, 0.0), home_radius=1.5, turn_s=12.0, climb_s=4.0):
    """Skybrush keypoints [[t, [x, y, z], [cps]]] per drone (x East, y North, z up).
    Homes sit on a small circle; each drone climbs straight up, moves straight out to its
    slot on the ring (so paths never cross), the ring turns once, and they come back."""
    out = []
    for k in range(n):
        a0 = 2 * math.pi * k / n
        ring = lambda a, r=radius, z=height: np.array([centre[0] + r * math.cos(a), centre[1] + r * math.sin(a), z])  # noqa: E731
        home = ring(a0, home_radius, 0.0)
        up = home + [0, 0, height]
        p0 = ring(a0)
        line = lambda a, b: [(a + (b - a) / 3).tolist(), (a + 2 * (b - a) / 3).tolist()]   # noqa: E731
        pts = [[0.0, home.tolist(), []],
               [climb_s, up.tolist(), line(home, up)],
               [climb_s + 3.0, p0.tolist(), line(up, p0)]]
        t = climb_s + 3.0
        steps = 8                                              # one turn as cubic arcs (8 x 45 degrees)
        kappa = 4 / 3 * math.tan(math.pi / (2 * steps)) * radius
        for s in range(1, steps + 1):
            a_prev, a1 = a0 + 2 * math.pi * (s - 1) / steps, a0 + 2 * math.pi * s / steps
            p_prev, p1 = ring(a_prev), ring(a1)
            c1 = p_prev + kappa * np.array([-math.sin(a_prev), math.cos(a_prev), 0])
            c2 = p1 - kappa * np.array([-math.sin(a1), math.cos(a1), 0])
            t += turn_s / steps
            pts.append([round(t, 3), p1.tolist(), [c1.tolist(), c2.tolist()]])
        pts.append([t + 3.0, up.tolist(), line(p0, up)])
        pts.append([t + 3.0 + climb_s, home.tolist(), line(up, home)])
        out.append({"name": f"Drone {k + 1}", "home": home.tolist(), "points": pts})
    return out


def write_demo_skyc(path, layout="inline", **kw) -> Path:
    """layout 'inline': trajectories in show.json; 'folders': drones/<name>/trajectory.json."""
    path = Path(path)
    drones = demo_points(**kw)
    entries = []
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for d in drones:
            traj = {"version": 1, "points": d["points"], "takeoffTime": 0.0, "landingTime": d["points"][-1][0]}
            if layout == "inline":
                entries.append({"type": "generic", "settings": {"name": d["name"], "home": d["home"],
                                                                "trajectory": traj}})
            else:
                entries.append({"type": "generic", "settings": {"name": d["name"], "home": d["home"]}})
                z.writestr(f"drones/{d['name']}/trajectory.json", json.dumps(traj))
        z.writestr("show.json", json.dumps({"version": 1, "environment": {"type": "indoor"},
                                            "meta": {"title": "IDPS demo ring"}, "swarm": {"drones": entries}}))
    return path
