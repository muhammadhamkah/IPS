"""Stand-in for the IDPS engine with PX4 SITL drones (run_sitl.sh).

Reads each simulated drone's true position (HIL_STATE_QUATERNION on udp 14800+i), places it
at that drone's home spot from the show manifest (every SITL drone starts at its own 0,0),
moves it to the marker (8 cm above the centre, EKF2_EV_POS_Z in run_sitl.sh), adds 2 mm noise,
and sends it back as the IDPS position: MAVLink ODOMETRY in NED with heading 0, exactly as
idps.show.fleet does. For the real thing, cameras and engine, see idps-sim px4 (px4/README.md).

    python px4/sitl/idps_stand_in.py out/show/manifest.json 3 [truth.csv]
"""
import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from idps.show.fleet import odometry_fields  # noqa: E402
from idps.sim.live import quat_to_R  # noqa: E402

R_EARTH = 6378137.0
MARKER_FRD = np.array([0.0, 0.0, -0.08])


def stand_in(instance: int, home, noise_m: float = 0.002, log=None):
    m = mavutil.mavlink_connection(f"udpin:127.0.0.1:{14800 + instance}", source_system=254, source_component=197)
    rng = np.random.default_rng(instance)
    origin = None
    while True:
        h = m.recv_match(type="HIL_STATE_QUATERNION", blocking=True, timeout=1.0)
        if h is None:
            continue
        lat, lon, alt = h.lat * 1e-7, h.lon * 1e-7, h.alt * 1e-3
        origin = origin or (lat, lon, alt)
        ned = np.array([math.radians(lat - origin[0]) * R_EARTH,
                        math.radians(lon - origin[1]) * R_EARTH * math.cos(math.radians(origin[0])),
                        -(alt - origin[2])]) + home + (noise := rng.normal(0, noise_m, 3))
        if log is not None:                                  # true centre, before noise and the marker offset
            log.write(f"{time.monotonic():.4f},{instance + 1},{ned[0] - noise[0]:.4f},{ned[1] - noise[1]:.4f},"
                      f"{ned[2] - noise[2]:.4f}\n")
        ned += quat_to_R(h.attitude_quaternion) @ MARKER_FRD
        m.mav.odometry_send(**odometry_fields(ned, int(time.monotonic() * 1e6)))


def main():
    man = json.loads(Path(sys.argv[1]).read_text())
    homes = {d["drone_id"]: np.array(d["home_ned"], float) for d in man["drones"]}
    n = int(sys.argv[2]) if len(sys.argv) > 2 else len(homes)
    log = open(sys.argv[3], "w", buffering=1) if len(sys.argv) > 3 else None   # optional truth log (CSV)
    if log:
        log.write("t_monotonic,drone_id,n,e,d\n")
    for i in range(n):
        threading.Thread(target=stand_in, args=(i, homes[i + 1], 0.002, log), daemon=True).start()
    print(f"IDPS stand-in: positions for drones 1..{n} (Ctrl+C to stop)")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
