"""Stand-in for the IDPS engine with PX4 SITL drones (run_sitl.sh).

Reads each simulated drone's true position (HIL_STATE_QUATERNION on udp 14800+i), places it
at that drone's home spot from the show manifest (every SITL drone starts at its own 0,0),
adds 2 mm noise, and sends it back as the IDPS position: MAVLink ODOMETRY in NED with
heading 0, exactly as idps.show.fleet does.

    python px4/sitl/idps_stand_in.py out/show/manifest.json 3
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

R_EARTH = 6378137.0


def stand_in(instance: int, home, noise_m: float = 0.002):
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
                        -(alt - origin[2])]) + home + rng.normal(0, noise_m, 3)
        m.mav.odometry_send(**odometry_fields(ned, int(time.monotonic() * 1e6)))


def main():
    man = json.loads(Path(sys.argv[1]).read_text())
    homes = {d["drone_id"]: np.array(d["home_ned"], float) for d in man["drones"]}
    n = int(sys.argv[2]) if len(sys.argv) > 2 else len(homes)
    for i in range(n):
        threading.Thread(target=stand_in, args=(i, homes[i + 1]), daemon=True).start()
    print(f"IDPS stand-in: positions for drones 1..{n} (Ctrl+C to stop)")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
