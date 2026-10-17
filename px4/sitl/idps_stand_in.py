"""Stand-in for the IDPS engine with PX4 SITL drones (run_sitl.sh).

Reads each simulated drone's true position (HIL_STATE_QUATERNION on udp 14800+i), places it
at that drone's home spot from the show manifest (every SITL drone starts at its own 0,0),
moves it to the marker (8 cm above the centre, EKF2_EV_POS_Z in run_sitl.sh), adds 0.5 mm noise,
and sends it back as the IDPS position: MAVLink ODOMETRY in NED with heading 0, exactly as
idps.show.fleet does. For the real thing, cameras and engine, see idps-sim px4 (px4/README.md).

    python px4/sitl/idps_stand_in.py out/show/manifest.json 3 [truth.csv [log-only]]

truth.csv logs each drone's true centre; truth.track.csv logs how far each drone is from its
file at its own show time (idps_show computes it), which a slow or stalled simulator host does
not distort.
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
from idps.sim.px4run import TRACK_HEADER, track_line  # noqa: E402

R_EARTH = 6378137.0
MARKER_FRD = np.array([0.0, 0.0, -0.08])


def stand_in(instance: int, home, noise_m: float = 0.0005, log=None, send=True, track=None):
    """Sends drone instance+1 its IDPS position. The exact simulator position comes as DEBUG_VECT
    "truth" (from the idps_show module, run_sitl.sh streams it); without it, latitude/longitude
    from HIL_STATE_QUATERNION (1.1 cm steps). The attitude (for the marker offset) comes from
    HIL_STATE_QUATERNION."""
    m = mavutil.mavlink_connection(f"udpin:127.0.0.1:{14800 + instance}", source_system=254, source_component=197)
    rng = np.random.default_rng(instance)
    origin, R, exact, px4_us = None, np.eye(3), 0.0, 0
    last, warned = time.monotonic(), False
    while True:
        msg = m.recv_match(type=["HIL_STATE_QUATERNION", "DEBUG_VECT", "DEBUG_FLOAT_ARRAY"], blocking=True, timeout=1.0)
        if msg is None:
            if not warned and time.monotonic() - last > 15:
                print(f"drone {instance + 1}: nothing on udp {14800 + instance} for 15 s. Is another program "
                      "using that port (a leftover idps-sim px4 or stand-in)?", flush=True)
                warned = True
            continue
        last = time.monotonic()
        now = time.monotonic()
        if msg.get_type() == "DEBUG_FLOAT_ARRAY":            # idps_show: drone minus file, at its show time
            if track is not None and msg.name.startswith("track"):
                track.write(track_line(instance + 1, msg.data))
            continue
        if msg.get_type() == "DEBUG_VECT":
            if not msg.name.startswith("truth"):
                continue
            exact = now
            px4_us = msg.time_usec
            centre = np.array([msg.x, msg.y, msg.z]) + home
        else:
            R = quat_to_R(msg.attitude_quaternion)
            if now - exact < 1.0:                            # the exact position is arriving: use it
                continue
            lat, lon, alt = msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3
            origin = origin or (lat, lon, alt)
            centre = np.array([math.radians(lat - origin[0]) * R_EARTH,
                               math.radians(lon - origin[1]) * R_EARTH * math.cos(math.radians(origin[0])),
                               -(alt - origin[2])]) + home
        if log is not None:                                  # the true centre
            log.write(f"{now:.4f},{instance + 1},{centre[0]:.5f},{centre[1]:.5f},{centre[2]:.5f},{px4_us}\n")
        if not send:
            continue
        ned = centre + R @ MARKER_FRD + rng.normal(0, noise_m, 3)
        m.mav.odometry_send(**odometry_fields(ned, int(now * 1e6), pos_sigma_m=max(noise_m, 0.001)))


def main():
    man = json.loads(Path(sys.argv[1]).read_text())
    homes = {d["drone_id"]: np.array(d["home_ned"], float) for d in man["drones"]}
    n = int(sys.argv[2]) if len(sys.argv) > 2 else len(homes)
    log = open(sys.argv[3], "w", buffering=1) if len(sys.argv) > 3 else None   # optional truth log (CSV)
    if log:
        log.write("t_monotonic,drone_id,n,e,d,px4_time_us\n")
    track = open(Path(sys.argv[3]).with_suffix(".track.csv"), "w", buffering=1) if log else None
    if track:                                      # drone minus file (m, NED) at the drone's own show time
        track.write(TRACK_HEADER)
    send = not (len(sys.argv) > 4 and sys.argv[4] == "log-only")   # log-only: SITL_VISION feeds EKF2
    for i in range(n):
        threading.Thread(target=stand_in, args=(i, homes[i + 1], 0.0005, log, send, track), daemon=True).start()
    print(f"IDPS stand-in: positions for drones 1..{n} (Ctrl+C to stop)")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
