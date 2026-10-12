"""The PC's live link to the drones during a show (decisions D20-D22).

The drones carry no companion computer and fly their own show files (``idps_show`` PX4
module, ``idps.show.onboard``). What still flows over Wi-Fi while they fly:

* **Positions.** For every drone the engine tracks, MAVLink ``ODOMETRY``: the IDPS position
  in the venue's NED frame and the heading. Drones never yaw and face North (D7), so the
  heading is the fixed yaw 0; PX4's EKF2 fuses both as external vision, and that heading
  replaces the magnetometer (off indoors).
* **Clock sync** (``onboard.ClockSync``), 5 messages a second per drone.

Drones are addressed by **drone ID** (D21): the show's "Drone N", the drone's PX4
``MAV_SYS_ID`` and its IR marker ID are the same number. ``Fleet.preflight`` checks, from the
marker IDs the cameras see, that every drone stands on its own home spot before the start.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


MAV_FRAME_LOCAL_NED = 1
MAV_FRAME_BODY_FRD = 12
# PX4 fuses ODOMETRY in EKF2 only when it is marked VISION or VIO (it publishes those as
# vehicle_visual_odometry); MOCAP goes to a topic EKF2 does not read.
MAV_ESTIMATOR_TYPE_VISION = 3
NAN = float("nan")


def odometry_fields(ned, time_usec: int, yaw_rad: float = 0.0, vel_ned=None,
                    pos_sigma_m: float = 0.005, yaw_sigma_rad: float = math.radians(3.0),
                    reset_counter: int = 0) -> dict:
    """Arguments for pymavlink's odometry_send: IDPS position (NED) and the fixed heading.

    Roll and pitch are the drone's own (PX4 only takes yaw from the quaternion when
    EKF2_EV_CTRL has the yaw bit). The heading uncertainty covers how squarely the drone was
    put down facing North on its home pad.
    """
    cov = [NAN] * 21                                   # upper triangle of the 6x6 pose covariance
    cov[0] = cov[6] = cov[11] = pos_sigma_m ** 2       # x, y, z; roll and pitch stay NaN
    cov[20] = yaw_sigma_rad ** 2                       # yaw
    vcov = [NAN] * 21
    if vel_ned is None:
        v = (NAN, NAN, NAN)
    else:
        v = tuple(float(x) for x in vel_ned)
        vcov[0] = vcov[6] = vcov[11] = 0.05 ** 2
    return dict(time_usec=int(time_usec), frame_id=MAV_FRAME_LOCAL_NED, child_frame_id=MAV_FRAME_BODY_FRD,
                x=float(ned[0]), y=float(ned[1]), z=float(ned[2]),
                q=[math.cos(yaw_rad / 2), 0.0, 0.0, math.sin(yaw_rad / 2)],
                vx=v[0], vy=v[1], vz=v[2], rollspeed=NAN, pitchspeed=NAN, yawspeed=NAN,
                pose_covariance=cov, velocity_covariance=vcov, reset_counter=reset_counter,
                estimator_type=MAV_ESTIMATOR_TYPE_VISION)


@dataclass
class Fix:
    """One IDPS position for a drone (from the engine)."""
    ned: np.ndarray
    vel: np.ndarray | None = None


class Fleet:
    """The show's drones: their home spots (from the manifest) and their links.

    ``links`` maps drone ID -> pymavlink connection (or None in tests that only check).
    """

    def __init__(self, manifest, links: dict):
        path = Path(manifest)
        man = json.loads(path.read_text())
        if man.get("format") != "idps-show/1":
            raise ValueError("not an idps-show/1 manifest")
        self.manifest = man
        self.homes = {d["drone_id"]: np.asarray(d["home_ned"], float) for d in man["drones"]}
        unknown = set(links) - set(self.homes)
        if unknown:
            raise ValueError(f"drone ID(s) not in the show: {sorted(unknown)}")
        self.links = links

    def preflight(self, seen: dict, tol_m: float = 0.3) -> list[str]:
        """Problems that must block the start, from what the cameras see on the ground.

        ``seen`` maps marker ID (= drone ID) -> NED position. Returns plain-language problems:
        a show drone not seen, a drone on another drone's spot or off its own, a marker that
        is not in the show, and a drone with no link. Empty list = clear to start.
        (Each drone also refuses to start if it is more than 0.5 m from its spot.)
        """
        problems = []
        for did, home in sorted(self.homes.items()):
            if did not in seen:
                problems.append(f"Drone {did}: not seen (missing, marker off, or blocked)")
                continue
            p = np.asarray(seen[did], float)[:2]
            off = float(np.linalg.norm(p - home[:2]))
            if off > tol_m:
                other = [o for o, h in self.homes.items() if o != did and np.linalg.norm(p - h[:2]) <= tol_m]
                where = f"on Drone {other[0]}'s spot" if other else f"{off:.2f} m from its spot"
                problems.append(f"Drone {did}: {where} (home N {home[0]:.2f} E {home[1]:.2f})")
        for mid in sorted(set(seen) - set(self.homes)):
            problems.append(f"Marker {mid}: a drone that is not in this show is in the area")
        for did in sorted(set(self.homes) - set(self.links)):
            problems.append(f"Drone {did}: no link given")
        return problems

    def send_positions(self, positions: dict, now_s: float | None = None) -> int:
        """Send ODOMETRY to every linked drone the engine sees ({drone ID: Fix}). A drone
        missing from ``positions`` gets nothing; if that lasts, PX4 lands it."""
        t_us = int((time.monotonic() if now_s is None else now_s) * 1e6)
        n = 0
        for did, mav in self.links.items():
            fix = positions.get(did)
            if fix is not None and mav is not None:
                mav.mav.odometry_send(**odometry_fields(fix.ned, t_us, 0.0, fix.vel))
                n += 1
        return n


def parse_links(specs) -> dict:
    """['7=udpout:192.168.50.107:14550', ...] -> {7: 'udpout:...'} (drone ID = DEVICE)"""
    out = {}
    for s in specs or []:
        did, _, dev = s.partition("=")
        if not dev:
            raise ValueError(f"--link needs DRONE_ID=DEVICE, got {s!r}")
        out[int(did)] = dev
    return out
