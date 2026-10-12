"""Fly a show from the PC: one MAVLink link per drone, over Wi-Fi (decision D20).

The drones carry no companion computer: each flight controller has Wi-Fi itself (or a
few-gram Wi-Fi-to-serial MAVLink bridge), so the PC does everything a companion would:

* **Position in.** For every drone the engine tracks, the PC sends MAVLink ``ODOMETRY``:
  the IDPS position in the venue's NED frame, and the heading. Drones never yaw and always
  face North (D7), so the heading sent is the fixed yaw 0. PX4's EKF2 fuses both as external
  vision; that heading is also what replaces the magnetometer (off indoors).
* **Show out.** For every drone, ``SET_POSITION_TARGET_LOCAL_NED`` at 50 Hz from its
  file (position, velocity, acceleration, yaw 0), exactly as ``idps.show.runner`` does for
  one drone.

Everything runs on the PC's clock, so the drones need no clock sync of their own.

Drones are addressed by **drone ID** (D21): the same number is the show's "Drone N", the
drone's PX4 ``MAV_SYS_ID`` and its IR marker ID. ``preflight`` checks, from the marker IDs the
cameras see, that every drone stands on its own home spot before anything takes off.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from idps.show.runner import SETPOINT_HZ, TrajectoryPlayer

MAV_FRAME_LOCAL_NED = 1
MAV_FRAME_BODY_FRD = 12
MAV_ESTIMATOR_TYPE_MOCAP = 6
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
                estimator_type=MAV_ESTIMATOR_TYPE_MOCAP)


@dataclass
class Fix:
    """One IDPS position for a drone (from the engine)."""
    ned: np.ndarray
    vel: np.ndarray | None = None


class FleetPlayer:
    """Streams positions and setpoints to every drone of a show.

    ``links`` maps drone ID -> pymavlink connection. ``positions`` (in ``tick``/``run``)
    is a callable returning {drone ID: Fix} for the drones the engine currently sees; a drone
    missing from it gets no ODOMETRY that tick (PX4 then times out and lands if it lasts).
    """

    def __init__(self, manifest, links: dict, yaw_rad: float = 0.0):
        path = Path(manifest)
        man = json.loads(path.read_text())
        if man.get("format") != "idps-show/1":
            raise ValueError("not an idps-show/1 manifest")
        self.manifest = man
        self.players = {d["drone_id"]: TrajectoryPlayer(path.parent / d["file"]) for d in man["drones"]}
        self.homes = {d["drone_id"]: np.asarray(d["home_ned"], float) for d in man["drones"]}
        unknown = set(links) - set(self.players)
        if unknown:
            raise ValueError(f"drone ID(s) not in the show: {sorted(unknown)}")
        self.links = links
        self.yaw = yaw_rad
        self.t_end = max(p.t[-1] for p in self.players.values())

    def preflight(self, seen: dict, tol_m: float = 0.3) -> list[str]:
        """Problems that must block take-off, from what the cameras see on the ground.

        ``seen`` maps marker ID (= drone ID) -> NED position. Returns plain-language problems:
        a show drone not seen, a drone on another drone's spot or off its own, a marker that
        is not in the show, and a drone with no link. Empty list = clear to arm.
        """
        problems = []
        for did, home in sorted(self.homes.items()):
            if did not in seen:
                problems.append(f"Drone {did}: not seen (missing, marker off, or blocked)")
                continue
            off = float(np.linalg.norm(np.asarray(seen[did], float)[:2] - home[:2]))
            if off > tol_m:
                other = [o for o, h in self.homes.items()
                         if o != did and np.linalg.norm(np.asarray(seen[did], float)[:2] - h[:2]) <= tol_m]
                where = f"on Drone {other[0]}'s spot" if other else f"{off:.2f} m from its spot"
                problems.append(f"Drone {did}: {where} (home N {home[0]:.2f} E {home[1]:.2f})")
        for mid in sorted(set(seen) - set(self.homes)):
            problems.append(f"Marker {mid}: a drone that is not in this show is in the area")
        for did in sorted(set(self.homes) - set(self.links)):
            problems.append(f"Drone {did}: no link given")
        return problems

    def tick(self, show_t: float, positions: dict, now_s: float, boot_s: float) -> None:
        ms = int((now_s - boot_s) * 1000)
        for did, mav in self.links.items():
            fix = positions.get(did)
            if fix is not None:
                mav.mav.odometry_send(**odometry_fields(fix.ned, int(now_s * 1e6), self.yaw, fix.vel))
            sp = self.players[did].setpoint(show_t)
            mav.mav.set_position_target_local_ned_send(mav.target_system, mav.target_component,
                                                       **sp.mavlink_fields(ms))

    def run(self, start_epoch_s: float, positions=lambda: {}, clock=time.time, sleep=time.sleep,
            abort=lambda: False) -> None:
        """From now (home setpoints, so PX4 accepts OFFBOARD) to 2 s after the last drone ends."""
        period = 1.0 / SETPOINT_HZ
        boot = clock()
        while not abort():
            now = clock()
            show_t = now - start_epoch_s
            if show_t > self.t_end + 2.0:
                break
            self.tick(show_t, positions(), now, boot)
            sleep(max(0.0, period - (clock() - now)))


def parse_links(specs) -> dict:
    """['7=udpout:192.168.50.107:14550', ...] -> {7: 'udpout:...'} (drone ID = DEVICE)"""
    out = {}
    for s in specs or []:
        did, _, dev = s.partition("=")
        if not dev:
            raise ValueError(f"--link needs DRONE_ID=DEVICE, got {s!r}")
        out[int(did)] = dev
    return out
