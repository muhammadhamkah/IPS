"""Testing tool: play one drone's file from the PC through PX4 offboard, or print it.

Shows are flown by the drones themselves (the ``idps_show`` PX4 module, D22); this streams
setpoints from the PC instead, to try a single path against one drone or PX4 SITL.

It loads a ``drone_NNN.json``, waits for the show start time and streams PX4 **offboard**
setpoints at 50 Hz over any MAVLink link (serial, or UDP over Wi-Fi):

* ``SET_POSITION_TARGET_LOCAL_NED`` with position, velocity and acceleration feed-forward
  and yaw 0 (North), in ``MAV_FRAME_LOCAL_NED``;
* PX4 needs the stream running before it accepts OFFBOARD mode, so the player starts
  streaming the home position before take-off.

Positions are in the venue's NED frame. They match PX4's local frame when PX4's EKF2 fuses
IDPS as external vision with vision as the height and position reference (docs/SHOW.md).

The MAVLink side needs ``pymavlink`` (``pip install pymavlink``); everything else is
plain Python so the timing logic can be tested without a drone.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

MAV_FRAME_LOCAL_NED = 1
TYPE_MASK_POS_VEL_ACC_YAW = 0b0000_1000_0000_0000      # use x,y,z, vx,vy,vz, afx,afy,afz, yaw; ignore yaw rate
SETPOINT_HZ = 50.0


@dataclass
class Setpoint:
    t: float
    pos: np.ndarray
    vel: np.ndarray
    acc: np.ndarray
    yaw_rad: float
    rgb: np.ndarray

    def mavlink_fields(self, time_boot_ms: int = 0) -> dict:
        """Arguments for pymavlink's set_position_target_local_ned_send (after target ids)."""
        return dict(time_boot_ms=time_boot_ms, coordinate_frame=MAV_FRAME_LOCAL_NED,
                    type_mask=TYPE_MASK_POS_VEL_ACC_YAW,
                    x=float(self.pos[0]), y=float(self.pos[1]), z=float(self.pos[2]),
                    vx=float(self.vel[0]), vy=float(self.vel[1]), vz=float(self.vel[2]),
                    afx=float(self.acc[0]), afy=float(self.acc[1]), afz=float(self.acc[2]),
                    yaw=float(self.yaw_rad), yaw_rate=0.0)


class TrajectoryPlayer:
    def __init__(self, path_or_doc):
        doc = path_or_doc if isinstance(path_or_doc, dict) else json.loads(Path(path_or_doc).read_text())
        if doc.get("format") != "idps-traj/1":
            raise ValueError("not an idps-traj/1 file")
        self.doc = doc
        cols = doc["columns"]
        S = np.asarray(doc["samples"], float)
        self.t = S[:, cols.index("t")]
        self.P = S[:, [cols.index(c) for c in ("n", "e", "d")]]
        self.V = S[:, [cols.index(c) for c in ("vn", "ve", "vd")]]
        self.A = S[:, [cols.index(c) for c in ("an", "ae", "ad")]]
        self.C = S[:, [cols.index(c) for c in ("r", "g", "b")]]
        self.yaw = np.radians(float(doc.get("yaw_deg", 0.0)))

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0])

    def setpoint(self, show_t: float) -> Setpoint:
        """Setpoint at show time (s); before the start it holds home, after the end the last point."""
        t = float(np.clip(show_t, self.t[0], self.t[-1]))
        k = int(np.clip(np.searchsorted(self.t, t, side="right") - 1, 0, len(self.t) - 2))
        dt = self.t[k + 1] - self.t[k]
        f = 0.0 if dt <= 0 else (t - self.t[k]) / dt
        lerp = lambda X: X[k] * (1 - f) + X[k + 1] * f        # noqa: E731
        moving = self.t[0] <= show_t <= self.t[-1]
        return Setpoint(t, lerp(self.P), lerp(self.V) if moving else np.zeros(3),
                        lerp(self.A) if moving else np.zeros(3), self.yaw, lerp(self.C))

    def run(self, mav, start_epoch_s: float, clock=time.time, sleep=time.sleep, abort=lambda: False):
        """Stream setpoints over a pymavlink connection from start_epoch_s (wall clock) to the end.
        Starts streaming the home setpoint 3 s early so PX4 accepts OFFBOARD."""
        period = 1.0 / SETPOINT_HZ
        boot = clock()
        while not abort():
            now = clock()
            show_t = now - start_epoch_s + self.t[0]
            if show_t > self.t[-1] + 2.0:
                break
            sp = self.setpoint(show_t)
            mav.mav.set_position_target_local_ned_send(
                mav.target_system, mav.target_component, **sp.mavlink_fields(int((now - boot) * 1000)))
            sleep(max(0.0, period - (clock() - now)))


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(prog="idps-show-play", description="Play a drone file on a PX4 drone")
    ap.add_argument("drone_file")
    ap.add_argument("--device", default="/dev/ttyAMA0",
                    help="flight-controller link: serial device, or e.g. udpin:0.0.0.0:14540 for PX4 SITL")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--start", type=float, help="show start, Unix time (s); default: now + 5 s")
    ap.add_argument("--dry-run", action="store_true", help="print setpoints at 2 Hz instead of sending")
    a = ap.parse_args(argv)
    pl = TrajectoryPlayer(a.drone_file)
    if a.dry_run:
        for t in np.arange(pl.t[0], pl.t[-1] + 1e-9, 0.5):
            sp = pl.setpoint(t)
            print(f"t={t:6.2f}  N {sp.pos[0]:7.3f}  E {sp.pos[1]:7.3f}  D {sp.pos[2]:7.3f}  "
                  f"v {np.linalg.norm(sp.vel):5.2f} m/s  rgb {sp.rgb.astype(int).tolist()}")
        return 0
    from pymavlink import mavutil                     # optional dependency
    mav = mavutil.mavlink_connection(a.device, baud=a.baud)
    mav.wait_heartbeat()
    pl.run(mav, a.start if a.start else time.time() + 5.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
