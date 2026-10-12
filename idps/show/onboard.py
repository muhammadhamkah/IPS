"""PC side of the onboard show player (decision D22): upload, start, abort, status.

Each drone runs the PX4 module ``idps_show`` (``px4/src/modules/idps_show``) and flies its
own file from its SD card. The PC only:

1. **uploads** each drone's ``drone_NNN.trj`` over MAVLink FTP and checks the drone's CRC of
   the file against its own;
2. keeps every drone on the **PC's clock**: ``ClockSync`` sends each drone the PC's time
   5 times a second for as long as the drones are up. Each drone keeps the fastest-arriving
   message per second and fits offset and drift, so all drones share one clock to about a
   millisecond and stay on it through the show, whatever their crystals do;
3. sends **start**: one show start time in the PC's clock, the same for every drone;
4. can **abort** (every drone lands where it is) and read each drone's **status**.

Commands are MAVLink COMMAND_INT, command 0 (PX4 VEHICLE_CMD_CUSTOM_0), to the drone's
MAV_SYS_ID (= drone ID, D21); the protocol is documented in the module's IdpsShow.hpp.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

from idps.show.trj import crc32

CMD_IDPS_SHOW = 0
LOAD, START, ABORT, STATUS, SYNC = 1, 2, 3, 4, 5
STATES = {0: "no show file", 1: "ready", 2: "waiting for start", 3: "flying the show", 4: "show finished",
          5: "aborted"}
MAV_RESULT = {0: "accepted", 1: "temporarily rejected", 2: "denied", 3: "unsupported", 4: "failed",
              5: "in progress", 6: "cancelled"}
SD_PATH = "/fs/microsd/idps_show.trj"      # flight controller; PX4 SITL uses "idps_show.trj"


@dataclass
class Reply:
    ok: bool
    result: str
    state: str
    crc: int
    text: str = ""


def crc_halves(crc: int) -> tuple[float, float]:
    return float((crc >> 16) & 0xFFFF), float(crc & 0xFFFF)


def start_time_fields(t_us: int) -> tuple[int, int]:
    """Drone clock time (µs) -> COMMAND_INT x, y (whole seconds, microseconds)."""
    return int(t_us // 1_000_000), int(t_us % 1_000_000)


def clock_offset_us(mav, samples: int = 15, timeout_s: float = 0.5, clock=time.monotonic_ns) -> tuple[int, int]:
    """Diagnostics: offset (drone clock − PC clock, µs) and round-trip time of the best sample.

    PX4 answers TIMESYNC(tc1=0, ts1=t) with tc1 = its own clock in ns and ts1 = t."""
    best = None
    for _ in range(samples):
        t0 = clock()
        mav.mav.timesync_send(0, t0)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            m = mav.recv_match(type="TIMESYNC", blocking=True, timeout=timeout_s)
            if m is None:
                break
            if m.ts1 == t0 and m.tc1 > 0 and m.get_srcSystem() == mav.target_system:
                t1 = clock()
                rtt = t1 - t0
                off = m.tc1 - (t0 + t1) // 2
                if best is None or rtt < best[1]:
                    best = (off, rtt)
                break
        time.sleep(0.02)
    if best is None:
        raise TimeoutError("no TIMESYNC reply from the drone")
    return best[0] // 1000, best[1] // 1000


def send_heartbeat(mav) -> None:
    """Ground-station heartbeat: PX4 only sends its text messages (the reason a drone refuses)
    to links it has heard a GCS heartbeat on."""
    mav.mav.heartbeat_send(6, 8, 0, 0, 0)            # MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID


def command(mav, drone_id: int, action: int, crc: int = 0, start_us: int | None = None,
            timeout_s: float = 3.0, retries: int = 3) -> Reply:
    """Send an IDPS show command and wait for the drone's COMMAND_ACK."""
    send_heartbeat(mav)
    hi, lo = crc_halves(crc)
    x, y = start_time_fields(start_us) if start_us is not None else (0x7FFFFFFF, 0x7FFFFFFF)
    for _ in range(retries):
        mav.mav.command_int_send(drone_id, 1, 0, CMD_IDPS_SHOW, 0, 0, float(action), float(drone_id), hi, lo,
                                 x, y, 0.0)
        deadline = time.monotonic() + timeout_s
        texts = []
        while time.monotonic() < deadline:
            m = mav.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True,
                               timeout=max(0.01, deadline - time.monotonic()))
            if m is None:
                break
            if m.get_srcSystem() != drone_id:
                continue
            if m.get_type() == "STATUSTEXT":
                texts.append(m.text)
                continue
            if m.command == CMD_IDPS_SHOW:
                if m.result != 0:                       # the drone says why in a STATUSTEXT
                    until = time.monotonic() + 0.5
                    while time.monotonic() < until:
                        t = mav.recv_match(type="STATUSTEXT", blocking=True, timeout=0.1)
                        if t is not None and t.get_srcSystem() == drone_id:
                            texts.append(t.text)
                return Reply(m.result == 0, MAV_RESULT.get(m.result, str(m.result)),
                             STATES.get(m.progress, str(m.progress)), m.result_param2 & 0xFFFFFFFF,
                             "; ".join(t.replace("IDPS show: ", "") for t in texts if "IDPS" in t))
    return Reply(False, "no answer", "?", 0)


def send_sync(mav, drone_id: int, clock=time.monotonic_ns) -> None:
    """One clock-sync message: the PC's time now (not acknowledged by the drone)."""
    x, y = start_time_fields(clock() // 1000)
    mav.mav.command_int_send(drone_id, 1, 0, CMD_IDPS_SHOW, 0, 0, float(SYNC), float(drone_id), 0.0, 0.0, x, y, 0.0)


class ClockSync:
    """Sends the PC's time to every drone at ``hz`` from a background thread. Start it at
    least ``WARMUP_S`` before starting a show and keep it running until the drones land."""

    WARMUP_S = 5.0

    def __init__(self, links: dict, hz: float = 5.0, clock=time.monotonic_ns):
        import threading
        self.links, self.period, self.clock = links, 1.0 / hz, clock
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.started = None

    def _run(self):
        n = 0
        while not self._stop.is_set():
            for did, mav in self.links.items():
                send_sync(mav, did, self.clock)
                if n % 5 == 0:
                    send_heartbeat(mav)
            n += 1
            self._stop.wait(self.period)

    def start(self):
        self.started = time.monotonic()
        self._thread.start()
        return self

    def warm(self) -> bool:
        return self.started is not None and time.monotonic() - self.started >= self.WARMUP_S

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)


def upload(mav, data: bytes, remote_path: str = SD_PATH) -> int:
    """Upload over MAVLink FTP; returns the drone's CRC of the written file (PX4 crc32)."""
    import tempfile
    from pymavlink.mavftp import MAVFTP
    ftp = MAVFTP(mav, mav.target_system, mav.target_component)
    with tempfile.NamedTemporaryFile(suffix=".trj", delete=False) as f:
        f.write(data)
        local = f.name
    try:
        ret = ftp.cmd_put([local, remote_path])
        if ret.return_code != 0:
            raise IOError(f"upload failed: {ret}")
        ret = ftp.process_ftp_reply("CreateFile", timeout=60)
        if ret.return_code != 0:
            raise IOError(f"upload failed: {ret}")
    finally:
        Path(local).unlink(missing_ok=True)
    return crc32(data)


@dataclass
class Drone:
    drone_id: int
    mav: object
    trj: bytes
    crc: int
    offset_us: int | None = None
    rtt_us: int | None = None


def drones_from_manifest(manifest, links: dict) -> dict:
    """{drone ID: Drone} for the drones that have a link."""
    import json
    path = Path(manifest)
    man = json.loads(path.read_text())
    out = {}
    for d in man["drones"]:
        did = d["drone_id"]
        if did in links:
            b = (path.parent / d["trj"]).read_bytes()
            out[did] = Drone(did, links[did], b, crc32(b))
    missing = sorted(set(links) - {d["drone_id"] for d in man["drones"]})
    if missing:
        raise ValueError(f"drone ID(s) not in the show: {missing}")
    return out


def start_all(drones: dict, lead_s: float = 10.0, clock=time.monotonic_ns) -> tuple[int, dict]:
    """Start every drone at one instant ``lead_s`` from now, in the PC's clock (``ClockSync``
    must have been running for a few seconds). Returns (show start in the PC clock, µs;
    {drone ID: Reply}). If any drone refuses, all are aborted, so nobody flies a partial show."""
    t_pc_us = clock() // 1000 + int(lead_s * 1e6)
    replies = {did: command(d.mav, did, START, d.crc, t_pc_us) for did, d in drones.items()}
    if not all(r.ok for r in replies.values()):
        for did, d in drones.items():
            command(d.mav, did, ABORT)
    return t_pc_us, replies
