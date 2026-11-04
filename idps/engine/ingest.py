"""Ingest: camera-station packets -> whole camera frames.

Validates each UDP packet, puts frames that were split over several packets back together,
drops duplicates, counts lost packets from the sequence numbers, and keeps each station's
HEALTH report.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from idps.protocol import packets as P

# one marker record (protocol MARKER struct "<ffIHBB"), read straight into arrays
MARKER_DT = np.dtype([("x", "<f4"), ("y", "<f4"), ("sum", "<u4"), ("peak", "<u2"), ("area", "u1"), ("mflags", "u1")])
assert MARKER_DT.itemsize == P.MARKER.size


@dataclass
class CameraFrame:
    station_id: int
    frame_id: int
    t_ns: int                    # mid-exposure, system time (station clocks are PTP-synced)
    xy: np.ndarray               # (N, 2) distorted pixel centroids
    sum: np.ndarray              # (N,) integrated brightness
    peak: np.ndarray             # (N,)
    mflags: np.ndarray           # (N,)
    sync_state: int = 0
    flags: int = 0
    rays: np.ndarray | None = None      # (N, 3) world lines of sight, filled in by the engine
    static: np.ndarray | None = None    # (N,) bool, dot at a known fixed reflection

    @property
    def n(self) -> int:
        return len(self.xy)


@dataclass
class StationStats:
    frames: int = 0
    packets: int = 0
    bad: int = 0
    first_seq: int | None = None
    last_seq: int | None = None
    health: dict = field(default_factory=dict)

    @property
    def lost(self) -> int:
        """Packets never received (sequence numbers are per station, so reordering is fine)."""
        if self.first_seq is None:
            return 0
        return max(0, ((self.last_seq - self.first_seq) & 0xFFFFFFFF) + 1 - self.packets)


class Ingest:
    def __init__(self):
        self.stations: dict[int, StationStats] = {}
        self._parts: dict[tuple[int, int], list] = {}       # (station, frame_id) -> FrameObs parts
        self._seen: dict[int, set] = {}                      # recent frame ids per station (dedupe)

    def feed(self, data: bytes) -> list[CameraFrame]:
        """One packet in; zero or one complete frame out (as a list)."""
        try:
            h = P.decode_header(data)
            if h.msg_type == P.MSG_FRAME_OBS:
                body = self._frame_obs(h, data)
            else:
                _, body = P.decode(data)
        except P.ProtocolError:
            self.stations.setdefault(-1, StationStats()).bad += 1
            return []
        st = self.stations.setdefault(h.station_id, StationStats())
        st.packets += 1
        if st.first_seq is None:
            st.first_seq = st.last_seq = h.seq
        elif ((h.seq - st.last_seq) & 0xFFFFFFFF) < 0x80000000:
            st.last_seq = h.seq
        elif 0 < ((st.first_seq - h.seq) & 0xFFFFFFFF) < 1000:
            st.first_seq = h.seq                     # an earlier packet that arrived late
        if h.msg_type == P.MSG_HEALTH:
            st.health = body
            return []
        obs: P.FrameObs = body
        seen = self._seen.setdefault(h.station_id, set())
        if not obs.flags & P.FLAG_PART:
            if obs.frame_id in seen:
                return []
            return [self._frame(obs, obs.markers, seen)]
        key = (h.station_id, obs.frame_id)
        parts = self._parts.setdefault(key, [])
        parts.append(obs)
        have = sum(len(p.markers) for p in parts)
        if have >= (obs.n_markers_total or 0) and any(p.flags & P.FLAG_LAST_PART for p in parts):
            del self._parts[key]
            parts.sort(key=lambda p: p.part_index)
            return [self._frame(parts[0], [p.markers for p in parts], seen)]
        if len(self._parts) > 256:                           # forget parts of frames that never completed
            for k in list(self._parts)[:128]:
                del self._parts[k]
        return []

    @staticmethod
    def _frame_obs(h: P.Header, data: bytes) -> P.FrameObs:
        """FRAME_OBS with its markers as one structured array (no per-marker objects)."""
        if h.payload_len < P.FRAME.size:
            raise P.ProtocolError("short frame")
        (frame_id, t_mid, exp, proc, sync, part, n, n_total, thr, drops) = P.FRAME.unpack_from(data, P.HEADER.size)
        if h.payload_len != P.FRAME.size + n * P.MARKER.size:
            raise P.ProtocolError("marker count mismatch")
        m = np.frombuffer(data, MARKER_DT, count=n, offset=P.HEADER.size + P.FRAME.size)
        return P.FrameObs(h.station_id, frame_id, t_mid, exp, m, proc, sync, thr, drops, n_total, part, h.flags)

    def _frame(self, obs: P.FrameObs, markers, seen) -> CameraFrame:
        seen.add(obs.frame_id)
        if len(seen) > 512:
            for fid in sorted(seen)[:256]:
                seen.discard(fid)
        self.stations[obs.station_id].frames += 1
        m = markers if isinstance(markers, np.ndarray) else (
            np.concatenate(markers) if markers and isinstance(markers[0], np.ndarray) else
            np.array([(k.x, k.y, k.sum, k.peak, k.area, k.mflags) for k in markers], MARKER_DT))
        xy = np.empty((len(m), 2))
        xy[:, 0], xy[:, 1] = m["x"], m["y"]
        return CameraFrame(obs.station_id, obs.frame_id, obs.t_mid_ns, xy, m["sum"].astype(float),
                           m["peak"].astype(float), m["mflags"].astype(int), obs.sync_state, obs.flags)
