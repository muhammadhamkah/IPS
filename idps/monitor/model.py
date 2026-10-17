"""Data the brain consumes and the issues it raises.

The positioning engine (or the simulator, until the engine exists) produces a
TrackSample per drone per tick and a StationStatus per camera station. The
brain turns those into Issues: each one has evidence, a location, and a
recommended fix; the operator accepts the fix or chooses to live with it."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class TrackSample:
    t: float                          # seconds (show clock)
    drone_id: int
    pos: np.ndarray                   # (3,) best estimate; while lost, the last known/predicted position
    seen_by: frozenset                # camera ids that detected the drone's marker this tick

    @property
    def n_cams(self) -> int:
        return len(self.seen_by)


@dataclass
class StationStatus:
    t: float
    camera_id: str
    online: bool = True               # packets arriving (the engine: data within the last second)
    fps: float = 0.0
    moved_mrad: float = 0.0           # live re-pose against beacons / drone markers vs calibration


# Issue kinds
BLIND_SPOT = "blind_spot"             # drones repeatedly lose tracking in one region
CAMERA_OFFLINE = "camera_offline"
CAMERA_MOVED = "camera_moved"

# Issue states
OPEN = "open"
FIX_ACCEPTED = "fix_accepted"         # operator accepted; waiting for the data to confirm it worked
LIVED_WITH = "lived_with"             # operator chose to live with it; re-raised only if it gets worse
RESOLVED = "resolved"


@dataclass
class Issue:
    id: str
    kind: str
    severity: str                     # "warning" | "critical"
    title: str
    evidence: str
    location: np.ndarray | None = None
    radius_m: float = 0.0
    cameras: list = field(default_factory=list)    # cameras involved (offline / moved / blocked)
    first_seen: float = 0.0
    last_seen: float = 0.0
    count: int = 0                    # loss events (blind spot) or seconds affected
    drones: set = field(default_factory=set)
    state: str = OPEN
    action: str | None = None         # "apply" (advisor change) | "restore" (re-aim to calibration)
    fixes: list = field(default_factory=list)      # idps.plan.advisor.Recommendation, applied in order
    fix_text: str = ""                # what accepting does, in plain words
    analysing: bool = False           # a recommendation is being computed
    focus: object = None              # idps.plan.advisor.Focus used for the recommendation
    rounds: int = 0                   # fixes accepted for this issue so far
    evidence_events: list = field(default_factory=list)   # loss events kept across fix rounds
    accepted_at: float = -1.0         # show time the operator accepted the fix
    passes: int = 0                   # drone samples through the region since its last loss
    no_fix: bool = False              # analysis found no single change that helps
    lived_rate: float = 0.0           # events per minute when the operator chose to live with it
    history: list = field(default_factory=list)    # (t, text) timeline for the alert card

    @property
    def recommendation(self):
        return self.fixes[0] if self.fixes else None

    def reset_analysis(self):
        self.fixes, self.focus, self.no_fix, self.fix_text = [], None, False, ""

    @property
    def active(self) -> bool:
        return self.state in (OPEN, FIX_ACCEPTED)

    def log(self, t: float, text: str):
        self.history.append((t, text))
