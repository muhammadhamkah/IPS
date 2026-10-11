"""The brain: always-on monitoring of tracking and station health.

It needs nothing from the operator. Fed with per-drone track samples and
per-station status (from the positioning engine, or the simulator), it:

* records **loss episodes**: a drone seen by fewer cameras than the requirement
  (degraded) or by fewer than two (lost) for at least `min_duration_s`;
* finds **patterns**: loss episodes clustering in one region of the volume over a
  sliding window, from several drones or over a long enough span, so a one-off
  occlusion by another drone is not reported;
* works out **why**: which cameras the model expects to see the region but the
  data shows do not (blocked line of sight), or that the model itself is thin there;
* asks the Advisor, restricted to that region, for the **fix** (re-aim, move or add a
  camera, with tape-measure dimensions);
* watches **stations**: a camera that stops sending, or one that has been knocked
  (live re-pose differs from calibration);
* follows up: an accepted fix is confirmed when drones pass through the region
  without losses; an issue the operator chose to live with is re-raised only if it
  gets clearly worse; quiet issues resolve themselves.

Recommendations are computed outside the feed loop (they take seconds): the
brain lists issues needing analysis (`pending()`), the caller runs
`analyse(issue)` (thread-safe; pure function of a snapshot) and hands the result
back with `set_analysis()`. `analyse_pending()` does both inline.
"""
from __future__ import annotations

import math
from collections import Counter, deque
from dataclasses import dataclass, field

import numpy as np

from idps.coverage.fisher import contributions
from idps.monitor.model import (BLIND_SPOT, CAMERA_MOVED, CAMERA_OFFLINE, FIX_ACCEPTED, LIVED_WITH, OPEN, RESOLVED,
                                Issue, StationStatus, TrackSample)
from idps.plan.model import PlannedCamera
from idps.plan.site import Site


@dataclass
class LossEvent:
    t0: float
    t1: float
    drone_id: int
    pos: np.ndarray                   # mean position during the episode
    worst: int                        # fewest cameras seen
    expected: frozenset               # cameras the model expects to see pos
    seen_frac: dict                   # camera -> fraction of the episode it saw the drone
    faulty: frozenset                 # cameras offline or knocked at the time
    missed: dict = field(default_factory=dict)   # camera -> its position: had the drone in view, did not see it

    @property
    def lost(self) -> bool:
        return self.worst < 2


@dataclass
class _Episode:
    t0: float
    t_last: float
    pos: list = field(default_factory=list)
    seen: Counter = field(default_factory=Counter)
    n: int = 0
    worst: int = 99


@dataclass
class BrainSettings:
    window_s: float = 180.0           # pattern memory
    cell_m: float = 2.0               # clustering resolution
    min_duration_s: float = 0.2       # shorter dips are ignored (the tracker coasts through them)
    chunk_s: float = 2.0              # a long episode counts once per chunk (a drone parked in a hole)
    min_events: int = 5               # a pattern needs this many episodes ...
    min_drones: int = 2               # ... from this many drones ...
    min_span_s: float = 30.0          # ... or spread over this long
    quiet_s: float = 90.0             # no losses this long, while drones pass through, resolves an issue
    min_passes: int = 30              # samples of drones passing through the region needed to call it quiet
    offline_after_s: float = 1.0
    moved_warn_mrad: float = 0.5
    moved_crit_mrad: float = 2.0
    worse_factor: float = 2.0         # re-raise a lived-with issue when its rate doubles
    blocked_seen_frac: float = 0.5    # an expected camera seeing drones there less often is called blocked
    cluster_every_s: float = 1.0
    settle_s: float = 5.0             # after an accepted fix, ignore losses this long (crew, servos settling)


class Brain:
    def __init__(self, site: Site, cameras: list[PlannedCamera], settings: BrainSettings | None = None):
        self.site = site
        self.s = settings or BrainSettings()
        self.min_cameras = site.requirements.min_cameras
        self.events: deque[LossEvent] = deque()
        self.issues: dict[str, Issue] = {}
        self._ep: dict[int, _Episode] = {}
        self._recent: deque = deque()          # (t, pos, seen_by): every sample in the window, for diagnosis
        self._last_online: dict[str, float] = {}
        self._online: dict[str, bool] = {}
        self._moved: dict[str, float] = {}
        self._last_cluster = -math.inf
        self._next_id = 1
        self.now = 0.0
        self.samples_seen = 0
        self.set_cameras(cameras)

    # ------------------------------------------------------------ inputs
    def set_cameras(self, cameras: list[PlannedCamera]):
        """The layout changed (an accepted fix was installed, or calibration updated poses)."""
        self.cams = list(cameras)
        self._ids = [c.id for c in self.cams]
        if self.cams:
            self._pos = np.array([c.position_m for c in self.cams], float)
            self._R = np.array([c.R for c in self.cams])
            self._f = np.array([self.site.sensor.f_px(c.lens_mm) for c in self.cams])
        for cid in list(self._last_online):
            if cid not in self._ids:
                self._last_online.pop(cid)
                self._online.pop(cid, None)
                self._moved.pop(cid, None)

    def expected(self, pos) -> frozenset:
        """Cameras the model says should see a point."""
        if not self.cams:
            return frozenset()
        vis, _ = contributions(self._pos, self._R, self._f, np.atleast_2d(pos), self.site.sensor,
                               self.site.requirements.noise, self.site.limits)
        return frozenset(self._ids[k] for k in np.nonzero(vis[:, 0])[0])

    @property
    def faulty(self) -> frozenset:
        """Cameras currently offline or knocked out of calibration."""
        off = {c for c, on in self._online.items() if not on}
        moved = {c for c, m in self._moved.items() if m >= self.s.moved_warn_mrad}
        return frozenset(off | moved)

    @property
    def offline(self) -> frozenset:
        return frozenset(c for c, on in self._online.items() if not on)

    def feed(self, samples: list[TrackSample], statuses: list[StationStatus] = ()) -> list[tuple[Issue, str]]:
        """Consume one tick of data. Returns (issue, change) for issues that are new,
        worse, updated or resolved, so the caller can alert the operator."""
        changes: list[tuple[Issue, str]] = []
        ts = [x.t for x in samples] + [x.t for x in statuses]
        if ts:
            self.now = max(self.now, max(ts))
        for st in statuses:
            changes += self._station(st)
        changes += self._station_timeouts()
        for smp in samples:
            self._sample(smp)
            self._recent.append((smp.t, np.asarray(smp.pos, float), smp.seen_by))
        self.samples_seen += len(samples)
        while self.events and self.events[0].t1 < self.now - self.s.window_s:
            self.events.popleft()
        while self._recent and self._recent[0][0] < self.now - self.s.window_s:
            self._recent.popleft()
        if self.now - self._last_cluster >= self.s.cluster_every_s:
            self._last_cluster = self.now
            changes += self._patterns()
            changes += self._resolve_quiet()
        return changes

    # ------------------------------------------------------------ stations
    def _station(self, st: StationStatus):
        out = []
        cid = st.camera_id
        if st.online:
            self._last_online[cid] = st.t
        else:
            self._last_online.setdefault(cid, st.t - self.s.offline_after_s)
        was = self._online.get(cid, True)
        now_on = st.online or (st.t - self._last_online.get(cid, st.t)) < self.s.offline_after_s
        self._online[cid] = now_on
        if was and not now_on:
            out += self._raise_offline(cid)
        elif not was and now_on:
            out += self._close(CAMERA_OFFLINE, cid, "back online")
        m = float(st.moved_mrad)
        prev = self._moved.get(cid, 0.0)
        self._moved[cid] = m
        if m >= self.s.moved_warn_mrad and now_on:
            out += self._raise_moved(cid, m, prev)
        elif prev >= self.s.moved_warn_mrad and m < self.s.moved_warn_mrad:
            out += self._close(CAMERA_MOVED, cid, "back on its calibrated view")
        return out

    def _station_timeouts(self):
        out = []
        for cid, t in self._last_online.items():
            if self._online.get(cid, True) and self.now - t >= self.s.offline_after_s:
                self._online[cid] = False
                out += self._raise_offline(cid)
        return out

    def _find(self, kind, camera=None):
        for iss in self.issues.values():
            if iss.kind == kind and (camera is None or camera in iss.cameras) and iss.state != RESOLVED:
                return iss
        return None

    def _new_issue(self, **kw) -> Issue:
        kw.setdefault("last_seen", self.now)
        iss = Issue(id=f"A{self._next_id:03d}", first_seen=self.now, **kw)
        self._next_id += 1
        self.issues[iss.id] = iss
        return iss

    def _cam(self, cid) -> PlannedCamera | None:
        return next((c for c in self.cams if c.id == cid), None)

    def _raise_offline(self, cid):
        iss = self._find(CAMERA_OFFLINE, cid)
        if iss is not None:
            iss.last_seen = self.now
            return []
        cam = self._cam(cid)
        where = (f" ({cam.mount_type} {cam.mount_id}, X {cam.position_m[0]:.1f}, Y {cam.position_m[1]:.1f}, "
                 f"{cam.position_m[2]:.1f} m high)") if cam else ""
        iss = self._new_issue(kind=CAMERA_OFFLINE, severity="critical", title=f"{cid} stopped sending",
                              evidence=f"No data from {cid}{where} for over {self.s.offline_after_s:g} s. "
                                       f"Check its power and network cable / Wi-Fi.",
                              location=np.array(cam.position_m) if cam else None, radius_m=1.0, cameras=[cid],
                              action="apply")
        iss.fix_text = f"Get {cid} back online (power, cable, Wi-Fi). Looking for a stop-gap re-aim of its neighbours…"
        iss.log(self.now, "Station stopped sending")
        return [(iss, "new")]

    def _raise_moved(self, cid, m, prev):
        iss = self._find(CAMERA_MOVED, cid)
        sev = "critical" if m >= self.s.moved_crit_mrad else "warning"
        cam = self._cam(cid)
        if iss is not None:
            iss.last_seen = self.now
            iss.evidence = self._moved_text(cid, m)
            if iss.state == LIVED_WITH and m >= max(prev, 1e-9) * self.s.worse_factor and iss.severity != sev:
                iss.state, iss.severity = OPEN, sev
                iss.log(self.now, f"Moved further ({m:.1f} mrad)")
                return [(iss, "worse")]
            if iss.severity != sev and iss.state == OPEN:
                iss.severity = sev
                return [(iss, "updated")]
            return []
        iss = self._new_issue(kind=CAMERA_MOVED, severity=sev, title=f"{cid} has been knocked",
                              evidence=self._moved_text(cid, m),
                              location=np.array(cam.position_m) if cam else None, radius_m=1.0, cameras=[cid],
                              action="restore")
        iss.fix_text = (f"Restore {cid}: the gimbal servos turn it back to its calibrated view and the saved "
                        f"calibration is reloaded (no re-wand). If the tripod itself was moved, put it back on "
                        f"its floor marks first.")
        iss.log(self.now, f"View moved {m:.1f} mrad from calibration")
        return [(iss, "new")]

    def _moved_text(self, cid, m):
        mm_at_10 = m * 10.0
        return (f"Live re-pose shows {cid}'s view has moved {m:.1f} mrad from its calibration (about "
                f"{mm_at_10:.0f} mm at 10 m), so its rays are rejected and nearby drones lose accuracy.")

    def _close(self, kind, cid, why):
        iss = self._find(kind, cid)
        if iss is None:
            return []
        iss.state = RESOLVED
        iss.last_seen = self.now
        iss.log(self.now, f"Resolved: {cid} {why}")
        return [(iss, "resolved")]

    # ------------------------------------------------------------ tracking
    def _sample(self, smp: TrackSample):
        n = smp.n_cams
        ep = self._ep.get(smp.drone_id)
        if n < self.min_cameras:
            if ep is None:
                ep = self._ep[smp.drone_id] = _Episode(smp.t, smp.t)
            ep.t_last = smp.t
            ep.pos.append(np.asarray(smp.pos, float))
            ep.seen.update(smp.seen_by)
            ep.n += 1
            ep.worst = min(ep.worst, n)
            if smp.t - ep.t0 >= self.s.chunk_s:
                self._end_episode(smp.drone_id, ep, ongoing=True)
        else:
            if ep is not None:
                self._end_episode(smp.drone_id, ep)
            for iss in self.issues.values():
                if iss.kind == BLIND_SPOT and iss.state != RESOLVED and iss.location is not None:
                    if np.linalg.norm(np.asarray(smp.pos) - iss.location) <= iss.radius_m:
                        iss.passes += 1

    def _end_episode(self, drone, ep: _Episode, ongoing=False):
        self._ep.pop(drone, None)
        if ongoing:
            self._ep[drone] = _Episode(ep.t_last, ep.t_last)
        if ep.t_last - ep.t0 < self.s.min_duration_s and not ongoing:
            return
        pos = np.mean(ep.pos, axis=0)
        frac = {c: k / ep.n for c, k in ep.seen.items()}
        exp = self.expected(pos)
        missed = {c: tuple(self._cam(c).position_m) for c in exp if frac.get(c, 0.0) < 0.5 and self._cam(c)}
        ev = LossEvent(ep.t0, ep.t_last, drone, pos, ep.worst, exp, frac, self.faulty, missed)
        self.events.append(ev)
        for iss in self.issues.values():          # a loss in an issue's region: not quiet any more
            if iss.kind == BLIND_SPOT and iss.state != RESOLVED and iss.location is not None:
                if np.linalg.norm(pos - iss.location) <= iss.radius_m:
                    iss.passes = 0
                    iss.last_seen = max(iss.last_seen, ev.t1)

    # ------------------------------------------------------------ patterns
    def _explained(self, ev: LossEvent) -> bool:
        """The loss is explained by a camera that was offline or knocked at the time."""
        return bool(ev.expected & ev.faulty)

    def clusters(self):
        """Connected groups of 'cell_m' cells holding unexplained loss events."""
        cs = self.s.cell_m
        cells: dict[tuple, list[LossEvent]] = {}
        for ev in self.events:
            if not self._explained(ev):
                cells.setdefault(tuple(np.floor(ev.pos / cs).astype(int)), []).append(ev)
        seen, out = set(), []
        for start in cells:
            if start in seen:
                continue
            stack, comp = [start], []
            seen.add(start)
            while stack:
                c = stack.pop()
                comp += cells[c]
                for d in np.ndindex(3, 3, 3):
                    nb = (c[0] + d[0] - 1, c[1] + d[1] - 1, c[2] + d[2] - 1)
                    if nb in cells and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
            out.append(comp)
        return out

    def _is_pattern(self, evs) -> bool:
        drones = {e.drone_id for e in evs}
        span = max(e.t1 for e in evs) - min(e.t0 for e in evs)
        if len(drones) >= self.s.min_drones:
            return len(evs) >= self.s.min_events
        # one drone: it may just be its own odd attitude there, so ask for more, over a longer span
        return len(evs) >= 2 * self.s.min_events and span >= self.s.min_span_s

    def _patterns(self):
        out = []
        for evs in self.clusters():
            if not self._is_pattern(evs):
                continue
            P = np.array([e.pos for e in evs])
            centre = P.mean(0)
            radius = float(np.clip(np.linalg.norm(P - centre, axis=1).max() + self.s.cell_m / 2, 1.5, 6.0))
            iss = None
            for cand in self.issues.values():
                if cand.kind == BLIND_SPOT and cand.state != RESOLVED and cand.location is not None and \
                        np.linalg.norm(cand.location - centre) <= cand.radius_m + self.s.cell_m:
                    iss = cand
                    break
            info = self._describe(evs, centre, radius)
            if iss is None:                      # the same place came back after resolving: reopen it
                for cand in self.issues.values():
                    if cand.kind == BLIND_SPOT and cand.state == RESOLVED and cand.location is not None and \
                            np.linalg.norm(cand.location - centre) <= cand.radius_m + self.s.cell_m:
                        iss = cand
                        iss.state = OPEN
                        iss.reset_analysis()
                        for k in ("severity", "title", "evidence", "location", "radius_m", "cameras", "count",
                                  "drones"):
                            setattr(iss, k, info[k])
                        iss.last_seen = info["last_seen"]
                        iss.log(self.now, "Came back")
                        out.append((iss, "worse"))
                        break
                if iss is not None:
                    continue
            if iss is None:
                iss = self._new_issue(kind=BLIND_SPOT, **info)
                iss.log(self.now, f"Pattern found: {iss.count} losses from {len(iss.drones)} drone(s)")
                out.append((iss, "new"))
                continue
            rate = self._rate(evs)
            for k in ("severity", "title", "evidence", "location", "radius_m", "cameras", "count", "drones"):
                setattr(iss, k, info[k])
            iss.last_seen = max(iss.last_seen, info["last_seen"])
            if iss.state == LIVED_WITH:
                if rate >= self.s.worse_factor * max(iss.lived_rate, 1e-9) and len(evs) >= 2 * self.s.min_events:
                    iss.state = OPEN
                    iss.severity = "critical"
                    iss.reset_analysis()                              # re-analyse with the new evidence
                    iss.log(self.now, f"Worse than when you chose to live with it: {rate:.1f} losses/min "
                                      f"(was {iss.lived_rate:.1f})")
                    out.append((iss, "worse"))
            elif iss.state == FIX_ACCEPTED:
                post = [e for e in evs if e.t0 >= iss.accepted_at + self.s.settle_s]
                if not post or not self._is_pattern(post):
                    continue                          # judge the fix on data after it settled
                iss.state = OPEN
                iss.reset_analysis()
                iss.log(self.now, "Still happening after the fix: looking for the next step")
                out.append((iss, "worse"))
        return out

    def blocked_cameras(self, centre, radius) -> list[str]:
        """Cameras the model expects to see drones in the region but that see them there far
        less often than they see drones elsewhere. Judged on every sample, not only the
        losses (which would make every camera look blocked); a partial obstruction (a banner
        across part of the view) counts too."""
        if len(self._recent) < 20 or not self.cams:
            return []
        rec = list(self._recent)
        if len(rec) > 4000:
            rec = rec[::len(rec) // 4000 + 1]
        P = np.array([p for _, p, _ in rec])
        inside = np.linalg.norm(P - centre, axis=1) <= radius
        if inside.sum() < 10:
            return []
        vis, _ = contributions(self._pos, self._R, self._f, P, self.site.sensor, self.site.requirements.noise,
                               self.site.limits)
        faulty = self.faulty
        out = []
        for k, cid in enumerate(self._ids):
            if cid in faulty:
                continue
            seen = np.array([cid in s for _, _, s in rec])
            ein, eout = vis[k] & inside, vis[k] & ~inside
            if ein.sum() < max(10, 0.5 * inside.sum()):
                continue
            r_in = seen[ein].mean()
            r_out = seen[eout].mean() if eout.sum() >= 20 else 0.95
            if r_in < self.s.blocked_seen_frac or r_in < 0.6 * r_out:
                out.append(cid)
        return sorted(out)

    def _rate(self, evs) -> float:
        span = max(self.now - min(e.t0 for e in evs), 30.0)
        return len(evs) / (min(span, self.s.window_s) / 60.0)

    def _describe(self, evs, centre, radius) -> dict:
        drones = {e.drone_id for e in evs}
        lost = sum(e.lost for e in evs)
        z = np.array([e.pos[2] for e in evs])
        longest = max(e.t1 - e.t0 for e in evs)
        blocked = self.blocked_cameras(centre, radius)
        what = "lose tracking" if lost >= 0.3 * len(evs) else f"drop below {self.min_cameras} cameras"
        title = (f"Drones {what} near X {centre[0]:.0f}, Y {centre[1]:.0f} m, "
                 f"{max(z.min(), 0):.0f}–{z.max():.0f} m high")
        minutes = min(self.now - min(e.t0 for e in evs), self.s.window_s) / 60
        ev_text = (f"{len(evs)} episodes in the last {max(minutes, 0.1):.1f} min from {len(drones)} drone(s) "
                   f"({lost} with tracking lost, longest {longest:.1f} s). ")
        # In the losses themselves: which cameras had the drone in view (by the plan) yet missed it?
        miss = Counter(c for e in evs for c in e.expected if e.seen_frac.get(c, 0.0) < 0.5)
        missing = sorted(c for c, k in miss.items() if k >= 0.6 * len(evs) and c not in self.faulty)
        names = blocked or missing
        def looks_up(cid):          # camera more than ~15° below the drones: it sees their underside
            cam = self._cam(cid)
            if cam is None:
                return False
            d = np.asarray(cam.position_m) - centre
            return math.degrees(math.atan2(d[2], math.hypot(d[0], d[1]))) < -15.0
        below = [c for c in names if looks_up(c)]
        many = len(names) > 1
        if names and len(below) >= 0.6 * len(names):
            ev_text += (f"{', '.join(names)} had the drones in view but did not see them: "
                        f"{'they are' if many else 'it is'} below the drones there, which breaks the rule that "
                        f"drones always fly below the cameras (the marker on top is hidden by the drone's body). "
                        f"Raise {'them' if many else 'it'} above the flight ceiling, or keep the drones lower.")
        elif blocked:
            ev_text += (f"{', '.join(blocked)} should see this region but see drones there far less often than "
                        f"elsewhere: likely a blocked line of sight (set pieces, rigging, a banner).")
        elif missing:
            ev_text += (f"In these losses {', '.join(missing)} had the drones in view but did not see them: "
                        f"something is in the way some of the time (set pieces, rigging, other drones).")
        else:
            ev_text += ("The cameras that should see it do, but too few at once there: the region needs more "
                        "coverage than the plan predicted.")
        return dict(severity="critical" if lost >= 0.3 * len(evs) else "warning", title=title, evidence=ev_text,
                    location=centre, radius_m=radius, cameras=blocked, count=len(evs), drones=drones,
                    last_seen=max(e.t1 for e in evs), action="apply")

    def _resolve_quiet(self):
        out = []
        for iss in self.issues.values():
            if iss.kind != BLIND_SPOT or iss.state == RESOLVED:
                continue
            if self.now - iss.last_seen >= self.s.quiet_s and iss.passes >= self.s.min_passes:
                was = iss.state
                iss.state = RESOLVED
                iss.log(self.now, "Fix confirmed: drones pass through without losing tracking" if was == FIX_ACCEPTED
                        else "Quiet: no more losses while drones pass through")
                self._purge(iss)
                out.append((iss, "resolved"))
        return out

    def _purge(self, iss: Issue):
        if iss.location is None:
            return
        keep = [e for e in self.events if np.linalg.norm(e.pos - iss.location) > iss.radius_m + self.s.cell_m]
        self.events = deque(keep)

    # ------------------------------------------------------------ operator
    def accept(self, issue_id: str) -> Issue:
        """The operator accepted the recommended fix (the caller applies it)."""
        iss = self.issues[issue_id]
        iss.state = FIX_ACCEPTED
        iss.accepted_at = self.now
        iss.passes = 0
        self._ep.clear()                 # episodes under way began before the fix
        iss.last_seen = self.now
        iss.log(self.now, "Fix accepted" + (": " + ", ".join(r.title for r in iss.fixes) if iss.fixes else ""))
        iss.rounds += 1
        if iss.kind == BLIND_SPOT and iss.location is not None:
            near = [e for e in self.events if np.linalg.norm(e.pos - iss.location) <= iss.radius_m + self.s.cell_m]
            iss.evidence_events = (iss.evidence_events + near)[-300:]     # what each camera missed, and from where
            self._purge(iss)             # judge the fix on fresh data only
        return iss

    def live_with(self, issue_id: str) -> Issue:
        """The operator chose to live with it: stop alerting unless it gets clearly worse."""
        iss = self.issues[issue_id]
        iss.state = LIVED_WITH
        if iss.kind == BLIND_SPOT and iss.location is not None:
            evs = [e for e in self.events if np.linalg.norm(e.pos - iss.location) <= iss.radius_m + self.s.cell_m]
            iss.lived_rate = self._rate(evs) if evs else 0.0
        iss.log(self.now, "Operator chose to live with it")
        return iss

    def mark_resolved(self, issue_id: str, why: str = "fixed"):
        iss = self.issues[issue_id]
        iss.state = RESOLVED
        iss.log(self.now, f"Resolved: {why}")
        if iss.kind == BLIND_SPOT:
            self._purge(iss)
        if iss.kind == CAMERA_MOVED:
            for c in iss.cameras:
                self._moved[c] = 0.0

    # ------------------------------------------------------------ analysis
    def pending(self) -> list[Issue]:
        return [i for i in self.issues.values()
                if i.state == OPEN and i.action == "apply" and not i.fixes and not i.analysing
                and not i.no_fix]

    def snapshot(self, iss: Issue):
        """Everything analyse() needs, copied so it can run in a worker thread."""
        loss_pos, missed = np.zeros((0, 3)), []
        if iss.location is not None:
            evs = [e for e in self.events if np.linalg.norm(e.pos - iss.location) <= iss.radius_m][-200:]
            evs = [e for e in iss.evidence_events if np.linalg.norm(e.pos - iss.location) <= iss.radius_m
                   + self.s.cell_m][-150:] + evs
            if evs:
                loss_pos = np.array([e.pos for e in evs])
                missed = [dict(e.missed) for e in evs]
        return (self.site, list(self.cams), iss.kind, iss.location, iss.radius_m, frozenset(iss.cameras),
                frozenset(self.faulty), self.min_cameras, loss_pos, missed, iss.rounds)

    @staticmethod
    def analyse(snapshot, progress=None, cancel=None) -> dict:
        """Find the fix for one issue. Returns dict(recs, focus, text)."""
        from idps.plan.advisor import Advisor, Focus
        site, cams, kind, loc, radius, blocked, faulty, min_c, loss_pos, missed, rounds = snapshot
        if kind == CAMERA_OFFLINE:
            off = set(blocked)
            adv = Advisor(site, cams, offline=off | set(faulty))
            if adv.health().failing == 0:
                return dict(recs=[], focus=None, text=f"The other cameras still give every point the required "
                                                      f"cameras: no stop-gap needed. Get {', '.join(sorted(off))} "
                                                      f"back online for margin.")
            recs = [r for r in adv.recommend(top=3, kinds=("reaim",), progress=progress) if r.points_fixed > 0]
            text = (f"Stop-gap while {', '.join(sorted(off))} is down: {recs[0].title} (motorised, nobody climbs) to "
                    f"cover part of its area." if recs else
                    f"No motorised re-aim covers its area: get {', '.join(sorted(off))} back online.")
            return dict(recs=recs, focus=None, text=text)
        from idps.plan.advisor import auto_fix
        # Where drones were lost, the measured visibility is used (a camera that should see a
        # point but did not is blocked there); elsewhere in the region the blocked cameras are
        # removed. If no change meets the requirement there, ask for one camera of margin.
        # Line-of-sight evidence: a camera that had the drone in view but missed it, from the
        # position it is still at (a re-aim keeps the obstruction in the way; a move clears it).
        block_at = None
        if len(loss_pos) and cams:
            block_at = {}
            for c in cams:
                here = np.asarray(c.position_m, float)
                B = np.array([c.id in m and np.linalg.norm(np.asarray(m[c.id]) - here) < 0.01 for m in missed])
                if B.any():
                    block_at[c.id] = (tuple(c.position_m), B)
        tries = [min_c, min_c + 1]
        for need in tries:
            if cancel is not None and cancel():
                break
            focus = Focus(tuple(np.asarray(loc, float)), float(radius), frozenset(blocked), need,
                          loss_pos if len(loss_pos) else None, block_at)
            # the best single change first; if none helps on its own, a short ordered list
            # After two rounds that did not cure it, moving cameras around has not escaped whatever is
            # in the way: only a camera from a new direction is worth the crew's time.
            kinds = ("reaim", "move", "add") if rounds < 2 else ("add",)
            steps, _ = auto_fix(site, cams, max_steps=3, kinds=kinds, focus=focus,
                                offline=set(faulty) - set(blocked), progress=progress, cancel=cancel)
            if steps and steps[-1].failing_after < steps[0].failing_before:
                steps = Brain._trim(site, cams, steps, focus, set(faulty) - set(blocked))
                extra = need - min_c
                why = ("so the places where drones were lost get the required cameras, counting only views "
                       "that actually saw them" if not extra else
                       f"planning the region for {need} cameras (requirement {min_c}) for margin")
                head = steps[0].title if len(steps) == 1 else f"{len(steps)} changes"
                text = f"{head}, {why}."
                if steps[-1].failing_after > 0:
                    text += (f" This only partly fixes it ({steps[0].failing_before} → {steps[-1].failing_after} "
                             f"problem spots): for the rest, clear the obstruction or add a tripod zone or truss "
                             f"near X {loc[0]:.0f}, Y {loc[1]:.0f} so the advisor has somewhere better to put a camera.")
                return dict(recs=steps, focus=focus, text=text)
        return dict(recs=[], focus=None, text="")

    @staticmethod
    def _trim(site, cams, steps, focus, offline):
        """Drop steps that remove no problem spot, if the rest still does as well; renumber."""
        from dataclasses import replace
        from idps.plan.advisor import Advisor, apply

        def failing(cs, f):
            a = Advisor(site, cs, focus=f, offline=offline)
            return int(a._failing(a.n, a.info).sum())

        def run(seq):
            cs, f, out = list(cams), focus, []
            before = failing(cs, f)
            for r in seq:
                cs = apply(cs, r)
                if r.kind == "move" and r.camera_id in f.blocked:
                    f = replace(f, blocked=f.blocked - {r.camera_id})
                after = failing(cs, f)
                out.append(replace(r, failing_before=before, failing_after=after))
                before = after
            return out
        useful = [r for r in steps if r.points_fixed > 0]
        if useful and len(useful) < len(steps):
            trimmed = run(useful)
            if trimmed[-1].failing_after <= steps[-1].failing_after:
                return trimmed
        return steps

    def set_analysis(self, issue_id: str, result: dict | None):
        iss = self.issues.get(issue_id)
        if iss is None:
            return
        iss.analysing = False
        if not result or not result["recs"]:
            iss.no_fix = True
            iss.fixes = []
            if iss.kind == BLIND_SPOT:
                iss.fix_text = ("No camera change from the allowed mounts improves this region. Clear the "
                                "obstruction, or add a tripod zone or truss near it so the advisor has somewhere "
                                "better to put a camera.")
                iss.log(self.now, "No fix found with the allowed mounts")
            else:
                iss.fix_text = (result or {}).get("text") or iss.fix_text
                if "no stop-gap" in iss.fix_text:
                    iss.severity = "warning"           # covered by the others: urgent, not critical
                    iss.log(self.now, "No stop-gap needed: the others cover its area")
                else:
                    iss.log(self.now, "No stop-gap found")
            return
        iss.no_fix = False
        if iss.kind == CAMERA_OFFLINE:
            iss.fixes = result["recs"][:1]                # one stop-gap re-aim
        else:
            iss.fixes = list(result["recs"])              # ordered changes, applied together
        iss.focus = result["focus"]
        iss.fix_text = result["text"]
        if iss.kind == BLIND_SPOT and iss.rounds >= 2:
            iss.fix_text = (f"{iss.rounds} rounds of changes have not cured this, so something physical is "
                            f"probably in the way near X {iss.location[0]:.0f}, Y {iss.location[1]:.0f}, "
                            f"{iss.location[2]:.0f} m high: walk the lines of sight from "
                            f"{', '.join(iss.cameras) or 'the cameras around it'} to that spot first. "
                            f"Otherwise: {iss.fix_text}")
        iss.log(self.now, "Recommended: " + ", ".join(r.title for r in iss.fixes))

    def analyse_pending(self):
        """Run analysis for every pending issue inline (tests, headless use)."""
        for iss in self.pending():
            iss.analysing = True
            self.set_analysis(iss.id, self.analyse(self.snapshot(iss)))

    # ------------------------------------------------------------ summary
    def active_issues(self) -> list[Issue]:
        return [i for i in self.issues.values() if i.state in (OPEN, FIX_ACCEPTED)]

    def status_line(self) -> str:
        act = [i for i in self.issues.values() if i.state == OPEN]
        if not act:
            return "All good: no repeated tracking losses, every station online and on its calibrated view."
        crit = sum(i.severity == "critical" for i in act)
        return f"{len(act)} open issue(s), {crit} critical."
