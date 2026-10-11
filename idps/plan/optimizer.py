"""Layout optimiser: choose cameras from the candidate set.

Objective: drive the total *deficit* over the evaluation grid to zero, where
each voxel's deficit is

    coverage:  max(0, min_cameras - n_cameras)
    accuracy:  min(cap, max(0, rms - rms_limit) / rms_limit)

and rms is the predicted 3D RMS error from the summed Fisher information
(rms_limit = p95_mm / 1.62). A voxel seen by < 2 cameras has infinite rms and
takes the capped accuracy deficit.

Method: lazy greedy selection (repeatedly add the candidate that removes the
most deficit), then a pruning pass that drops cameras whose removal does not
increase the deficit. The objective is not strictly submodular, so lazy
greedy is a heuristic; it is fast and works well for coverage problems.
"""
from __future__ import annotations

import heapq
import time
from dataclasses import dataclass, field

import numpy as np

from idps.core.noise import P95_PER_RMS
from idps.coverage.fisher import contributions, rms_mm
from idps.plan.candidates import CandidateSet
from idps.plan.site import Site

ACCURACY_CAP = 4.0


@dataclass
class Objective:
    min_cameras: int
    rms_limit_mm: float
    w_coverage: float = 1.0
    w_accuracy: float = 1.0

    def voxel_deficit(self, n: np.ndarray, info: np.ndarray) -> np.ndarray:
        cov = np.maximum(0, self.min_cameras - n)
        rms = rms_mm(info)
        acc = np.minimum(ACCURACY_CAP, np.maximum(0.0, rms - self.rms_limit_mm) / self.rms_limit_mm)
        acc = np.where(np.isfinite(rms), acc, ACCURACY_CAP)
        return self.w_coverage * cov + self.w_accuracy * acc


@dataclass
class OptimiseResult:
    selected: list[int]                 # candidate indices
    deficit_history: list[float] = field(default_factory=list)
    final_deficit: float = 0.0
    seconds: float = 0.0
    stopped_because: str = ""


class Optimiser:
    def __init__(self, site: Site, cands: CandidateSet, points: np.ndarray, log=print):
        self.site, self.c, self.pts, self.log = site, cands, points, log
        self.R = cands.R
        req = site.requirements
        self.obj = Objective(req.min_cameras, req.p95_mm / P95_PER_RMS)

    def _contrib(self, idx):
        idx = np.atleast_1d(idx)
        return contributions(self.c.pos[idx], self.R[idx], self.c.f_px[idx], self.pts,
                             self.site.sensor, self.site.requirements.noise, self.site.limits)

    def _gains(self, idx, n, info, base_def):
        """Deficit reduction for each candidate in idx, given the current state."""
        vis, inf = self._contrib(idx)
        new_def = self.obj.voxel_deficit(n[None, :] + vis, info[None, :, :] + inf)
        return (base_def[None, :] - new_def).sum(axis=1)

    def run(self, max_cameras: int | None = None, batch: int = 256) -> OptimiseResult:
        t0 = time.time()
        max_cameras = max_cameras or self.site.max_cameras
        V, N = len(self.pts), len(self.c)
        n = np.zeros(V, int)
        info = np.zeros((V, 6))
        base_def = self.obj.voxel_deficit(n, info)
        total = float(base_def.sum())
        res = OptimiseResult(selected=[], deficit_history=[total])
        self.log(f"optimiser: {N} candidates, {V} evaluation points, initial deficit {total:.1f}")

        # Initial (upper-bound) gains for every candidate.
        heap = []
        for s in range(0, N, batch):
            idx = np.arange(s, min(N, s + batch))
            for i, g in zip(idx, self._gains(idx, n, info, base_def)):
                if g > 1e-9:
                    heap.append((-float(g), int(i)))
        heapq.heapify(heap)

        mount_use: dict[str, int] = {}
        cap = self.site.candidates.max_cameras_per_mount
        chosen_aims: set = set()
        res.stopped_because = "no candidate improves the layout"
        while heap:
            if total <= 1e-9:
                res.stopped_because = "all evaluation points meet the requirements"
                break
            if len(res.selected) >= max_cameras:
                res.stopped_because = f"reached max_cameras={max_cameras}"
                break
            _, i = heapq.heappop(heap)
            key = self.c.mount_key[i]
            if mount_use.get(key, 0) >= cap or (key, self.c.az[i], self.c.el[i]) in chosen_aims:
                continue
            g = float(self._gains([i], n, info, base_def)[0])
            if g <= 1e-9:
                continue
            if heap and g < -heap[0][0] - 1e-9:
                heapq.heappush(heap, (-g, i))      # stale bound: re-queue with the fresh gain
                continue
            vis, inf = self._contrib([i])
            n += vis[0]
            info += inf[0]
            base_def = self.obj.voxel_deficit(n, info)
            total = float(base_def.sum())
            res.selected.append(i)
            res.deficit_history.append(total)
            mount_use[key] = mount_use.get(key, 0) + 1
            chosen_aims.add((key, self.c.az[i], self.c.el[i]))
            if len(res.selected) % 10 == 0:
                self.log(f"  {len(res.selected):4d} cameras  deficit {total:9.2f}"
                         f"  ({time.time() - t0:5.1f} s)")

        res.selected = self._prune(res.selected)
        n, info = self.state(res.selected)
        res.final_deficit = float(self.obj.voxel_deficit(n, info).sum())
        res.seconds = time.time() - t0
        self.log(f"optimiser: {len(res.selected)} cameras after pruning, deficit {res.final_deficit:.2f},"
                 f" {res.seconds:.1f} s ({res.stopped_because})")
        return res

    def state(self, selected):
        V = len(self.pts)
        n = np.zeros(V, int)
        info = np.zeros((V, 6))
        for s in range(0, len(selected), 256):
            vis, inf = self._contrib(np.array(selected[s:s + 256]))
            n += vis.sum(0)
            info += inf.sum(0)
        return n, info

    def _prune(self, selected):
        """Drop cameras whose removal does not increase the total deficit."""
        if not selected:
            return selected
        sel = list(selected)
        n, info = self.state(sel)
        total = float(self.obj.voxel_deficit(n, info).sum())
        vis, inf = self._contrib(np.array(sel))
        # Try the least useful cameras first (fewest visible points).
        order = np.argsort(vis.sum(1))
        keep = np.ones(len(sel), bool)
        for k in order:
            n2 = n - vis[k]
            info2 = info - inf[k]
            t2 = float(self.obj.voxel_deficit(n2, info2).sum())
            if t2 <= total + 1e-9:
                keep[k] = False
                n, info, total = n2, info2, t2
        removed = int((~keep).sum())
        if removed:
            self.log(f"optimiser: pruning removed {removed} redundant cameras")
        return [s for s, k in zip(sel, keep) if k]
