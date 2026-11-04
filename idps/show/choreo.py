"""Choreography: build a drone show from formations, moves between them and rotations.

A show is a list of steps. Each step lasts a given time and ends with every drone at a known
place:

    show = Choreography(home)                         # (N, 3) take-off spots, show frame (x East, y North, z up)
    show.move_to(grid_1m, 4.0)                        # minimum-jerk move (smooth start and stop)
    show.hold(2.0)
    show.move_to(sphere, 6.0, assign=True)            # choose which drone goes where
    show.rotate(360, 12.0, centre=(0, 0, 2.5))        # about the vertical, eased in and out
    ...
    show.write_csv_zip("show.zip")                    # Skybrush CSV: the IDPS show tools read it

With assign=True every way of assigning drones to the new formation's places is tried (for up to
8 drones). The one kept has the largest minimum distance between any two drones during the
move, then the shortest longest path. Colours are set per step.
"""
from __future__ import annotations

import io
import itertools
import math
import zipfile

import numpy as np

RATE_HZ = 20.0


def min_jerk(s):
    """0..1 -> 0..1 with zero velocity and acceleration at both ends."""
    s = np.clip(s, 0.0, 1.0)
    return s ** 3 * (10 - 15 * s + 6 * s * s)


def grid(n: int, spacing: float, height: float, centre=(0.0, 0.0)) -> np.ndarray:
    """n drones in a row along x (East), `spacing` apart, at `height`."""
    x = (np.arange(n) - (n - 1) / 2) * spacing
    return np.column_stack([x + centre[0], np.full(n, centre[1]), np.full(n, height)])


def sphere5(radius: float, centre) -> np.ndarray:
    """5 points on a sphere: top, bottom and an equilateral triangle on the equator."""
    c = np.asarray(centre, float)
    a = np.radians([90, 210, 330])
    eq = np.column_stack([radius * np.cos(a), radius * np.sin(a), np.zeros(3)])
    return np.vstack([[0, 0, radius], [0, 0, -radius], eq]) + c


def square5(side: float, centre, vertical: bool = True) -> np.ndarray:
    """4 corners of a square and its centre; vertical: in the x-z plane (facing North/South)."""
    h = side / 2
    c = np.asarray(centre, float)
    corners = [[-h, 0, -h], [h, 0, -h], [h, 0, h], [-h, 0, h]] if vertical else \
        [[-h, -h, 0], [h, -h, 0], [h, h, 0], [-h, h, 0]]
    return np.vstack([corners, [[0, 0, 0]]]) + c


def min_distance(P: np.ndarray) -> float:
    """Smallest distance between any two drones over time. P: (N, T, 3)."""
    N = len(P)
    return min((float(np.linalg.norm(P[i] - P[j], axis=1).min()) for i in range(N) for j in range(i + 1, N)),
               default=float("inf"))


class Choreography:
    def __init__(self, home, rgb=(255, 255, 255)):
        self.home = np.asarray(home, float)
        self.N = len(self.home)
        self.t = [0.0]
        self.P = [self.home.copy()]                        # list of (N, 3) at RATE_HZ
        self.C = [np.tile(rgb, (self.N, 1)).astype(float)]
        self.steps: list[tuple[str, float, float]] = []    # (what, start, end)

    @property
    def now(self) -> float:
        return self.t[-1]

    @property
    def where(self) -> np.ndarray:
        return self.P[-1]

    def _append(self, poses, rgb, what):
        t0 = self.now
        dt = 1.0 / RATE_HZ
        for k, p in enumerate(poses, 1):
            self.t.append(t0 + k * dt)
            self.P.append(p)
            self.C.append(np.tile(rgb if rgb is not None else self.C[-1][0], (self.N, 1)).astype(float)
                          if rgb is None or np.ndim(rgb) == 1 else np.asarray(rgb, float))
        self.steps.append((what, t0, self.now))
        return self

    def hold(self, seconds: float, rgb=None):
        n = int(round(seconds * RATE_HZ))
        return self._append([self.where.copy()] * n, rgb, "hold")

    def move_to(self, target, seconds: float, assign: bool = False, rgb=None, what="move", stagger_s: float = 0.0):
        """Move every drone to its place in `target` (straight lines, minimum jerk). assign: choose
        which drone goes where. stagger_s: each drone may start up to this much later (and so
        moves faster) to keep drones apart; the step still takes `seconds`."""
        target = np.asarray(target, float)
        start = self.where.copy()
        order, delays = list(range(self.N)), np.zeros(self.N)
        if assign or stagger_s > 0:
            order, delays = self.plan_move(start, target, seconds, stagger_s, assign)
        target = target[order]
        n = int(round(seconds * RATE_HZ))
        tt = np.arange(1, n + 1) / RATE_HZ
        return self._append([start + (target - start) * self._progress(ti, delays, seconds)[:, None] for ti in tt],
                            rgb, what)

    @staticmethod
    def _progress(t, delays, seconds):
        """Each drone's 0..1 progress at time t of a move with per-drone start delays."""
        span = seconds - delays.max()
        return min_jerk((t - delays) / span)

    @classmethod
    def plan_move(cls, start, target, seconds: float = 1.0, stagger_s: float = 0.0, assign: bool = True):
        """(order of target places, per-drone start delays) for a straight-line move: the largest
        minimum separation during the move, then the shortest longest path."""
        N = len(start)
        tt = np.linspace(0, seconds, 61)
        perms = (itertools.permutations(range(N)) if assign and N <= 8 else [tuple(range(N))])
        levels = np.linspace(0, stagger_s, 3) if stagger_s > 0 else np.zeros(1)
        delay_sets = [np.array(d) for d in itertools.product(levels, repeat=N)] if stagger_s > 0 else [np.zeros(N)]
        def score(perm, d):
            tgt = target[list(perm)]
            prog = np.stack([cls._progress(t, d, seconds) for t in tt], 1)                # (N, T)
            P = start[:, None] + (tgt - start)[:, None] * prog[..., None]
            return (round(min_distance(P), 3), -float(np.linalg.norm(tgt - start, axis=1).max()), -float(d.sum()))

        if len(delay_sets) > 1:                    # stagger only the most promising assignments
            perms = sorted(perms, key=lambda q: score(q, np.zeros(N)), reverse=True)[:6]
        best, key = None, None
        for perm in perms:
            tgt = target[list(perm)]
            longest = -float(np.linalg.norm(tgt - start, axis=1).max())
            for d in delay_sets:
                prog = np.stack([cls._progress(t, d, seconds) for t in tt], 1)            # (N, T)
                P = start[:, None] + (tgt - start)[:, None] * prog[..., None]
                k = (round(min_distance(P), 3), longest, -float(d.sum()))
                if key is None or k > key:
                    best, key = (list(perm), d), k
        return best

    @classmethod
    def plan_chain(cls, home, formations, seconds, stagger_s: float = 0.0, beam: int = 8):
        """Which drone takes which place in each formation of a sequence that starts and ends on
        `home` (every drone back on its own slot): the largest minimum separation over every
        move of the chain. Returns one order per formation (target[order] = places by drone).

        A move's separation depends on the places before and after it, so choosing each
        formation on its own can leave a later move crowded. A beam of the best `beam` partial
        chains is kept, and the finalists are scored with staggered starts."""
        home = np.asarray(home, float)
        forms = [np.asarray(f, float) for f in formations]
        secs = list(seconds) if np.iterable(seconds) else [seconds] * (len(forms) + 1)
        N = len(home)

        chains = [((), home, np.inf)]                                       # (orders, last places, worst)
        for k, f in enumerate(forms):
            grown = []
            for orders, last, worst in chains:
                for perm in itertools.permutations(range(N)):
                    tgt = f[list(perm)]
                    grown.append((orders + (list(perm),), tgt, min(worst, cls._move_min(last, tgt, secs[k], 0.0))))
            grown.sort(key=lambda c: -c[2])
            chains = grown[:beam]
        best, key = None, None
        for orders, last, _ in chains:                                      # finalists: with staggering
            places = [home] + [f[o] for f, o in zip(forms, orders)] + [home]
            worst = min(cls._move_min(a, b, secs[i], stagger_s) for i, (a, b) in enumerate(zip(places, places[1:])))
            if key is None or worst > key:
                best, key = list(orders), worst
        return best

    @classmethod
    def _move_min(cls, a, b, seconds, stagger_s):
        """Largest minimum separation a straight move a -> b (fixed order) can have."""
        _, d = cls.plan_move(a, b, seconds, stagger_s, assign=False)
        tt = np.linspace(0, seconds, 61)
        prog = np.stack([cls._progress(t, d, seconds) for t in tt], 1)
        return min_distance(a[:, None] + (b - a)[:, None] * prog[..., None])

    @classmethod
    def best_assignment(cls, start, target, seconds: float = 1.0):
        return cls.plan_move(start, target, seconds)[0]

    def rotate(self, degrees: float, seconds: float, centre=(0.0, 0.0, 0.0), rgb=None):
        """Turn the formation about the vertical through `centre`, eased in and out."""
        c = np.asarray(centre, float)
        start = self.where.copy() - c
        n = int(round(seconds * RATE_HZ))
        poses = []
        for si in min_jerk(np.arange(1, n + 1) / n):
            a = math.radians(degrees) * si
            R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
            poses.append(start @ R.T + c)
        return self._append(poses, rgb, f"rotate {degrees:g}°")

    # ------------------------------------------------------------- output
    def arrays(self):
        """t (T,), positions (N, T, 3), colours (N, T, 3)."""
        return np.array(self.t), np.stack(self.P, 1), np.stack(self.C, 1)

    def report(self) -> dict:
        t, P, _ = self.arrays()
        V = np.gradient(P, t, axis=1)
        A = np.gradient(V, t, axis=1)
        return {"duration_s": float(t[-1]), "max_speed_mps": float(np.linalg.norm(V, axis=2).max()),
                "max_climb_mps": float(V[..., 2].max()), "max_descent_mps": float(-V[..., 2].min()),
                "max_accel_mps2": float(np.linalg.norm(A, axis=2).max()),
                "min_separation_m": min_distance(P[:, int(RATE_HZ * 0.5):-int(RATE_HZ * 0.5)]),
                "steps": self.steps}

    def write_csv_zip(self, path, names=None):
        """Skybrush-style CSV ZIP: one CSV per drone (time in ms, x, y, z in m, RGB)."""
        t, P, C = self.arrays()
        names = names or [f"Drone {i + 1}" for i in range(self.N)]
        with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
            for i, name in enumerate(names):
                buf = io.StringIO()
                buf.write("Time [msec],x [m],y [m],z [m],Red,Green,Blue\n")
                for k in range(len(t)):
                    x, y, zz = P[i, k]
                    r, g, b = (int(round(v)) for v in C[i, k])
                    buf.write(f"{int(round(t[k] * 1000))},{x:.4f},{y:.4f},{zz:.4f},{r},{g},{b}\n")
                z.writestr(f"{name.replace(' ', '_')}.csv", buf.getvalue())
        return path
