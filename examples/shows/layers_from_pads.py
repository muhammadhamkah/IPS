"""Up to 4 drones per charging pad: a layered grid that waves, turns once, closes up, turns back.

    python examples/shows/layers_from_pads.py out/show.zip [N] [--site venue.yaml]
    idps-show check out/show.zip --site venue.yaml --pads venue.yaml

The venue (default examples/sites/show_8x8.yaml) gives the performance area and the charging
pads (placed in the Planner, or a pads section by hand). Show frame: x East, y North, z up
(metres), origin = the venue origin. The drones take off from their pads in 4 batches, batch k =
slot k of every pad, and the show starts and ends in the air in the layered grid: batch 1 (the first up) at
the highest layer and the last batch the lowest, its drones in a row. The rows stand across the line from the pads to the middle of the
area, batch 1 farthest from the pads and batch 4 the nearest, so no batch flies over another on
its way in or out; in a row the drones keep their pads' order, so their paths do not cross. The rows
run along the pads' row (across the way the pads face), so a batch closes in from its 1.2 m
spacing in the grid to the pads' spacing on its way down without coming any closer.

Downwash by construction: each batch keeps to its own strip (ROW apart) for the whole show: it
waves up and down and closes up within its strip, and the formation's turns are rigid (they keep
every sideways distance). A formation too wide to turn inside the area holds still instead.
"""
import argparse
from pathlib import Path

import numpy as np
import yaml

from idps.show.choreo import Choreography

WHITE, BLUE, CYAN, RED, AMBER = (255, 255, 255), (30, 80, 255), (0, 200, 255), (255, 30, 40), (255, 160, 0)
SLOTS = 4                           # drones per pad = batches
COL = 1.2                           # drones along a row
CLOSE = 0.95                        # ... closed up
ROW = 1.1                           # rows (batches) apart: each batch's strip (far enough apart that
                                    # each batch lands after the one before is down, at a 5 s gap)
WAVE = 0.3                          # up and down within the layer (m)
WALL_MARGIN = 0.5                   # the formation keeps this far inside the area while turning
GRID_AHEAD_M = 1.0                  # the grid's nearest row at least this far in front of the pads
ROOT = Path(__file__).resolve().parents[2]


def batch(did: int) -> int:
    return (did - 1) % SLOTS + 1


class Layout:
    """Where each drone holds in the grid, from the venue's pads and performance area."""

    def __init__(self, site: dict, n: int):
        from idps.core.frames import Frame
        from idps.show.pads import pads_from_site
        lay = pads_from_site(site)
        if lay is None:
            raise SystemExit("the venue has no charging pads: place them in the Planner (Charging pad tool, C)")
        if n > SLOTS * len(lay.pads_ne):
            raise SystemExit(f"{n} drones need {-(-n // SLOTS)} charging pads; the venue has {len(lay.pads_ne)}")
        self.lay, self.n = lay, n
        frame = Frame.from_dict(site.get("origin"))
        poly = _polygon(site["volume"])
        self.area_ne = frame.ne(poly)                                     # (V, 2)
        self.centre = self.area_ne.mean(0)
        a = np.radians(lay.facing_deg)
        self.u = np.array([np.cos(a), np.sin(a)])                         # the pads' front: away from them
        self.v = np.array([-self.u[1], self.u[0]])                       # along the rows
        self.layers = lay.layers_m
        # the whole grid in front of the pads: no batch on its way down flies over another one
        # still holding in the grid (pads under the middle of the area)
        nearest = self.centre @ self.u - (SLOTS - 1) / 2 * ROW
        ahead = lay.pads_ne @ self.u
        need = float(ahead.max()) + GRID_AHEAD_M
        if nearest < need:
            self.centre = self.centre + self.u * (need - nearest)

    def grid(self, col: float = COL, wave=None) -> np.ndarray:
        """(n, 3) show-frame places (x East, y North, z up); wave: +1/-1 lifts alternate drones."""
        P = np.zeros((self.n, 3))
        for b in range(1, SLOTS + 1):
            members = [d for d in range(1, self.n + 1) if batch(d) == b]
            members.sort(key=lambda d: self.lay.pads_ne[(d - 1) // SLOTS] @ self.v)   # keep the pads' order
            for i, did in enumerate(members):
                ne = self.centre + self.u * ((SLOTS + 1) / 2 - b) * ROW + self.v * (i - (len(members) - 1) / 2) * col
                dz = 0.0 if wave is None else WAVE * wave * (1 if (i + b) % 2 == 0 else -1)
                P[did - 1] = [ne[1], ne[0], self.layers[b - 1] + dz]
        return P

    def can_turn(self) -> bool:
        """Does the formation, turned any way about the area's middle, stay inside the area?"""
        r = np.linalg.norm(self.grid()[:, :2] - [self.centre[1], self.centre[0]], axis=1).max()
        a, b = self.area_ne, np.roll(self.area_ne, -1, 0)
        d = b - a
        t = np.clip(((self.centre - a) * d).sum(1) / np.maximum((d * d).sum(1), 1e-12), 0, 1)
        wall = np.linalg.norm(a + d * t[:, None] - self.centre, axis=1).min()
        return r + WALL_MARGIN <= wall


def _polygon(vol: dict) -> np.ndarray:
    if "polygon" in vol:
        return np.array(vol["polygon"], float)
    (x0, y0, _), (x1, y1, _) = vol["min"], vol["max"]
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)


def build(site: dict, n: int = 20) -> Choreography:
    L = Layout(site, n)
    g = L.grid()
    centre = (L.centre[1], L.centre[0], 0.0)
    ch = Choreography(g, WHITE)
    ch.hold(2.0)
    ch.move_to(L.grid(wave=+1), 3.0, rgb=BLUE, what="wave up")
    ch.move_to(L.grid(wave=-1), 3.0, rgb=CYAN, what="wave down")
    ch.move_to(L.grid(wave=+1), 3.0, rgb=BLUE, what="wave up")
    ch.move_to(g, 3.0, rgb=WHITE, what="level")
    ch.hold(1.0)
    turn = L.can_turn()
    if turn:
        ch.rotate(360, 30.0, centre, rgb=AMBER)
    else:
        ch.hold(30.0, rgb=AMBER)
    ch.hold(1.5, rgb=WHITE)
    ch.move_to(L.grid(CLOSE), 4.0, rgb=RED, what="close up")
    ch.hold(1.0)
    if turn:
        ch.rotate(-360, 26.0, centre, rgb=RED)
    else:
        ch.hold(26.0, rgb=RED)
    ch.hold(1.0)
    ch.move_to(g, 4.0, rgb=WHITE, what="open out to the grid")
    ch.hold(2.0)
    ch.turns = turn
    return ch


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out", nargs="?", default="layers_from_pads.zip")
    ap.add_argument("n", nargs="?", type=int, default=20)
    ap.add_argument("--site", default=str(ROOT / "examples/sites/show_8x8.yaml"),
                    help="venue file with charging pads (default: the 8 x 8 m show floor)")
    a = ap.parse_args()
    c = build(yaml.safe_load(Path(a.site).read_text()), a.n)
    rep = c.report()
    for k, v in rep.items():
        if k != "steps":
            print(f"{k}: {v:.2f}")
    for what, t0, t1 in rep["steps"]:
        print(f"  {t0:5.1f}–{t1:5.1f} s  {what}")
    if not c.turns:
        print("the formation is too wide to turn inside the area: it holds still instead")
    print("wrote", c.write_csv_zip(a.out))
