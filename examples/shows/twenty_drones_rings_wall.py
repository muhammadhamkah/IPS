"""20 drones from 5 charging pads: a layered grid, a cone of rings that turns once, a sloping wall
that turns once, back to the grid.

    python examples/shows/twenty_drones_rings_wall.py out/twenty.zip
    idps-show check out/twenty.zip --site examples/sites/show_8x8.yaml --pads examples/pads/show_8x8.yaml

Show frame: x East, y North, z up (metres), origin = the venue origin. The drones take off from
their pads (examples/pads/show_8x8.yaml: a row along the South edge) in 4 batches, batch k = slot
k of every pad, and the show starts and ends in the air in the layered grid: batch k at k metres,
5 drones in a row along East, the rows spread along North. Batch 1 holds the row farthest from the
pads and batch 4 the nearest, so no batch flies over another on its way in or out.

Downwash: no drone is ever above another within 0.4 m sideways. The grid's rows are 0.75 m apart
along North, the cone's rings 0.5 m apart in radius, the wall's rows 0.45 m apart along North (it
slopes), and every move keeps each layer at its own height. Up to 1.2 m/s.
"""
import math
import sys

import numpy as np

from idps.show.choreo import Choreography

WHITE, BLUE, CYAN, RED, AMBER = (255, 255, 255), (30, 80, 255), (0, 200, 255), (255, 30, 40), (255, 160, 0)
PADS, BATCHES = 5, 4
LAYERS = (1.0, 2.0, 3.0, 4.0)        # batch k flies at LAYERS[k-1] (as examples/pads/show_8x8.yaml)
COL = 1.2                            # grid and wall: drones along East
GRID_ROW = 0.75                      # grid: rows along North
WALL_ROW = 0.45                      # wall: rows along North (sloping, so no drone is above another)
RADII = (0.9, 1.4, 1.9, 2.4)         # cone: batch 1's ring (lowest) to batch 4's (highest)
CENTRE = (0.0, 0.0, 0.0)


def ids(n: int = PADS * BATCHES) -> list[int]:
    return list(range(1, n + 1))


def batch_col(did: int) -> tuple[int, int]:
    """(batch 1..4, column 0..4) of drone ID did: 4 drones per pad, pads along East."""
    return (did - 1) % BATCHES + 1, (did - 1) // BATCHES


def grid(n: int, row: float = GRID_ROW) -> np.ndarray:
    P = []
    for did in ids(n):
        b, c = batch_col(did)
        P.append([(c - (PADS - 1) / 2) * COL, ((BATCHES + 1) / 2 - b) * row, LAYERS[b - 1]])
    return np.array(P, float)


def cone(n: int) -> np.ndarray:
    """Each batch's drones on a ring at its own height, in the same West-to-East order as its grid row
    (so the move is a fan-out within the layer)."""
    P = np.zeros((n, 3))
    for b in range(1, BATCHES + 1):
        members = [d for d in ids(n) if batch_col(d)[0] == b]
        k = len(members)
        # spread over the ring, the westmost drone at the West point, turning through South
        a = math.pi + 2 * math.pi * np.arange(k) / max(k, 1)
        for d, ang in zip(members, a):
            P[d - 1] = [RADII[b - 1] * math.cos(ang), RADII[b - 1] * math.sin(ang), LAYERS[b - 1]]
    return P


def build(n: int = PADS * BATCHES) -> Choreography:
    g, w, c = grid(n), grid(n, WALL_ROW), cone(n)
    ch = Choreography(g, WHITE)
    ch.hold(2.0)
    ch.move_to(c, 8.0, rgb=BLUE, what="into a cone of rings")
    ch.hold(1.5)
    ch.rotate(360, 28.0, CENTRE, rgb=CYAN)
    ch.hold(2.0, rgb=BLUE)
    ch.move_to(g, 8.0, rgb=WHITE, what="back to the grid")
    ch.hold(2.0)
    ch.move_to(w, 4.0, rgb=RED, what="close up into a sloping wall")
    ch.hold(1.5)
    ch.rotate(360, 28.0, CENTRE, rgb=AMBER)
    ch.hold(2.0, rgb=RED)
    ch.move_to(g, 4.0, rgb=WHITE, what="back to the grid")
    ch.hold(2.0)
    return ch


if __name__ == "__main__":
    c = build()
    rep = c.report()
    for k, v in rep.items():
        if k != "steps":
            print(f"{k}: {v:.2f}")
    for what, a, b in rep["steps"]:
        print(f"  {a:5.1f}–{b:5.1f} s  {what}")
    print("wrote", c.write_csv_zip(sys.argv[1] if len(sys.argv) > 1 else "twenty_drones_rings_wall.zip"))
