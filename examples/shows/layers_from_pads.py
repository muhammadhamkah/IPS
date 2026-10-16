"""Up to 20 drones from charging pads: a layered grid that waves, turns once, closes up, turns back.

    python examples/shows/layers_from_pads.py out/show.zip [N]
    idps-show check out/show.zip --site examples/sites/show_8x8.yaml --pads examples/pads/show_8x8.yaml

Show frame: x East, y North, z up (metres), origin = the venue origin. The drones take off from
their pads (examples/pads/show_8x8.yaml: 5 pads in a row along the South edge, 4 drones each) in
4 batches, batch k = slot k of every pad, and the show starts and ends in the air in the layered
grid: batch k at k metres, its drones in a row along East. Batch 1 holds the row farthest from the
pads and batch 4 the nearest, so no batch flies over another on its way in or out.

Downwash by construction: each batch keeps to its own strip along North (ROW apart) for the whole
show: it waves up and down and closes up within its strip, and the formation's turns are rigid
(they keep every sideways distance). So no drone is ever above another within 0.9 m sideways.
"""
import sys

import numpy as np

from idps.show.choreo import Choreography

WHITE, BLUE, CYAN, RED, AMBER = (255, 255, 255), (30, 80, 255), (0, 200, 255), (255, 30, 40), (255, 160, 0)
SLOTS = 4                           # drones per pad = batches
LAYERS = (1.0, 2.0, 3.0, 4.0)       # batch k flies at LAYERS[k-1] (as examples/pads/show_8x8.yaml)
COL = 1.2                           # drones along East in a row
CLOSE = 0.95                        # ... closed up
ROW = 0.9                           # rows (batches) along North: each batch's strip
WAVE = 0.3                          # up and down within the layer (m)
CENTRE = (0.0, 0.0, 0.0)


def batch(did: int) -> int:
    return (did - 1) % SLOTS + 1


def rows(n: int) -> dict:
    """batch -> its drone IDs, West to East (pads are in a row along East, pad 1 westmost)."""
    out = {}
    for did in range(1, n + 1):
        out.setdefault(batch(did), []).append(did)
    return out


def grid(n: int, col: float = COL, wave=None) -> np.ndarray:
    """The layered grid; wave: +1/-1 lifts alternate drones of each row by WAVE (starting +/-)."""
    P = np.zeros((n, 3))
    for b, members in rows(n).items():
        for i, did in enumerate(members):
            dz = 0.0 if wave is None else WAVE * wave * (1 if (i + b) % 2 == 0 else -1)
            P[did - 1] = [(i - (len(members) - 1) / 2) * col, ((SLOTS + 1) / 2 - b) * ROW, LAYERS[b - 1] + dz]
    return P


def build(n: int = 20) -> Choreography:
    g = grid(n)
    ch = Choreography(g, WHITE)
    ch.hold(2.0)
    ch.move_to(grid(n, wave=+1), 3.0, rgb=BLUE, what="wave up")
    ch.move_to(grid(n, wave=-1), 3.0, rgb=CYAN, what="wave down")
    ch.move_to(grid(n, wave=+1), 3.0, rgb=BLUE, what="wave up")
    ch.move_to(g, 3.0, rgb=WHITE, what="level")
    ch.hold(1.0)
    ch.rotate(360, 28.0, CENTRE, rgb=AMBER)
    ch.hold(1.5, rgb=WHITE)
    ch.move_to(grid(n, CLOSE), 4.0, rgb=RED, what="close up")
    ch.hold(1.0)
    ch.rotate(-360, 24.0, CENTRE, rgb=RED)
    ch.hold(1.0)
    ch.move_to(g, 4.0, rgb=WHITE, what="open out to the grid")
    ch.hold(2.0)
    return ch


if __name__ == "__main__":
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    c = build(n)
    rep = c.report()
    for k, v in rep.items():
        if k != "steps":
            print(f"{k}: {v:.2f}")
    for what, a, b in rep["steps"]:
        print(f"  {a:5.1f}–{b:5.1f} s  {what}")
    print("wrote", c.write_csv_zip(sys.argv[1] if len(sys.argv) > 1 else "layers_from_pads.zip"))
