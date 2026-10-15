"""20 drones: a 5 x 4 grid, two rings that turn once, a vertical wall that turns once, back to the grid.

    python examples/shows/twenty_drones_rings_wall.py out/twenty.zip
    idps-show check out/twenty.zip --site SITE --plan PLAN

Show frame: x East, y North, z up (metres), origin = the venue origin (examples/sites/show_8x8.yaml:
8 x 8 m, up to 5.5 m). Who goes where is chosen so that no two paths come close, by construction:
the grid's 14 outer drones keep their bearing into the big ring and its 6 inner ones into the small
ring (higher up); for the wall, each grid row first climbs to its own height and then slides to the
middle. Every drone lands on its own take-off spot. Up to 1.2 m/s.
"""
import math
import sys

import numpy as np

from idps.show.choreo import Choreography

WHITE, BLUE, CYAN, RED, AMBER = (255, 255, 255), (30, 80, 255), (0, 200, 255), (255, 30, 40), (255, 160, 0)
COLS, ROWS, SPACING = 5, 4, 1.5             # take-off grid: 5 along East, 4 along North
LIFT = 1.5                                  # grid height after take-off
OUTER = dict(r=2.6, z=2.0)                  # big ring: the grid's 14 outer drones
INNER = dict(r=1.2, z=3.5)                  # small ring: its 6 inner drones
WALL_X, WALL_Z = 1.2, (1.3, 2.4, 3.5, 4.6)  # wall facing North: column spacing, row heights
CENTRE = (0.0, 0.0, 0.0)


def spots() -> np.ndarray:
    """Take-off spots, drone by drone (row by row from the South, West to East)."""
    x = (np.arange(COLS) - (COLS - 1) / 2) * SPACING
    y = (np.arange(ROWS) - (ROWS - 1) / 2) * SPACING
    return np.array([[xi, yi, 0.0] for yi in y for xi in x])


def ring_places(home: np.ndarray) -> np.ndarray:
    """Each drone's place in the two rings: outer grid drones into the big ring and inner ones into
    the small ring, in bearing order, so the paths fan out and never cross."""
    col = np.arange(len(home)) % COLS
    row = np.arange(len(home)) // COLS
    outer = (col == 0) | (col == COLS - 1) | (row == 0) | (row == ROWS - 1)
    places = np.zeros_like(home)
    for sel, ring in ((outer, OUTER), (~outer, INNER)):
        idx = np.flatnonzero(sel)
        bearing = np.arctan2(home[idx, 1], home[idx, 0])
        order = idx[np.argsort(bearing)]
        n = len(order)
        # turn the ring so its slots sit as close as possible to the drones' own bearings
        b = np.sort(bearing)
        offset = math.atan2(np.mean(np.sin(b - 2 * np.pi * np.arange(n) / n)),
                            np.mean(np.cos(b - 2 * np.pi * np.arange(n) / n)))
        a = offset + 2 * np.pi * np.arange(n) / n
        places[order] = np.column_stack([ring["r"] * np.cos(a), ring["r"] * np.sin(a), np.full(n, ring["z"])])
    return places


def build() -> Choreography:
    home = spots()
    grid = home + [0, 0, LIFT]
    rings = ring_places(home)
    row = np.arange(len(home)) // COLS
    col = np.arange(len(home)) % COLS
    risen = grid.copy()
    risen[:, 2] = np.array(WALL_Z)[row]                       # each row at its own height
    wall = np.column_stack([(col - (COLS - 1) / 2) * WALL_X, np.zeros(len(home)), risen[:, 2]])
    c = Choreography(home, WHITE)
    c.hold(1.0)
    c.move_to(grid, 5.0, what="take off to the grid")
    c.hold(2.0)
    c.move_to(rings, 8.0, rgb=BLUE, what="into two rings")
    c.hold(1.5)
    c.rotate(360, 28.0, CENTRE, rgb=CYAN)
    c.hold(2.0, rgb=BLUE)
    c.move_to(grid, 8.0, rgb=WHITE, what="back to the grid")
    c.hold(2.0)
    c.move_to(risen, 5.0, rgb=RED, what="rows climb to their heights")
    c.move_to(wall, 6.0, rgb=RED, what="slide into a wall")
    c.hold(1.5)
    c.rotate(360, 26.0, CENTRE, rgb=AMBER)
    c.hold(2.0, rgb=RED)
    c.move_to(risen, 6.0, rgb=WHITE, what="slide back")
    c.move_to(grid, 5.0, what="down to the grid")
    c.hold(2.0)
    c.move_to(home, 5.0, what="land")
    c.hold(1.0)
    return c


if __name__ == "__main__":
    c = build()
    rep = c.report()
    for k, v in rep.items():
        if k != "steps":
            print(f"{k}: {v:.2f}")
    for what, a, b in rep["steps"]:
        print(f"  {a:5.1f}–{b:5.1f} s  {what}")
    print("wrote", c.write_csv_zip(sys.argv[1] if len(sys.argv) > 1 else "twenty_drones_rings_wall.zip"))
