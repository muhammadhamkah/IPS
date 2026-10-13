"""5 drones: a 1 m grid, a sphere that turns once, a square that turns once, back to the grid.

    python examples/shows/five_drones_sphere_square.py out/five.zip
    idps-show check out/five.zip --site SITE --plan PLAN

Show frame: x East, y North, z up (metres), origin = the venue origin.
"""
import sys

from idps.show.choreo import Choreography, grid, sphere5, square5

WHITE, BLUE, CYAN, RED, AMBER = (255, 255, 255), (30, 80, 255), (0, 200, 255), (255, 30, 40), (255, 160, 0)
CENTRE = (0.0, 0.0, 2.5)


def build() -> Choreography:
    spots = grid(5, 1.0, 0.0)                         # take-off spots: a row along East, 1 m apart
    grid_1m = grid(5, 1.0, 1.0)
    sphere, square = sphere5(1.2, CENTRE), square5(2.0, CENTRE)
    # Who goes where in the sphere and the square, chosen for the whole chain grid -> sphere ->
    # square -> grid (both formations turn a full 360°, so they end where they started), with
    # every drone back on its own slot at the end. Found once (about 1.5 minutes) with
    #     Choreography.plan_chain(grid_1m, [sphere, square], 6.0, stagger_s=1.5, beam=24)
    # and kept here. Closest approach in the three moves: 1.00, 1.41 and 0.93 m.
    o_sphere, o_square = [3, 0, 2, 1, 4], [0, 3, 4, 1, 2]
    c = Choreography(spots, WHITE)
    c.hold(1.0)
    c.move_to(grid_1m, 4.0, what="take off to the 1 m grid")
    c.hold(2.0)
    c.move_to(sphere[o_sphere], 6.0, rgb=BLUE, what="into a sphere", stagger_s=1.5)
    c.hold(1.5)
    c.rotate(360, 12.0, CENTRE, rgb=CYAN)
    c.hold(2.0, rgb=BLUE)                             # stop
    c.move_to(square[o_square], 6.0, rgb=RED, what="into a square", stagger_s=1.5)
    c.hold(1.5)
    c.rotate(360, 12.0, CENTRE, rgb=AMBER)
    c.hold(2.0, rgb=RED)
    c.move_to(grid_1m, 6.0, rgb=WHITE, what="back to own slots in the 1 m grid", stagger_s=1.5)
    c.hold(2.0)
    c.move_to(spots, 4.0, what="land")                # each drone lands on its own take-off spot
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
    print("wrote", c.write_csv_zip(sys.argv[1] if len(sys.argv) > 1 else "five_drones_sphere_square.zip"))
