# Show designer (Designer tab)

Build a show from **scenes** and watch it fly in your venue. The show starts and ends in the air
in the venue's grid above the charging pads (the drones take off to it and land from it in their
4 batches, as for every show), so the venue needs its performance area, origin and charging
pads first (Planner tab).

## Scenes

A scene is a formation on a vertical wall facing the audience. **Add scene** offers:

| scene | what it is |
|---|---|
| Grid | rows and columns |
| Circle, Heart | drones evenly round the outline |
| Text… | the letters you type, drones spread evenly over them |
| Image… | the shape in a picture (PNG, JPG): what is not transparent, or what differs from the picture's border colour. The drones can take the picture's colours. |

For the selected scene:

| setting | meaning |
|---|---|
| Width | the formation's wider side. Left at "as small as the spacing allows", the closest drones are 1.25 × the minimum distance apart. |
| Place | in the middle of the area, or East and North of the origin |
| Height of its middle | automatic: the lowest drone 0.5 m above the lowest flight height |
| Faces | the compass direction from the scene to the audience. Automatic: the show's setting. |
| Depth | Flat; Tilt (the top leans to the audience: also moves drones out from under each other); Bulge; By the image's brightness |
| Holds for | how long the scene stays |
| Flight to it takes | automatic: from the distance and the show's speed |
| Colour | the LEDs' colour, or the image's colours |
| Wave | the drones rise and fall by the wave height, the wave travelling along the wall (length, period) |
| Turns while it holds | the wall turns about its middle by this angle |

A wave and a turn ease in and out, so the drones start and stop smoothly.

## Flights between scenes

Worked out for you, every time something changes:

1. Each drone takes the place in the next scene that makes the flights shortest. Such flights do
   not cross.
2. Drones that would still come closer than the minimum distance are kept apart: a path bows out
   towards or away from the audience (or up or down), or the drone leaves a little later.
3. A flight gets more time until no drone is rushed.

The last scene's places are chosen to suit the flight back to the grid as well, because every
drone must return to its own grid place (it lands on its own pad).

## Checks

Under the view, as you edit (✖ must be fixed, ⚠ is for you to know):

- two drones closer than the minimum distance (drawn with a red edge at that moment);
- drones outside the performance area, or below or above the flight heights;
- a drone faster than 3 m/s, or accelerating harder than 4 m/s²;
- ⚠ drones one above another. A wall stacks drones, so the lower ones fly in the upper ones'
  downwash; IDPS's own rule forbids it, the full loop flies it anyway. Tilt offsets them.

Click a line to see that moment.

## Using the show

- **Send to Simulate**: the planned cameras film it and the engine tracks it.
- **Send to PX4 flight**: simulated PX4 drones fly it, from the charging pads and back.
- **Export show file…**: a CSV ZIP (`idps-show check`, `export`, `upload` read it).
- **Save… / Open…**: the design itself (a small YAML file), to carry on later.

## In code

```python
from idps.show.designer import Design, Scene
d = Design(n=20, scenes=[Scene("grid"), Scene("text", text="HI"), Scene("circle", wave_m=0.3)])
c = d.compile(site_dict)            # c.findings, c.stats, c.P (N, T, 3), c.C
c.write_csv_zip("show.zip")
```

## Not yet

A timeline with music, editing single drones, more movements (ripple, breathe), colour
animations, images flat on the floor.
