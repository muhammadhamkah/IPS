# Calibration (`idps-calib`)

The engine needs two things about every camera:

- **Its lens:** focal length, image centre and distortion. Measured once per camera, from pictures
  of a checkerboard.
- **Where it stands and points.** Measured after the cameras are set up, and again whenever one is
  moved, by moving one LED through the room.

The result is one file, `calibration.yaml`, which `idps-engine live --calib calibration.yaml`
reads. This is the first, practical version of ARCHITECTURE.md section 6: steps 6.1 to 6.3,
with one LED in place of the three-LED wand and the calibration square.

`idps-calib` runs on the Mac. It needs two extra packages:

```sh
cd ~/IPS && source ~/Developer/IPS/.venv/bin/activate
pip install -e '.[calib]'
```

## 1. The lens (once per camera; works with one station)

1. Make the board and print it:
   ```sh
   idps-calib board -o board.svg
   ```
   Open `board.svg` in a browser and print it at **100 %** (no "fit to page") on A4, landscape.
   Measure one square with a ruler: it should be 25 mm. Glue or tape the sheet flat onto
   something stiff (a clipboard, a book cover).
2. Set the camera's focus and lock it. The lens must not be touched after this step.
3. Take 15–25 pictures on the Pi. The IR filter blocks visible light, so take them **without
   the filter**, in a well-lit room, with a longer exposure:
   ```sh
   for i in $(seq 1 25); do idps-station snapshot --exposure-us 8000 --out b$i.pgm; sleep 3; done
   ```
   Between pictures, move the board:
   - hold it at different angles (tilted left, right, up, down, up to about 45°);
   - put it in every part of the image, **including the four corners**;
   - keep it about 30–60 cm away, so it fills a quarter to a half of the image;
   - hold it still for each picture.
4. Copy the pictures to the Mac (`scp pi@<pi address>:b*.pgm .`) and run:
   ```sh
   idps-calib lens b*.pgm --station 1 -o lens_1.yaml
   ```
   If your printed squares aren't 25 mm, add `--square-mm 24.6` (what you measured).

It prints the focal length, the field of view, and how well the corners fit:

| corners fit to | meaning |
|---|---|
| ≤ 0.1 px | good (the target) |
| 0.1–0.2 px | usable |
| > 0.2 px | poor: check the focus is locked, the board is flat, and the pictures are sharp |

If it says the board never reached parts of the image, take more pictures with the board in the
corners.

## 2. The room (two or more stations)

One blinking LED is enough: the test marker (docs/MARKER.md) on a stick or in your hand.

1. **Mark 3 or 4 points on the floor and measure them** from the origin, in metres. They must
   not be in one line. For example: the origin, 1 m North of it, 1 m East of it. Write them in
   `points.yaml`, in the order you will visit them. `z` is the LED's height above the floor:
   ```yaml
   points:
     - {xyz: [0.0, 0.0, 0.02]}     # origin
     - {xyz: [1.0, 0.0, 0.02]}
     - {xyz: [0.0, 1.0, 0.02]}
   ```
   `xyz` is in the plan's drawing frame (metres, Z up). With `--site venue.yaml` you can write
   `{ned: [1.0, 0.0, -0.02]}` (North, East, Down) instead.
2. Start every station (`idps-station run --engine <the Mac's address>`), then on the Mac:
   ```sh
   idps-calib record -o room.idpsraw --seconds 90
   ```
3. While it records:
   - put the LED on each measured point and **leave it still for 3 seconds**, in order;
   - then move it **slowly** (about walking pace or less) through the whole space the cameras
     see: low, high, the middle, the edges.

   Every second the recorder prints which stations see the LED. It needs two or more at a time.
4. Solve:
   ```sh
   idps-calib room room.idpsraw --lens lens_1.yaml --lens lens_2.yaml --points points.yaml -o calibration.yaml
   ```

It prints each camera's position, how well its dots fit, and how close the rest points came to
where you measured them:

| dots fit to | meaning |
|---|---|
| ≤ 0.15 px | good |
| 0.15–0.4 px | usable |
| > 0.4 px | poor: a wrong lens file, a camera that moved during the recording, or bad clock sync |

Rest points more than about 5 mm off mean a mis-measured point or the wrong order.

### Options

| option | use |
|---|---|
| `--fov-deg 79` | A station with no lens file: start from the lens's nominal field of view and refine its focal length and distortion here. Good for a first try. Measured lenses are better. |
| `--refine-lens` | Also refine the measured lenses' focal length and distortion |
| `--plan plan.yaml` | No points file: the world frame comes from the cameras' planned positions. Only as good as the cameras were placed and aimed. |
| `--save-lenses DIR` | Write the refined lens files |
| `--max-points 2500` | LED positions used; fewer is faster |

## How it works

- **Matching.** Each station reports its dots frame by frame. The tool keeps the frames where a
  station sees exactly one dot, after removing that station's fixed reflections (a spot lit in
  more than half of its frames). The cameras don't expose at the same instant and the LED moves
  between their exposures, so each station's dot is interpolated to common instants between two
  of its own consecutive frames.
- **Rest points.** A stretch of 1.5 s or more where the LED stays within 0.6 px in every camera
  that sees it. Each becomes one averaged observation. They are matched to `points.yaml` in
  time order.
- **Start.** The two cameras sharing the most LED positions give a first relative pose
  (essential matrix, RANSAC). Their shared positions are triangulated, and every other camera
  is placed against those points.
- **Bundle adjustment.** All camera poses and LED positions are refined together, so every dot
  fits its LED position (sparse least squares, robust loss). Dots that still miss by several
  pixels are dropped and it runs again.
- **World frame.** The reconstruction has no scale, origin or direction of its own. A best-fit
  move, turn and scale puts the rest points on their measured coordinates.

## Tested (simulator)

`tests/test_calib.py`, in the simulated lab (9 cameras, realistic dot noise, fixed reflections,
false dots, unsynchronised exposures, a blinking LED, 50 s of movement):

| case | camera position | camera direction |
|---|---|---|
| measured lenses, 4 rest points | within 2 mm | within 0.2 mrad |
| lenses 3 % off and no distortion, refined | within 5 mm | within 0.5 mrad |
| 2 cameras, world frame from the plan | within 2 mm | within 0.3 mrad |

The lens tool recovers a known lens from rendered checkerboard pictures to 0.4 % in focal
length. None of this has run on a real camera yet.

## Not yet in this version

- the three-LED wand and the calibration square (one LED and rest points stand in for both);
- beacons, live re-pose and continuous calibration (ARCHITECTURE 6.3b–6.7);
- a quality report with per-camera uncertainty.
