# Designing and flying a show

```
Blender + Skybrush Studio ──.skyc / CSV ZIP──▶ idps-show check ──▶ idps-show export ──▶ drone_NNN.trj + manifest.json
  (design inside the IDPS venue)               (venue, cameras,     (one file per              │ idps-show upload
                                                limits, spacing)     drone ID, NED)             ▼ (Wi-Fi, MAVLink FTP)
                                                                              SD card of drone N: idps_show.trj
idps-show start (PC) ── one start time + clock sync ──▶ idps_show module (PX4) flies its own file
IDPS engine (PC)     ── position + heading 0 ─────────▶ PX4 EKF2
```

Decided: shows are designed in **Skybrush Studio for Blender**, and the drones run **PX4**.
Skybrush's own upload and flight tools target its ArduPilot-based firmware, so IDPS does the
PX4 side:
* checking the show against the venue;
* the per-drone files;
* uploading them;
* starting every drone at one instant.

**Each drone flies its own file from its SD card (D22).** This is the same workflow as with
commercial show drones: upload a file to each drone, then press start. A small PX4 module
(`px4/`, `idps_show`) plays it. The PC sends no flight path during the show, only:
* the IDPS positions;
* a few clock-sync messages a second;
* start and abort.

The drones carry **no companion computer** (D20).

## 1. Design in Blender

1. Install Skybrush Studio for Blender (free add-on). Set the show type to **Indoor**.
2. **Build the venue in Blender:**
   ```
   idps-show blender-venue --site venue.yaml --plan plan.yaml -o idps_venue.py
   ```
   In Blender, open the Scripting tab, open `idps_venue.py` and run it. It creates an
   **IDPS venue** collection:
   * the **performance area** (blue wireframe);
   * the **proven coverage** box (green), where the cameras are proven to meet the accuracy;
   * the **origin** with **N** and **E** arrows;
   * the trusses, and every camera with its view direction.

   Design the show inside the green box.
3. **Axes.** Blender **+X = East, +Y = North, +Z = up**, and Blender's origin = the venue origin
   (the calibration square). In Blender's top view with North up the screen, the show looks
   exactly as it will fly. A show designed elsewhere can be turned (`--rotate`) or moved
   (`--offset N,E,D`).
   In NED, North is +X; in Blender it is +Y, so that the top view reads like a map. If you
   would rather have Blender's +X point North too, design with `--axes NWU` (+X North, +Y West,
   +Z up) and pass the same option to every command.
4. **Export** from Skybrush Studio's Export panel:
   * **.skyc**, the main format: trajectories and lights;
   * **CSV (ZIP)**: one CSV per drone (time in ms, x, y, z, RGB). Use this if the .skyc export
     is unavailable, or for the LED colours (IDPS reads colours from the CSV; .skyc light
     programmes are kept as-is).
   * **XTJZ per-drone files** (what some show conversion software uploads to each drone, e.g.
     `1.dat`, `2.dat.gz`): a folder or ZIP of them. Positions are in cm, North-East-Down from
     the show origin, 10 per second; colours 18 per position (180 per second). Plain and compressed (signed)
     files are both read. The file has no take-off or landing: it starts and ends in the air.

### Or design a show in code (`idps.show.choreo`)

A show built from formations, moves and rotations, without Blender:

```
python examples/shows/five_drones_sphere_square.py out/five.zip     # writes a Skybrush-style CSV ZIP
idps-show check out/five.zip --site venue.yaml --plan plan.yaml
```

* **Moves are minimum-jerk:** they start and stop with zero speed and acceleration.
* **Rotations** turn a formation about the vertical, eased in and out. The drones keep facing
  North: they move, they don't yaw.
* **`move_to(..., assign=True)`** chooses which drone goes to which place, keeping drones as far
  apart as possible during the move.
* **`stagger_s`** lets drones start up to that much later than each other, to keep them apart.
* **`Choreography.plan_chain`** chooses the places for a whole sequence (grid → sphere → square →
  grid) together. Every drone then ends on its own take-off spot, which the next show's
  pre-flight check requires.

The example: 5 drones, 63 s, a 1 m grid → sphere → 360° → square → 360° → back to the grid.
* Up to 1.2 m/s and 1.3 m/s², with the closest designed approach 0.93 m.
* In the simulator with 60 g quads, flown on camera positions only:
  * the engine was within p95 3 mm in flight;
  * the drones were 6–7 cm from the path (median), p95 12–15 cm, worst 23 cm;
  * every drone landed 4–11 cm from its own take-off spot;
  * the closest two drones actually came was 0.66 m.

  Design for at least the airframe's tracking error on top of the spacing you need.

## 2. Check

```
idps-show check show.skyc --site venue.yaml --plan plan.yaml [--max-speed 3 --max-accel 4 --min-sep 1]
```

| Check | What it means |
|---|---|
| **Outside** | A drone leaves the performance area, or climbs above the ceiling, which would put it higher than the cameras (D17) |
| **Margin** (warning) | Within the planner's inset from a wall or the ceiling, where coverage is not proven |
| **Coverage** | Along every path, using the planner's accuracy model: at least the required number of cameras and p95 within the limit |
| **Speed / acceleration** | Within the airframe's limits (set them for your drones) |
| **Separation** | Every pair of drones stays at least `--min-sep` apart, at all times |
| **Home spacing** | Take-off and landing spots are far enough apart |

The report gives the time, the NED position and the drones involved for each problem. The
included demo show (`idps-show demo`) passes. Shifted 2 m North, it fails with *outside* and
*coverage* errors, which is the kind of mistake this step exists to catch.

## 3. Export one file per drone ID

```
idps-show export show.skyc --site venue.yaml --plan plan.yaml -o out/show
```

The export **runs the checks first and refuses if there are errors**. `--force` overrides
this, for testing only. It writes:

* `drone_NNN.trj`, the file that goes on drone N's SD card: a compact binary of the same
  samples with a checksum (`idps/show/trj.py`; about 470 kB for 10 minutes);
* `drone_NNN.json` (`idps-traj/1`), the same in readable form, one per show drone, named by
  its drone ID, containing:
  * the trajectory sampled at 20 Hz in the venue's NED frame;
  * velocity and acceleration (feed-forward for PX4);
  * LED colour;
  * yaw fixed at 0 = North (D7);
  * the home spot.
* `manifest.json`: the drone IDs, their home spots, the show length, the placement and the check
  result, with a checksum per file.

**Drone IDs (D21).** Every physical drone has a fixed ID from 1 to 255, set once:
* PX4 `MAV_SYS_ID` = the ID, so the drone answers to it over MAVLink;
* its IR marker blinks the same ID;
* a label on the drone shows it.

The show's **Drone 7** (the number in the Blender/Skybrush name) is flown by the drone with
**ID 7**, from `drone_007.trj`. That drone is put on Drone 7's home spot. Names without
numbers are numbered 1..N in order. Two drones with the same number stop the export.

To swap in a spare drone, give the spare the missing drone's ID (set `MAV_SYS_ID` and its
marker), or rename the drone in the show.

### Charging pads: take-off in batches, the show starts in the air

The drones stand on charging pads, 4 per pad in a 2 × 2 square (0.2 m apart). Drone ID =
4 × (pad − 1) + slot, and the slot is the drone's **take-off batch**: batch 1 is slot 1 of every
pad, and so on. Place the pads in the Planner (Charging pad tool, C, after the trusses and before
planning the cameras: docs/WORKFLOW.md); the venue file then carries them and `--pads venue.yaml`
reads them from it. Or describe them in a file of their own (`examples/pads/show_8x8.yaml`):

```yaml
pads: [[-3.2, -2.0], [-3.2, -1.0], [-3.2, 0.0]]   # pad centres, venue N and E (m), anywhere
slot_spacing_m: 0.2
layers_m: [1, 2, 3, 4]       # batch k's height in the initial and final grid
batch_gap_s: 5
hold_timeout_s: 60           # no start this long after the last batch took off: land
climb_speed_mps: 1.5         # up from the pad and down onto it (peak speed)
transfer_speed_mps: 1.5      # level, between the pad and the grid
```

The show you design starts and ends in the air: its **first frame is the initial grid** and its
**last frame the final grid**, with batch k at `layers_m[k]` in both. Check and export with
`--pads pads.yaml`:

0. **Tracked on the pads.** No drone takes off before the cameras see every drone on its own
   slot: `idps-show start --positions positions.csv` (the engine's live log) waits for that (up to
   `--track-wait`, 60 s) and names the drones not seen or on the wrong slot; each drone also refuses
   the launch without an IDPS position from the last 0.3 s or more than 0.1 m from its slot.
1. **Launch** (`idps-show start`): batch k takes off `batch_gap_s` × (k − 1) after the launch
   time. It climbs straight up over its pad slot to its layer, steps 0.5 m forward (the pad's
   front: over the slots already gone), then flies level to its place in the initial grid, and
   holds there.
2. **Show**: once every drone holds, the PC sends the start, and the show plays from the grid.
3. **Back to the pads**: after the show (from the final grid), or if no start comes within
   `hold_timeout_s` after the last batch took off, batch k leaves `batch_gap_s` × (n − k)
   later: last up, first down, so every drone flies as long. It flies level to 0.5 m in front of
   its pad slot, steps back over it, comes straight down onto it (arriving at zero speed), and PX4
   lands and disarms it.

**Pads can stand side by side, 0.5 m apart (centre to centre) or more.** A batch flies its long
legs as one formation (every drone takes as long as the batch's longest leg, written into each
drone's file), so drones from neighbouring pads close in from their 1.2 m in the grid to the pads'
spacing without coming any closer. On the way back each batch waits in front of its slots until
the batch beside it is down. A drone whose path to the grid would cross another pad first steps
forward past it.

**A pad's front faces where its drones fly** (`facing`: `auto`, the default: across the pads'
row towards the performance area when they stand in a row, else towards the area's middle; or
north / east / south / west). Slots 1 and 2 are at the front, so the
first batches leave first and the later ones step forward over empty slots: never over a drone
still on the pad, wherever the pads stand. Drones still face North (yaw 0); only the slots turn.

**The cameras must tell the drones on a pad apart.** Four drones 0.2 m apart, seen from far away
or edge-on, merge or get mixed up. Planning reports, per pad, the cameras that see its 4 drones at
least 8 px apart in the image (`pad_coverage`); a pad with fewer than the required cameras is
drawn red and named in the plan log (simulated: 2 such cameras failed to identify them, 3
worked). Pads tucked under the cameras on the near side are the usual cause: move them 1.5 m or
more in from the edge, or add a mount that looks down on them.

Every leg is a minimum-jerk move: T = max(1.5 s, 1.875 × distance / speed). The drone (the PX4
module) computes the legs itself from its file's header (batch, gap, speeds, timeout), so all
drones keep to one timeline on the PC's clock. The checks fly the same legs and report:
* a drone whose first or last frame is not at its batch's layer;
* drones closer than 0.5 m while taking off or landing;
* **downwash**: a flying drone within 0.4 m sideways above another (up to 3 m below it), in the
  take-off, both landings, and the show itself.

**The batch gap must cover the pad.** With slots 0.2 m apart, a batch must have climbed and moved
aside before the next batch climbs beside it, and on the way back a batch must be down (touchdown
counted as 0.5 s) before the next one arrives over the pad. With the example show, a 5 s gap
needs 1.5 m/s climb and transfer; at 1.0 m/s it takes 7.5 s, at 0.8 m/s 8.5–9 s. When the gap
is too short, the check says so and gives the smallest safe one.

Design the grid in front of the pads (where they face), with its rows along the pads' row: batch
1 holds the row farthest from the pads and the last batch the nearest, so no batch flies over
another on its way in or out. In the example the rows are 1.1 m apart.

`examples/shows/layers_from_pads.py --site venue.yaml` is such a show for any venue with pads (4
drones per pad, 80 s): its grid stands in front of the pads (at least 1 m) towards the middle of
the area, each batch keeps to its own strip for the whole show (it waves and closes up within it,
and the turns are rigid), so it is downwash-free by construction; a formation too wide to turn
inside the area holds still instead. Checked at a 5 s gap with 1.5 m/s climb and transfer: pads
in a row 0.5 m apart along an edge, in a corner and under the middle of a 20 × 14 m area, rows at
0.6, 0.75 and 1.0 m, and the 8 × 8 m example with 1–20 drones.

## 4. Upload, start, fly

**On each drone** (D20), there is no companion computer, to save weight:
* a PX4 flight controller with Wi-Fi: either built in, or a few-gram Wi-Fi-to-serial MAVLink
  bridge such as an ESP32 or ESP8266 module (the bridge only relays, it runs nothing);
* PX4 built with the `idps_show` module (`px4/README.md`), started from `etc/extras.txt`;
* **one** IDPS IR marker, at the drone's reference point (top centre, over the centre of
  mass). Declare any remaining offset to PX4 with `EKF2_EV_POS_X/Y/Z`;
* the show LEDs.

**Heading.** Drones never yaw. They always face **North, +X in NED** (D7). Because of that:
* Every drone is put down **facing North** on its home pad (mark each pad with an arrow).
* The magnetometer stays **off**, since it is unreliable indoors.
* The PC sends **heading 0** with every position, and PX4 fuses it as external-vision yaw.
  PX4 needs some heading source, and this is it. See R20 for how far this can be trusted and
  how IDPS will watch for a drone that is turned.

**Upload.** `--link DRONE_ID=address` gives each drone's network address. Every command
first checks that the drone at that address answers with that `MAV_SYS_ID`, so a mixed-up
address can't put Drone 3's file on drone 7.
```
idps-show upload out/show/manifest.json --link 1=udpout:192.168.50.101:14550 --link 7=udpout:192.168.50.107:14550 ...
idps-show status out/show/manifest.json --link ...    # each drone: ready / file matches
```
Upload copies each drone's `.trj` to `/fs/microsd/idps_show.trj` over MAVLink FTP. The drone
then loads it, checks it, and reports its CRC, which must equal the PC's. The file stays on
the card, so a drone that reboots is still ready.

**Start.**
```
idps-show start out/show/manifest.json --link ... --lead 15
```
1. The PC starts the **clock sync**: its time goes to every drone 5 times a second, for the
   whole show.
2. After 5 s it sends **one start time**, in the PC's clock, to every drone.
3. Each drone accepts only if:
   * its file matches the PC's copy;
   * it is landed and disarmed;
   * IDPS positions are arriving;
   * it is within 0.5 m of its own home spot;
   * its clock is synced.
4. If any drone refuses, **every** drone is told to abort, so nobody flies a partial show.
   The refusing drone's reason is printed.
5. **Countdown:** at −3 s each drone switches to OFFBOARD, at −2 s it arms, and at 0 it flies
   its file. At the end it lands and disarms.
6. Keep `start` running until the show ends, because it keeps the clocks. **Ctrl+C (or
   SIGTERM) aborts:** every drone lands where it is. `idps-show abort --link ...` does the
   same from another terminal.

**Pre-flight check: every drone on its own spot.** Before the start, the cameras read every
marker ID on the ground, and `Fleet.preflight` blocks the start on any of these:
* *Drone 4: not seen*;
* *Drone 1: on Drone 2's spot*;
* *Drone 3: 1.00 m from its spot*;
* *Marker 9: a drone that is not in this show*;
* a drone with no link.

Each drone also checks its own spot itself, as above. The camera-side check needs the
engine's positions, so it runs once the engine is built.

**Tested in the PX4 simulator** with 3 drones, using real PX4 v1.16 with the module and a
stand-in for the IDPS positions. All of these passed:
* upload;
* start together;
* flying (the drones agreed on the clock to 2–10 ms);
* landing;
* abort mid-show;
* refusing to start when two drones were swapped.

**Closed loop:** the same show then flew on positions from simulated cameras and the real
engine, with no stand-in. The engine named the drones on their spots, their start checks
passed, and they flew and landed. In flight the engine was within p95 3 mm of the true
marker.

Details are in `px4/README.md`.

`idps-show play drone_007.json --dry-run` prints one drone's path, for testing.

**PX4 parameters:**
* external vision: `EKF2_EV_CTRL` = horizontal position + vertical position + yaw;
  `EKF2_HGT_REF` = vision;
* `EKF2_EV_DELAY` = 0 and `EKF2_DELAY_MAX` = 50 ms with `idps-engine live --timesync`
  (positions carry their exposure time); without it, `EKF2_EV_DELAY` = the measured latency
  from the camera exposure to PX4 (positions travel over Wi-Fi, so measure it on the real link);
* `EKF2_MAG_TYPE` = None. PX4's local frame then *is* the venue's NED frame.

**Failsafes:**
* loss of positions, e.g. Wi-Fi drops → land (EKF2 vision timeouts). The flight path itself
  is on the drone, so a short drop doesn't stall the show;
* the module stops publishing → land (`COM_OF_LOSS_T`, `COM_OBL_RC_ACT`);
* a geofence matching the performance area.

**Wi-Fi:** use a dedicated 5 GHz network for the drones, separate from the camera network
(cameras are wired, D16). During a show each drone receives about 50 position messages a
second (about 300 bytes each on air) plus 5 small clock-sync messages: around 50 packets and
16 kB a second per drone. That is half the packets of streaming the path from the PC, and
packets are what limit Wi-Fi. Latency and dropouts matter more than bandwidth; measure them
in Phase 7.

## Open questions

* **Drone flight controller with Wi-Fi.** Choose the board (PX4 support, weight, Wi-Fi
  range and latency), or a standard PX4 board plus a Wi-Fi MAVLink bridge.
* **Still to build:**
  * the engine's position stream (`Fleet.send_positions`), and the pre-flight check on live
    marker IDs;
  * driving the show LEDs from the module (depends on the LED hardware);
  * the heading monitor (R20);
  * a dashboard **Show** tab: 3D playback, check results, the pre-flight check per drone ID,
    start and abort;
  * rehearsing a show in the simulator with the live-monitoring brain watching.
* **Next simulator tests, before the first flight:**
  * the failsafes (Wi-Fi loss mid-show);
  * a drone put down turned 5–30° from North (R20);
  * a larger swarm on a real Wi-Fi network (packet loss and latency).
