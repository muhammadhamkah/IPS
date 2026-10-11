# Designing and flying a show

```
Blender + Skybrush Studio ──.skyc──▶ idps-show check ──▶ idps-show export ──▶ slot_NN.json ──▶ each drone's
  (design inside the                 (venue, cameras,     (one file per        (Wi-Fi)          companion computer
   IDPS venue)                         limits, spacing)     drone slot, NED)                    ──MAVLink──▶ PX4
                                                                                                      ▲
                                IDPS engine (PC) ── positions in NED, 50–120 Hz, over Wi-Fi ──────────┘
```

Decided: shows are designed in **Skybrush Studio for Blender**, and the drones run **PX4**.
Skybrush's own upload and flight tools target its ArduPilot-based firmware, so IDPS does the
PX4 side:
* checking the show against the venue;
* the per-drone files;
* the player on each drone;
* the position stream.

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
4. **Export** from Skybrush Studio's Export panel:
   * **.skyc**, the main format: trajectories and lights;
   * **CSV (ZIP)**: one CSV per drone (time in ms, x, y, z, RGB). Use this if the .skyc export
     is unavailable, or for the LED colours (IDPS reads colours from the CSV; .skyc light
     programmes are kept as-is).

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

## 3. Export one file per drone slot

```
idps-show export show.skyc --site venue.yaml --plan plan.yaml -o out/show
```

The export **runs the checks first and refuses if there are errors**. `--force` overrides
this, for testing only. It writes:

* `slot_NN.json` (`idps-traj/1`), one per show drone, containing:
  * the trajectory sampled at 20 Hz in the venue's NED frame;
  * velocity and acceleration (feed-forward for PX4);
  * LED colour;
  * yaw fixed at 0 = North (D7);
  * the home spot.
* `manifest.json`: the slots, their home spots, the show length, the placement and the check
  result, with a checksum per file.

**Slots, not drones.** The show defines slots (Drone 1, Drone 2, …). At the venue, any drone
can be put on any slot's home spot. IDPS identifies which drone stands there from its marker
ID, and that drone gets the slot's file.

## 4. On each drone (PX4)

**Hardware per drone:**
* PX4 flight controller;
* a small **companion computer** with Wi-Fi, e.g. a Raspberry Pi Zero 2 W, connected to the
  flight controller by serial (MAVLink);
* the IDPS IR marker(s);
* the show LEDs.

**Positions in.** The PC sends each drone its IDPS position over Wi-Fi. The companion forwards
it to PX4 as MAVLink `ODOMETRY` (or `VISION_POSITION_ESTIMATE`). PX4's EKF2 fuses it as
external vision:
* `EKF2_EV_CTRL`: horizontal and vertical position (plus yaw, see below);
* `EKF2_HGT_REF` = vision;
* `EKF2_EV_DELAY` = the measured latency.

PX4's local frame then *is* the venue's NED frame.

**Show out.** `idps-show play slot_NN.json` (the reference player, `idps/show/runner.py`)
streams `SET_POSITION_TARGET_LOCAL_NED` at 50 Hz in OFFBOARD mode:
* it sends position, velocity and acceleration, and yaw 0 (type mask 2048: everything except
  yaw rate);
* it starts streaming the home position a few seconds early, because PX4 only accepts
  OFFBOARD once setpoints are flowing.

`--dry-run` prints the setpoints without a drone.

**Start and timing.** The companions keep their clocks with chrony over Wi-Fi; millisecond
accuracy is plenty for show timing. The PC tells every drone the show start time, and every
player starts at that instant.

**Failsafes (PX4 parameters):**
* loss of positions → land (EKF2 timeouts);
* loss of the offboard stream → land (`COM_OF_LOSS_T`, `COM_OBL_RC_ACT`);
* a geofence matching the performance area.

## Open questions

* **Drone heading (yaw).** PX4 needs to know which way each drone faces. One IR marker gives
  position only, and magnetometers are unreliable indoors (steel trusses, motors). Proposal:
  **two markers per drone** (front and back, known offsets), so IDPS measures the heading and
  sends it in `ODOMETRY`. To decide with the drone design.
* **Companion computer and Wi-Fi.** Choose the board (weight, power), and plan a dedicated
  5 GHz Wi-Fi network for positions and commands, separate from the camera network.
* **Still to build:**
  * the engine's position stream to the drones (with the positioning engine);
  * a dashboard **Show** tab: 3D playback, check results, slot assignment, start and abort;
  * rehearsing a show in the simulator with the live-monitoring brain watching.
* **Verify with PX4 SITL** (the simulator) before the first flight:
  * the frame alignment;
  * the OFFBOARD entry;
  * the failsafes.
