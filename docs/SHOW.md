# Designing and flying a show

```
Blender + Skybrush Studio ──.skyc──▶ idps-show check ──▶ idps-show export ──▶ drone_NNN.json + manifest.json
  (design inside the                 (venue, cameras,     (one file per                │
   IDPS venue)                         limits, spacing)     drone ID, NED)               ▼
                                                                          idps-show fly (PC) ──Wi-Fi, MAVLink──▶ PX4 on each drone
                                                                            • setpoints from each drone's file, 50 Hz
                                                                            • IDPS position + heading 0 (North)
```

Decided: shows are designed in **Skybrush Studio for Blender**, and the drones run **PX4**.
Skybrush's own upload and flight tools target its ArduPilot-based firmware, so IDPS does the
PX4 side:
* checking the show against the venue;
* the per-drone files;
* flying them from the PC.

The drones carry **no companion computer** (D20). Each flight controller is reached over
Wi-Fi, and the PC sends it both its position and its show.

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

* `drone_NNN.json` (`idps-traj/1`), one per show drone, named by its drone ID, containing:
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
**ID 7**, from `drone_007.json`. That drone is put on Drone 7's home spot. Names without
numbers are numbered 1..N in order. Two drones with the same number stop the export.

To swap in a spare drone, give the spare the missing drone's ID (set `MAV_SYS_ID` and its
marker), or rename the drone in the show.

## 4. Fly: the PC drives every drone (PX4)

**On each drone** (D20), there is no companion computer, to save weight:
* a PX4 flight controller with Wi-Fi: either built in, or a few-gram Wi-Fi-to-serial MAVLink
  bridge such as an ESP32 or ESP8266 module (the bridge only relays, it runs nothing);
* **one** IDPS IR marker, at the drone's reference point (top centre, over the centre of
  mass). Declare any remaining offset to PX4 with `EKF2_EV_POS_X/Y/Z`;
* the show LEDs.

**Heading.** Drones never yaw. They always face **North, +X in NED** (D7). Because of that:
* Every drone is put down **facing North** on its home pad (mark each pad with an arrow).
* The magnetometer stays **off**, since it is unreliable indoors.
* The PC sends **heading 0** with every position, and PX4 fuses it as external-vision yaw.
  PX4 needs some heading source, and this is it. See R20 for how far this can be trusted and
  how IDPS will watch for a drone that is turned.

**Flying the show:**
```
idps-show fly out/show/manifest.json --link 1=udpout:192.168.50.101:14550 --link 7=udpout:192.168.50.107:14550 ...
```
`idps/show/fleet.py` runs every drone from one loop at 50 Hz.
* **To each drone it sends:**
  * MAVLink `ODOMETRY`: the IDPS position in NED plus heading 0, for every drone the engine
    currently sees;
  * `SET_POSITION_TARGET_LOCAL_NED`: position, velocity and acceleration from the drone's
    own file, plus yaw 0 (type mask 2048).
* **Home first:** it streams the home setpoints from the moment it starts, because PX4 only
  accepts OFFBOARD once setpoints are flowing. Arm the drones and switch them to OFFBOARD; the
  show starts at `--start`, or 5 s after launch by default.
* **No clock sync on the drones:** all timing comes from the PC.

`--link DRONE_ID=address` gives each drone's network address. `fly` checks that the drone
at that address answers with that `MAV_SYS_ID`, so a mixed-up address can't send Drone 3's
path to drone 7.

**Pre-flight check: every drone on its own spot.** Before arming, the cameras read every
marker ID on the ground, and `FleetPlayer.preflight` blocks take-off on any of these:
* *Drone 4: not seen*;
* *Drone 1: on Drone 2's spot*;
* *Drone 3: 1.00 m from its spot*;
* *Marker 9: a drone that is not in this show*;
* a drone with no link.

The check runs on the engine's positions, so until the engine is built `fly` runs only
against PX4 SITL (`--sitl`), where the simulated drones position themselves.

`idps-show play drone_007.json` flies a single drone (`--dry-run` prints it).

**PX4 parameters:**
* external vision: `EKF2_EV_CTRL` = horizontal position + vertical position + yaw;
  `EKF2_HGT_REF` = vision;
* `EKF2_EV_DELAY` = the measured latency from the camera exposure to PX4 (positions travel
  over Wi-Fi, so measure it on the real link);
* `EKF2_MAG_TYPE` = None. PX4's local frame then *is* the venue's NED frame.

**Failsafes:**
* loss of positions → land (EKF2 vision timeouts);
* loss of the offboard stream, e.g. Wi-Fi drops → land (`COM_OF_LOSS_T`, `COM_OBL_RC_ACT`);
* a geofence matching the performance area.

**Wi-Fi:** use a dedicated 5 GHz network for the drones, separate from the camera network
(cameras are wired, D16). Each drone needs about 50 position and 50 setpoint messages a
second, each under 300 bytes: around 30 kB/s per drone, so 50 drones fit comfortably.
Latency and dropouts matter more than bandwidth; measure them in Phase 7.

## Open questions

* **Drone flight controller with Wi-Fi.** Choose the board (PX4 support, weight, Wi-Fi
  range and latency), or a standard PX4 board plus a Wi-Fi MAVLink bridge.
* **Still to build:**
  * the engine's position stream to `fly`, and the pre-flight check on live marker IDs;
  * the heading monitor (R20);
  * a dashboard **Show** tab: 3D playback, check results, the pre-flight check per drone ID,
    start and abort;
  * rehearsing a show in the simulator with the live-monitoring brain watching.
* **Verify with PX4 SITL** (the simulator) before the first flight:
  * external-vision heading with the magnetometer off;
  * the frame alignment;
  * the OFFBOARD entry;
  * the failsafes.
