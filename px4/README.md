# IDPS on the drone: the `idps_show` PX4 module

Each drone flies its own part of the show from its SD card (decision D22). The PC uploads
the files, keeps the drones' clocks together, sends one start time, and streams the IDPS
positions. The whole show protocol is in
[`src/modules/idps_show/IdpsShow.hpp`](src/modules/idps_show/IdpsShow.hpp), and the PC side is
in `idps/show/onboard.py`.

```
idps-show export ─▶ drone_007.trj ─▶ idps-show upload ─(MAVLink FTP, Wi-Fi)─▶ /fs/microsd/idps_show.trj
                                     idps-show start  ─(start time + clock sync)─▶ idps_show module ─▶ PX4 OFFBOARD
                                     IDPS engine      ─(ODOMETRY: position, heading 0)─▶ EKF2
```

## What the module does

1. **At boot** it loads `idps_show.trj` from the SD card and checks it: the format, the CRC
   and the drone ID (it must equal `MAV_SYS_ID`).
2. **On `start`** it checks the drone, then counts down on the PC's clock. It checks that:
   * the file matches the PC's copy (by CRC);
   * the drone is landed and disarmed;
   * IDPS positions are arriving;
   * the drone is within 0.5 m of its home spot;
   * clock sync with the PC is running;
   * the start is 4–600 s ahead.
3. **Countdown and flight:**
   * It streams the home setpoint.
   * At −3 s it switches to OFFBOARD; at −2 s it arms.
   * At 0 it flies the file at 50 Hz: position, velocity and acceleration, interpolated
     between the 20 Hz samples, with yaw fixed at 0 (North).
4. **At the end** it commands LAND, and PX4 disarms on touchdown.
5. **On `abort`** it commands LAND where the drone is. If anything else takes the drone out
   of OFFBOARD (a failsafe, or the pilot), it stops and leaves the drone to PX4.

**Clock.** The PC sends its time to every drone five times a second. Wi-Fi delay only ever
adds time, so each drone keeps the fastest message of each second and fits offset and drift
through the last 16. Every drone therefore runs the show on the PC's clock, and its own
crystal doesn't matter.

**Not done yet: the show LEDs.** The module works out each moment's colour (`idps_show
status` prints it), but driving the LED hardware depends on the LEDs you choose.

## Build

PX4 v1.16 or later. The module is built in as an external module, and stock PX4 is unchanged.
It uses PX4's reserved command number 0 (`VEHICLE_CMD_CUSTOM_0`), which PX4's commander
leaves for modules.

```
git clone --recursive --branch v1.16.0 https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
make px4_fmu-v6c_default EXTERNAL_MODULES_LOCATION=/path/to/IPS/px4   # your flight controller's target
make px4_sitl_default    EXTERNAL_MODULES_LOCATION=/path/to/IPS/px4   # simulator
```

The PX4 build needs Python 3.11 or 3.12; its `empy` package fails to build on 3.13. A shallow
clone also needs a NuttX tag for the version script:
`git -C platforms/nuttx/NuttX/nuttx tag nuttx-11.0.0`.

## On each drone

* **Start the module at boot.** Add one line to `etc/extras.txt` on the SD card:
  ```
  idps_show start
  ```
* **Parameters:**

| Parameter | Value | Why |
|---|---|---|
| `MAV_SYS_ID` | the drone ID (1–255) | Same as its marker ID and label (D21) |
| `EKF2_EV_CTRL` | 11 | Fuse IDPS horizontal and vertical position, and yaw |
| `EKF2_HGT_REF` | 3 (vision) | Height from IDPS |
| `EKF2_GPS_CTRL`, `EKF2_BARO_CTRL` | 0 | Indoors: IDPS only |
| `EKF2_MAG_TYPE` | 5 (none), and `SYS_HAS_MAG` 0 | Heading comes from IDPS as 0 = North (D20, R20) |
| `EKF2_EV_DELAY` | measured | Camera exposure to PX4, over Wi-Fi |
| `EKF2_EV_POS_X/Y/Z` | marker offset | If the marker isn't at the reference point (R16) |
| `COM_OF_LOSS_T`, `COM_OBL_RC_ACT` | e.g. 0.5 s, Land | If the module stops streaming |
| Geofence | the performance area | Last line of defence |

* **IDPS positions** must be sent as `ODOMETRY` with `estimator_type` VISION. PX4 routes MOCAP
  to a topic its estimator (EKF2) doesn't read. `idps.show.fleet` does this.

## Simulator test (no hardware)

```
px4/sitl/run_sitl.sh 3                                    # 3 SIH quads, drone IDs 1-3, headless
idps-show demo -o demo.skyc --drones 3
idps-show export demo.skyc --site examples/sites/lab.yaml --plan examples/plans/lab.yaml -o out/show
python px4/sitl/idps_stand_in.py out/show/manifest.json 3 &   # plays IDPS: true position -> ODOMETRY
L="--link 1=udpin:127.0.0.1:14540 --link 2=udpin:127.0.0.1:14541 --link 3=udpin:127.0.0.1:14542"
idps-show upload out/show/manifest.json $L --sitl
idps-show start  out/show/manifest.json $L --lead 10
```

**Results** (PX4 v1.16 SITL with SIH, 3 drones, 26 s demo show):

| Test | Result |
|---|---|
| Upload | 20.9 kB per drone in about 7 s. Each drone's CRC matched the PC's |
| Start all three at one instant | All armed, flew their parts and landed (disarmed on touchdown) |
| Clock agreement between drones | Within 2–10 ms. The simulator's clocks drift 2–3 % with CPU load; real crystals drift about 0.002 % |
| Tracking (true position vs file) | 8–11 cm median, about 30 cm p95, at up to 1.3 m/s. This depends on the airframe's tuning: default gains gave twice the error. Tune real show drones |
| Abort mid-show | All three landed where they were |
| Two drones swapped on their spots | Both refused ("2.60 m from home spot"), so nobody started |
