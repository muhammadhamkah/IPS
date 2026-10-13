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
   * At 0 it flies the file at 200 Hz: position, velocity and acceleration, interpolated
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

For the 60 g simulator profile, also apply `px4/sitl/sih_noise_scale.patch` before building the
simulator (`git apply /path/to/IPS/px4/sitl/sih_noise_scale.patch`). It only lets SIH's IMU
noise be scaled from the environment; without it `IMU_NOISE` has no effect.

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
| `EKF2_EV_DELAY` | measured | Camera exposure to PX4: what `idps-engine live` reports as "exposure to send", plus Wi-Fi |
| `EKF2_DELAY_MAX` | `EKF2_EV_DELAY` + 10 ms | EKF2 fuses this far in the past and predicts the rest from the IMU. PX4's 200 ms default leaves centimetres of IMU drift in every position |
| `MPC_TKO_RAMP_T` | 0.5 s | PX4 eases into a take-off over 3 s by default, while the show file climbs from its first second |
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

## Small show drones (60 g)

`DRONE=micro60 px4/sitl/run_sitl.sh 5` flies SIH as a 60 g indoor show quad instead of its 1 kg
default:
* the airframe: a 65 mm frame (motors 2.3 cm from the axes), thrust-to-weight 3.5, 20 ms
  motors and its real inertia;
* show-drone limits: 5 m/s up, 4 m/s down, 60° tilt;
* controller gains to suit: rate gains scaled for its ~10× larger angular authority, 720°/s
  rates, and firmer position and velocity loops.

SIH only inverts the inertia matrix when its determinant is large enough, and a real 60 g quad's
is not: the airframe then silently never rotates. The profile therefore multiplies inertia and
the torque arms by the same factor of 10. The dynamics are identical, and the inverse is valid.

**A 5-drone show** (114 s, up to 2.3 m/s and 8.5 m/s²), flown on exact positions:
* all five flew and landed;
* about 9 cm median from the path, p95 22–24 cm, worst 0.44–0.6 m.

### Within 3 cm of the file

The choreographed 5-drone show (`examples/shows/five_drones_sphere_square.py`: grid, sphere,
360° turn, square, turn, grid; 63 s, up to 1.2 m/s) first flew 6–7 cm median and 17–27 cm worst
on micro60. What it took to get within 3 cm:

| Cause | Fix (all in `run_sitl.sh`'s micro60 profile or the module) |
|---|---|
| EKF2 fused vision 200 ms in the past (`EKF2_DELAY_MAX`, raised by unused sensors' delays) and integrated the noisy IMU over the rest: 2–4 cm of estimate error | `EKF2_DELAY_MAX` = vision delay + 10 ms; other sensors' delays 0; `EKF2_TAU_POS/VEL` 0.1; `EKF2_ACC_NOISE` 1.5 |
| SIH adds a fixed-wing's in-flight IMU noise (accelerometer 0.5/1.7/1.4 m/s², gyro 8°/s, white): the drone wanders 2–3 cm RMS even hovering | `px4/sitl/sih_noise_scale.patch` lets `IMU_NOISE` scale it; micro60 uses 0.1 (see below) |
| Velocity loop with D and a large I term; drag carried by the vertical P term alone | `MPC_XY_VEL_D_ACC` 0, `MPC_XY_VEL_I_ACC` 0.5, `MPC_XY_P` 5, `MPC_Z_P` 4, `MPC_Z_VEL_P_ACC` 12, `MPC_Z_VEL_I_ACC` 4 |
| A new clock-sync point moved show time by up to 10 ms at once: a 1 cm setpoint jump at 1 m/s | The module plays the show on a slewed clock (catches up at 5 ms/s, never steps) |

**One command** builds the show, starts the drones, flies and scores it:

```
PX4_DIR=~/PX4-Autopilot px4/sitl/run_loop.sh vision    # engine-like positions inside PX4
PX4_DIR=~/PX4-Autopilot px4/sitl/run_loop.sh cameras   # the closed loop: simulated cameras + engine
```

`N=3` flies 3 of the 5 drones; results and logs go to `out/loop`.

To measure tracking, the module reports how far the drone is from its file at its own show time
(`DEBUG_FLOAT_ARRAY "track"`, logged by the stand-in as `truth.track.csv` and by `idps-sim px4`
as `track.csv`). Comparing the PC's clock with the simulator's would add the simulator's clock
slips (it runs a few % off real time and stalls under load) to the error.

**Results** (5 drones, the whole show, positions inside PX4 via `SITL_VISION`, `IMU_NOISE` 0.1):

| Positions | Median | Worst (all drones) |
|---|---|---|
| Exact, no delay | 3–4 mm | 20 mm |
| Exact, 20 ms late (`EV_DELAY_MS=20`) | 3 mm | 15 mm |
| Like the engine: 2 mm noise, 120 Hz, 28 ms late (`SITL_VISION="28 2 120" EV_DELAY_MS=28`) | 4–5 mm | **22 mm** |
| Exact, but `IMU_NOISE` 0.25 | 6–8 mm | 31 mm |
| Exact, 40 ms late and not compensated (`EV_DELAY_MS=0`) | 5–8 mm | 51 mm |

* **Repeat runs vary.** Most runs stay within 2.2 cm, but some show one smooth 4–10 cm sag on
  one or two drones at once (3-drone runs: 5 of 6 within 3 cm; 5-drone runs on a 4-core machine:
  1 of 4). That the sags hit separate simulated drones at the same instant points to the host
  starving the simulators rather than the flight code; confirm on a faster machine.
* **The IMU's vibration decides it.** At SIH's full noise the drones wander 2 cm RMS whatever the
  tuning (worst 7–10 cm); at 0.1 they stay within 2.2 cm, at 0.25 just over 3 cm. 0.1 is still
  several times a bare MEMS IMU's noise; on a real 60 g quad it means balanced props and a
  soft-mounted flight controller. Measure it on the real airframe (PX4's `sensor_combined` in
  hover) before trusting these numbers.
* **Set `EKF2_EV_DELAY` to the real delay.** 40 ms of uncompensated delay doubles the worst case.

**No quadcopter can accelerate downward faster than gravity:** its propellers only push. A
path that asks for more than 9.8 m/s² downward (a fast vertical bounce) cannot be followed at
any weight, however the drone is tuned.

## Closed loop: the simulated cameras fly the drones

Here the stand-in is replaced by the real thing. Nothing in the loop knows where a drone is
except through the cameras:

```
SITL drones (PX4 + SIH physics) ──true state──▶ idps-sim px4: 9 simulated camera stations, in real time
        ▲                                                   │ UDP, the real wire protocol
        └──── ODOMETRY (VISION) ◀── idps-engine live ◀──────┘
```

```
EV_DELAY_MS=20 px4/sitl/run_sitl.sh 3                     # 20 ms: camera exposure to the drone
idps-sim px4 examples/scenarios/lab_three_drones.yaml --manifest out/show/manifest.json -o out/loop &
idps-engine live --port 5600 --calib out/loop/calibration.yaml --site examples/sites/lab.yaml \
    --link 1=udpin:127.0.0.1:14030 --link 2=udpin:127.0.0.1:14031 --link 3=udpin:127.0.0.1:14032 \
    -o out/loop/positions.csv &
idps-show upload out/show/manifest.json $L --sitl
idps-show start  out/show/manifest.json $L --lead 10      # prints "show time 0 = ..."
python px4/sitl/score_loop.py out/loop out/show/manifest.json --start <show time 0>
```

* **`idps-sim px4`** takes its cameras and camera, timing and network models from the
  scenario. Its drones are the show's drones, each standing on its home spot at the start.
  * Each SITL drone's true state arrives 100 times a second, with its exact position from the
    module (`DEBUG_VECT "truth"`; latitude and longitude alone come in 1.1 cm steps).
  * Each state is timed by PX4's own clock, offset by its quickest delivery: timing states by
    when Python received them added tens of ms of jitter, centimetres at show speeds.
  * The last 0.64 s of states are kept, so a frame filmed late still sees each drone where it
    was at the frame's time.
  * The stations film on the wall clock, and `truth.csv` records what they filmed; `track.csv`
    is each drone's distance from its file (see above).
* **`--workers K`** shares the cameras out over K processes, as a real install has one computer
  per camera. With 5 SITL drones and 11 cameras on a 4-core machine, two workers kept the
  cameras within about 40 ms of the clock (p99).
* **The engine reaches each drone on PX4's payload link (udp 14030+i).** A SITL drone has
  6 MAVLink links: PX4 uses 5 and the truth link is the 6th.
* **The drones' states are received in a separate process.** Inside the camera loop's own
  process, the receiving threads held up the cameras by up to 400 ms (Python's interpreter
  lock). Now the cameras stay within about 15 ms of the clock.

**Results** (3 drones, the 26 s demo show, the lab's 9 cameras, positions only from the
engine):

| Test | Result |
|---|---|
| Before take-off | The engine named all three drones by their blink codes. Every drone's start checks (positions arriving, within 0.5 m of its home spot) passed on the engine's positions |
| Show | All three took off at the same instant, flew their parts and landed |
| Engine position vs the marker the cameras filmed | **In flight: p95 2.8–3.0 mm, worst 5.1 mm.** On the ground: p95 ≤ 2.8 mm |
| PX4's own estimate vs the truth | 2 mm on the ground; 2–3.5 cm in flight |
| Flight (true centre vs the show file) | Median 10–12 cm, p95 23–27 cm, worst 38–43 cm at the demo path's sharp corners |

* **Flight tracking is about as good as with the perfect-position stand-in** (8–11 cm median,
  about 30 cm p95). It comes from PX4 controlling the simulated quad (it wobbles about ±10 cm
  even on smooth parts of the path), not from the positions. Tune the real airframe.
* **`idps-engine live` reports "exposure to send" every 2 s** (about 25–30 ms here): set
  `EKF2_EV_DELAY` to it (`EV_DELAY_MS` for `run_sitl.sh`), plus the Wi-Fi delay on real drones.
* **The loop needs a machine that runs it in real time.** On a 4-core cloud machine, 5 micro60
  drones and 11 cameras at 60 fps did not fit: with PX4 logging on, the cameras fell seconds
  behind, and PX4's simulated clocks (lockstep: each instance's own, a few % off and stalling
  under load) disagreed with the cameras' by tens to hundreds of ms. Positions then reached the
  drones late and out of step, and the drones flew away although the engine's positions were
  right (1.5–1.9 mm median, p95 3–4 mm). `PX4_BUILD=px4_sitl_nolockstep` (`make
  px4_sitl_nolockstep ...`) runs PX4 on the real clock but costs about half a core per drone.
  Use 8+ cores, `PX4_PARAM_SDLOG_MODE=-1` (no PX4 logs), and check the cameras' "behind the
  clock" and the engine's "exposure to send" before trusting a loop result.
* **The default take-off ramp was the worst error.** Each drone sat on the ground for 1.5 s
  while the file climbed, then lurched up to 24 cm sideways: up to 64 cm off in height. With
  `MPC_TKO_RAMP_T` = 0.5 s it lifts off with the file.
