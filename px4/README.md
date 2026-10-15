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
noise be scaled from the environment; without it `IMU_NOISE` has no effect. Apply
`px4/sitl/sih_realtime_pacing.patch` too: it keeps the simulated clock on the wall clock (see
"The loop needs a machine that runs it in real time" below). Both together:
`git apply /path/to/IPS/px4/sitl/sih_noise_scale.patch /path/to/IPS/px4/sitl/sih_realtime_pacing.patch`.

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
| Slow attitude and rate loops: with `MC_ROLL_P` 10 the attitude followed its setpoint ~110 ms late (rate loop ~4 Hz, a real 60 g quad has 10–20 Hz). The position loop had ~6° phase margin at 1.2 Hz: after any jolt it rang and **circled** (error direction swept 1600–1800°, 3 s to settle), and velocity P 9 flew away | `MC_ROLL_P`/`MC_PITCH_P` 20, `MC_*RATE_K` 0.25, `MC_*RATE_D` 0.0015, `IMU_DGYRO_CUTOFF` 50; outer loops `MPC_XY_P` 4, `MPC_XY_VEL_P_ACC` 8, `MPC_XY_VEL_D_ACC` 0.15, I 0.5. Same jolt: settles in 0.7–1.1 s with 2 swings, no orbit |
| The rest of the lag delays the acceleration feed-forward; PX4's velocity D term (on measured acceleration) takes back D × the feed-forward | Module: acceleration taken `IDPS_ACC_LEAD` ahead (micro60: 0.1 s) and scaled by 1 + D |
| Feed-forwards were per show second, but the show runs on the PC's clock (in SITL 4–5 % off PX4's) | Module scales velocity by the clock rate and acceleration by its square (no effect on hardware) |
| Drag carried by the vertical P term alone | `MPC_Z_P` 4, `MPC_Z_VEL_P_ACC` 12, `MPC_Z_VEL_I_ACC` 4 |
| A new clock-sync point moved show time by up to 10 ms at once: a 1 cm setpoint jump at 1 m/s | The module plays the show on a slewed clock (catches up at 5 ms/s, never steps) |

Ruled out on the way (logs and models of each link): the show file and the 200 Hz setpoints
(~1 mm), EKF2 on real motion (8–12 ms, ~4° at 1.2 Hz; it does overshoot a sudden jump in the
positions by ~50 %), PX4's use of the feed-forwards (correct, nothing saturates), SIH's ×10
inertia/arm scaling (exact) and host load (a flight with 4 CPU burners was clean).
`sitl_vision kick N E D [ms]` jolts a drone's positions to measure how it settles.

**One command** builds the show, starts the drones, flies and scores it:

```
PX4_DIR=~/PX4-Autopilot px4/sitl/run_loop.sh vision    # engine-like positions inside PX4
PX4_DIR=~/PX4-Autopilot px4/sitl/run_loop.sh cameras   # the closed loop: simulated cameras + engine
```

`N=3` flies 3 of the 5 drones; results and logs go to `out/loop`.

**Or in the dashboard** (`idps-dashboard`), **PX4 flight** tab: pick quick check or full loop,
the number of drones and your PX4-Autopilot folder, and press **Fly the show**. The flight
shows live in 3D, each drone coloured by its distance from its file (within 30 mm green), with a
per-drone table (median, p95, worst; double-click a drone to jump to its worst moment) and the
distance over the whole show. **Open run…** replays any finished run folder (`out/loop`,
`out/px4_runs/...`).

To measure tracking, the module reports how far the drone is from its file at its own show time
(`DEBUG_FLOAT_ARRAY "track"`, logged by the stand-in as `truth.track.csv` and by `idps-sim px4`
as `track.csv`). Comparing the PC's clock with the simulator's would add the simulator's clock
slips (it runs a few % off real time and stalls under load) to the error.

**Results** (5 drones, the whole show, positions inside PX4 via `SITL_VISION`, `IMU_NOISE` 0.1):

| Positions | Median | Worst (all drones) |
|---|---|---|
| **Like the engine, current tuning** (`run_loop.sh vision`), 3 runs | **3 mm** (p95 5–8) | **13–15 mm** |
| Exact, no delay (before the retune, as the rows below) | 3–4 mm | 20 mm |
| Exact, 20 ms late (`EV_DELAY_MS=20`) | 3 mm | 15 mm |
| Like the engine: 2 mm noise, 120 Hz, 28 ms late (`SITL_VISION="28 2 120" EV_DELAY_MS=28`), before the attitude retune | 4–5 mm | 22 mm |
| Exact, but `IMU_NOISE` 0.25 | 6–8 mm | 31 mm |
| Exact, 40 ms late and not compensated (`EV_DELAY_MS=0`) | 5–8 mm | 51 mm |

* **Repeat runs.** Before the attitude retune, some runs showed one smooth 4–10 cm excursion
  (on 5-drone runs, often at show time ~45 s on the drones flying the turning square); a run under
  4 CPU burners was clean, so host load is not the cause. Since the retune and the
  acceleration lead, 5 of 5 runs stayed within 16 mm. If one reappears, fly with
  `PX4_PARAM_SDLOG_MODE=0` and look at that moment in the PX4 logs.
* **The IMU's vibration decides it.** At SIH's full noise the drones wander 2 cm RMS whatever the
  tuning (worst 7–10 cm); at 0.1 they stay within 2.2 cm, at 0.25 just over 3 cm. 0.1 is still
  several times a bare MEMS IMU's noise; on a real 60 g quad it means balanced props and a
  soft-mounted flight controller. Measure it on the real airframe (PX4's `sensor_combined` in
  hover) before trusting these numbers.
* **Set `EKF2_EV_DELAY` to the real delay.** 40 ms of uncompensated delay doubles the worst case.

**No quadcopter can accelerate downward faster than gravity:** its propellers only push. A
path that asks for more than 9.8 m/s² downward (a fast vertical bounce) cannot be followed at
any weight, however the drone is tuned.

### 20 drones

`examples/shows/twenty_drones_rings_wall.py` (117 s): a 5 × 4 take-off grid 1.5 m apart, two rings
that turn once (the 14 outer drones at 2 m, the 6 inner ones at 3.5 m), a 5 × 4 vertical wall that
turns once, every drone back on its own spot; up to 1.16 m/s, closest approach 1.10 m, the 8 × 8 m
floor's 11 cameras see every drone all the time. Who goes where is safe by construction (drones keep
their bearing into the rings; each grid row climbs to its own height before sliding into the wall).
`run_loop.sh` flies it for N = 6–20 (drones 1..N of it); the dashboard's drone count goes to 20.

* The cameras and engine on this show with drones exactly on their files: all 20 identified,
  median 1.3–1.7 mm, worst 10 mm; filming runs at about 2× real time in one process.
* **Each flying PX4 drone takes about 30 % of a CPU core** (27–36 % measured): 20 drones about 6–7
  cores before the cameras and engine. Check the end-of-run report ("why drones left their files")
  for simulator stalls before trusting a 20-drone result on a laptop.
* Above 10 drones PX4's own startup sends every instance's offboard link to one port (14549):
  `run_sitl.sh` moves it to 15540+i for instances 10 and up. `idps-show start` sends the start to
  every drone before reading any answer (one after the other, the last of 20 got it too late).
* On a 4-core machine 20 drones start, take the upload and the start, but PX4 then reports sensor
  timeouts (it cannot keep up) and drones fall into failsafes: use 8+ performance cores.

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
* **Simultaneous 5–40 cm pushes on several drones were the simulator's clock, not the flying.**
  Lockstep SIH sleeps a fixed time after each physics step, so every oversleep and host stall is
  lost for good: measured here, the PX4 clocks fell 1.8 s behind over the 63 s show, in 50–70 ms
  jumps when the host was busy, exactly when several drones were pushed at once. The positions
  the engine sent were right (within 4 mm); those queued while PX4 stood still reached it together
  and looked fresh. `sih_realtime_pacing.patch` paces SIH on the wall clock and catches up after a
  stall (the clocks then stay within 4 ms, and the physics stays consistent through a stall, which
  `px4_sitl_nolockstep` does not: there a stall is a jolt in the physics). The dashboard's camera
  loop uses `px4_sitl_default` when the patch is in the PX4 source, `px4_sitl_nolockstep` if not.
  5 drones, 11 cameras, 4 cores: 0 of 3 runs within
  30 mm before; after, 8 of 10 (worst 15.5–21.8 mm). Of the other two, one had a 12 cm push
  after a 76 ms hold-up on the PC side, and in one three drones lifted off 3.5 s late.
  On a Mac (dashboard, camera loop, 5 drones): 382 mm worst before (`px4_sitl_nolockstep`), 16.1 mm
  worst, 6 mm median, every drone within 30 mm after.
* **A position that reaches PX4 late pulls a moving drone back.** PX4 takes every position as
  filmed `EKF2_EV_DELAY` before it arrived; one held up 50 ms by a busy PC is 5 cm behind at
  1 m/s. With `idps-engine live --ev-delay-ms` (the drones' `EKF2_EV_DELAY`) a position more than
  30 ms older than that is not sent: a short gap does no harm (EKF2 predicts with the IMU). Moving
  late positions along the engine's velocity instead made it worse (that velocity is too noisy:
  p95 rose from 11–12 to 13–18 mm). Not proven in SITL yet (`DROP_LATE=1 run_loop.sh cameras`
  turns it on): 0 of 3 runs within 30 mm with it, on a 4-core machine whose stalls vary a lot.
* **The default take-off ramp was the worst error.** Each drone sat on the ground for 1.5 s
  while the file climbed, then lurched up to 24 cm sideways: up to 64 cm off in height. With
  `MPC_TKO_RAMP_T` = 0.5 s it lifts off with the file.
