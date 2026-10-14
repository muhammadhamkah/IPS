# Positioning engine (`idps-engine`)

The engine runs on the PC. It turns the bright dots that every camera station reports into
each drone's position, and sends each drone its own position. Design: ARCHITECTURE.md §5.
Code: `idps/engine/`.

```
stations ──UDP──▶ ingest ─▶ 1/120 s slots ─▶ match to tracks ─▶ triangulate ─▶ Kalman filter ─▶ position (NED)
 (dots + time)    (reassemble,   (all cameras'   (project each       (lines of sight   (per marker)     ├─▶ drone N (ODOMETRY)
                   count loss)    frames)         track's prediction)  cross)                          └─▶ log / dashboard
                                         └─▶ unmatched dots ─▶ start new tracks (3 cameras agree)
                                                      blink code ─▶ marker ID = drone ID (D21)
```

## How it works

1. **Ingest.** Each packet is checked. Frames split over several packets are put back
   together, duplicates are dropped, and lost packets are counted per station.
2. **Lines of sight.** Each dot is converted from pixel to direction, with the lens
   distortion removed. This is done for every camera's dots in a slot at once.
3. **Matching.** Each track's predicted position, at each camera's exact exposure time, is
   projected into that camera. The nearest dot within the gate belongs to that track.
   * The gate is 4 px plus the track's own uncertainty.
   * Nearest pairs win first, and each dot is used once.
   * A moving track can't take a dot on a known fixed reflection.
   * A dot the station flags as two merged markers is left out when 3 other cameras see the
     drone.
4. **Screening.** The engine finds the point closest to all of a track's lines of sight
   (weighted by distance). That point is used only to check the slot:
   * a camera that misses it by more than 3 px is dropped;
   * a point more than 5σ from the prediction is treated as a glitch and skipped, unless
     that happens 3 times in a row (then it's real motion).
5. **Tracking.** A constant-velocity Kalman filter runs per marker. It is updated directly
   with each camera's **line of sight at that camera's own exposure time**. The drone's
   position and velocity must put it on every line at that instant, so both are estimated
   together.
   * The first version triangulated a point and corrected each camera's line with the
     estimated velocity. A slightly wrong velocity then biased the point, slots with
     different camera sets pulled the velocity apart, and the result was short 10–18 mm
     bursts. Updating with the lines themselves removes that feedback.
   * A track is confirmed after 8 updates and deleted after 0.5 s without one.
6. **New tracks.** These start from leftover dots: two cameras whose lines nearly cross
   inside the volume, confirmed by a dot in a third camera.
   * Each camera's **fixed reflections** are learned: a leftover dot that stays within
     0.5 px for 2 s. They aren't used to start tracks.
7. **IDs.** Every 0.5 s, each track's blink code is decoded from 2.5 s of brightness, taken
   from all the cameras that see it.
   * Each camera's slow brightness trend (distance, angle) is removed with an exact
     one-code (1 s) moving mean.
   * Chip timing is found, then the code is decoded and its check bits verified.
   * Two agreeing decodes name the track.
   * If a track is lost (an occlusion, a jump), a new track starting where it was lost takes
     over its ID at once, instead of waiting about 3 s for the code again.
8. **Output.** Each identified track gives:
   * its position in the venue's NED frame (from the site's origin and North), velocity,
     covariance and camera count;
   * in live mode, a MAVLink `ODOMETRY` to the drone with the same ID (heading 0, marked
     VISION, D20).

## Results (simulator, calibration with realistic errors)

The simulator models:
* centroid noise;
* lens distortion;
* PTP-synced Pi 5 clocks;
* packet loss and reordering;
* merged blobs;
* fixed reflections and false dots;
* occlusion by other drones;
* tilting drones with the marker 8 cm above the centre.

The engine is given the *estimated* calibration (camera pose errors of 0.05–0.1 mrad and
2 mm), not the true one.

| Scenario | Drones / cameras | Identified after | Position error: median / worst p95 / worst | Available | Wrong IDs | Speed (C++ core) |
|---|---|---|---|---|---|---|
| Lab 6 × 6 × 3 m (`lab_three_drones`) | 3 / 9 | 3.1 s | 1.9 mm / 4.0 mm / 4.7 mm | 100 % | 0 | 41× real time |
| Venue 50 × 20 × 20 m (`venue_swarm`) | 25 / 17 | ≤ 3.6 s | 1.6 mm / 3.8 mm / 8.0 mm | 100 % | 0 | 8.7× real time |
| Venue, **the 50-drone target** (`venue_swarm_50`) | 50 / 17 | ≤ 3.8 s | 1.7 mm / 4.8 mm / 8.8 mm | 100 % | 0 | **5.2× real time** |

The requirement is p95 ≤ 10 mm.

## Speed and delay

The per-slot work (matching, triangulation, tracking, new tracks, reflections, blink codes)
runs in a **C++ core**, `idps/engine/_core.cpp` (pybind11, built by `pip install`). It is a
line-by-line port of the Python engine in `engine.py`, which stays as the reference and as
the fallback when no compiler is available. A test checks that both give the same tracks, IDs
and positions (to 1e-9 m) on a simulated lab show.

**50 drones / 17 cameras** (this cloud machine, one core):

| | Python reference | C++ core |
|---|---|---|
| Speed | 0.2× real time | **5.2× real time** |
| Processing per 1/120 s slot | | p99 3.8 ms (budget 8.3 ms) |
| Delay, camera exposure → position out | | **median 19 ms, p99 22 ms, worst 28 ms** |

The delay is mostly fixed by design:
* the slot (on average half of 8.3 ms);
* waiting 8 ms for late packets (`--reorder-ms`; 4–8 ms is enough for wired PTP stations,
  while Wi-Fi stations need about 25 ms);
* the station's own processing (about 3 ms);
* the network (about 2 ms).

PX4 is told this delay with `EKF2_EV_DELAY`. It stamps each position on arrival, so one that
arrives later than that (a busy PC, a held-up frame) would look newer than it is: with
`--ev-delay-ms` set to the drones' `EKF2_EV_DELAY`, `live` moves each position along its velocity
to the age PX4 assumes, and does not send one over 100 ms late.

**Two things made the difference beyond C++:**
* Each dot is read from the packet straight into arrays.
* Blink-code decoding is spread out: at most 2 decodes per slot, and every 2 s, not 0.5 s,
  once a drone is named. Before, all drones decoded in the same slot, which gave 35–50 ms
  spikes.

**Line-of-sight filter update.** Updating the filter with each camera's line of sight
(above), instead of a triangulated point, cut the worst p95 at 50 drones from 9.3 to 4.8 mm,
and the worst single position from 18 to 8.8 mm. It costs some speed (6.1× → 5.2× real
time) for the larger update per track.

## Use

```
pip install -e .              # also builds the C++ core (needs a C++ compiler: Xcode command line tools on a Mac)
idps-sim run examples/scenarios/venue_swarm_50.yaml -o out/sim
idps-engine replay out/sim/packets.idpsraw --calib out/sim/calibration.yaml --truth out/sim/truth.csv [-o positions.csv]

idps-engine live --port 5600 --calib calibration.yaml --site venue.yaml \
    --link 1=udpout:192.168.50.101:14550 --link 2=udpout:192.168.50.102:14550 --ev-delay-ms 30 -o positions.csv
```

* `--truth` scores the output against the simulator's ground truth.
* `--true-calib` (simulator only) uses the exact camera poses, to separate calibration error
  from everything else.
* `--backend python` runs the Python reference instead of the C++ core.

## Closed loop with PX4

`idps-sim px4` films PX4 simulator drones in real time and streams to `idps-engine live`.
The engine's positions are all PX4 flies on. With 3 drones flying the demo show, the
engine's positions in flight were within p95 3.0 mm (worst 5.1 mm) of the marker the
cameras filmed. Setup and results are in [px4/README.md](../px4/README.md#closed-loop-the-simulated-cameras-fly-the-drones).

## Not done yet

* **The pre-flight spot check on live marker IDs** (`Fleet.preflight`): it needs the
  engine running at the venue.
* **Asynchronous per-camera updates** (ARCHITECTURE §5.1, Phase 5+), multi-marker drones and
  live re-pose of moving gimbals.

**Simulator change made with the engine:** the drone's tilt now follows its acceleration
with a 0.08 s lag. Before, it flipped instantly at waypoints, so the marker 8 cm above the
centre jumped about 12 cm in one step: a "teleport" no real drone makes, which the engine
had to survive.
