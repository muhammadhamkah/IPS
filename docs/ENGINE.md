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
4. **Triangulation.** The engine finds the point closest to all of a track's lines of
   sight, weighting each line by its distance from the camera.
   * The point is time-stamped at the average exposure time of those cameras. Each line is
     shifted by the drone's velocity to that instant.
   * A camera that misses the point by more than 3 px is dropped.
5. **Tracking.** A constant-velocity Kalman filter runs per marker.
   * A measurement more than 5σ from the prediction is treated as a glitch and skipped,
     unless that happens 3 times in a row (then it's real motion).
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

| Scenario | Drones / cameras | Identified after | Position error: median / p95 / worst | Positions available | Wrong IDs | Speed |
|---|---|---|---|---|---|---|
| Lab 6 × 6 × 3 m (`lab_three_drones`) | 3 / 9 | 3.0 s | 1.9–2.1 mm / 2.1–5.1 mm / 5.7 mm | 100 % | 0 | 3.5× real time |
| Venue 50 × 20 × 20 m (`venue_swarm`), 4 m/s random paths | 25 / 17 | 2.5–3.5 s | 1.7 mm (mean) / ≤ 6.4 mm / 15 mm | 100 % | 0 | 1.0× real time |

The requirement is p95 ≤ 10 mm. The live mode was checked the same way: the simulator
streamed UDP in real time to `idps-engine live`, which output NED positions with the same
accuracy.

**Speed is the open problem.** The engine is plain Python and NumPy. It keeps up with the
lab at 3.5× real time, but only just with 25 drones and 17 cameras. The per-slot work is
already batched. The next steps, in order:
1. move the remaining per-dot Python loops (matching candidates, track start) into array
   code;
2. split ingest and engine into two processes;
3. port the per-slot core to C++ (ARCHITECTURE §5.2), if profiling still says so.

## Use

```
idps-sim run examples/scenarios/venue_swarm.yaml -o out/sim
idps-engine replay out/sim/packets.idpsraw --calib out/sim/calibration.yaml --truth out/sim/truth.csv [-o positions.csv]

idps-engine live --port 5600 --calib calibration.yaml --site venue.yaml \
    --link 1=udpout:192.168.50.101:14550 --link 2=udpout:192.168.50.102:14550 -o positions.csv
```

* `--truth` scores the output against the simulator's ground truth.
* `--true-calib` (simulator only) uses the exact camera poses, to separate calibration error
  from everything else.

## Not done yet

* **Closing the loop with PX4 SITL.** The simulated cameras watching the SITL drones, so
  the engine's positions actually fly them. Today the PX4 tests use a stand-in for the
  engine, and the engine tests use pre-planned paths.
* **The pre-flight spot check on live marker IDs** (`Fleet.preflight`): it needs the
  engine running at the venue.
* **Asynchronous per-camera updates** (ARCHITECTURE §5.1, Phase 5+), multi-marker drones and
  live re-pose of moving gimbals.
* **Speed**, above.

**Simulator change made with the engine:** the drone's tilt now follows its acceleration
with a 0.08 s lag. Before, it flipped instantly at waypoints, so the marker 8 cm above the
centre jumped about 12 cm in one step: a "teleport" no real drone makes, which the engine
had to survive.
