# IDPS Software Architecture

Status: **proposal v0.1**, open for review. No production code has been written yet.
Companion documents: [PROTOCOL.md](PROTOCOL.md), [RISKS.md](RISKS.md),
[VALIDATION.md](VALIDATION.md), [ROADMAP.md](ROADMAP.md).

---

## 1. Design principles

1. **Positioning only.** IDPS outputs timestamped marker and drone positions with uncertainty.
   Flight control and Blender trajectory generation consume that output. They are never
   compiled into IDPS.
2. **Observations, not video.** Camera stations send 2D marker observations. Images move only
   on explicit request, for calibration and diagnostics.
3. **Simulator first.** A synthetic camera station speaks the *exact* wire protocol of a real
   one. The engine, UI, recorder and calibration tools cannot tell the difference. This lets
   all central software be built and tested before hardware arrives.
4. **Everything is replayable.** Raw packets are recorded. The engine is driven by data
   timestamps, not by the wall clock, so a recorded session replays bit-for-bit
   deterministically. Every algorithm change can be re-evaluated on old data.
5. **Measure, don't assume.** Each stage reports its own latency, drop counters and error
   statistics. Each accuracy claim carries the test that produced it (see VALIDATION.md).
6. **Never backlog.** Every queue is bounded. When a stage falls behind, it drops the *oldest*
   data and increments a counter. Late data is worth less than no data in a 120 Hz tracker.
7. **Scale-agnostic.** No component assumes a fixed number of cameras, markers or drones.
   The same code runs a 1-camera bench and a 100+ camera hall.
8. **Uncertainty is first-class.** Every 3D position carries a covariance. Every camera
   calibration carries a quality report. The UI shows both.

---

## 2. System context

```text
  Drone markers (850 nm, coded)        Fixed reference beacons (850 nm, surveyed)
            \                                   /
             v                                 v
  +------------------------------------------------------------+
  |  Camera Station  x N        (Raspberry Pi 4B + OV9281)      |
  |  idps-edge (C++)  : capture -> detect -> timestamp -> UDP   |
  |  idps-agent       : control, gimbal, snapshots, health      |
  |  ptp4l / chrony   : clock discipline                        |
  +------------------------------------------------------------+
        | UDP observations (data plane)     ^ TCP/ZeroMQ control
        v                                   |
  +------------------------------------------------------------+
  |  Central PC                                                 |
  |  idps-engine : ingest -> time align -> associate ->         |
  |                triangulate -> track -> identify -> QA       |
  |  idps-record : raw packet + state logging (MCAP)            |
  |  idps-calib  : intrinsics, wand bundle adjustment, re-pose  |
  |  PTP grandmaster (or dedicated sync master)                 |
  +------------------------------------------------------------+
        | ZeroMQ PUB (state @120 Hz)        ^ ZeroMQ REQ (commands)
        v                                   |
  +---------------------------+   +-------------------------------+
  | idps-ui (dashboard)       |   | Consumers (later): flight     |
  | PySide6 + pyqtgraph GL    |   | control, Blender, loggers     |
  +---------------------------+   +-------------------------------+

  Development only:  idps-sim  -> emits the same UDP packets as N virtual stations
                     idps-replay -> replays an MCAP recording into the engine
```

The **sync master** handles time synchronisation, and ROADMAP Phase 5 decides its final form:

* Option A: Ethernet PTP. Software timestamping on the Pi 4B; this NIC capability still needs verifying.
* Option B: a wired hardware trigger line to each camera's FSIN input.
* Option C: a radio broadcast sync pulse that drives each camera's trigger. The same pulse can
  also drive the drone LEDs (see PROTOCOL.md §6).

---

## 3. Coordinate frames, units and time

| Item | Convention |
|---|---|
| Units | SI everywhere: metres, radians, seconds. Wire timestamps are `int64` nanoseconds. |
| World frame `W` | Right-handed, Z up, origin and axes defined by the surveyed reference beacons or the L-frame. |
| Camera frame `C` | OpenCV convention: +X right, +Y down, +Z along the optical axis. |
| Extrinsics | `T_CW` (world→camera) stored as rotation vector + translation, with a 6×6 covariance. |
| Intrinsics | OpenCV pinhole plus a distortion model (Brown-Conrady k1..k3, p1, p2; fisheye model for lenses beyond ~90° HFOV). |
| Pixel coordinates | Stations transmit **distorted** (raw) subpixel coordinates. Undistortion happens on the PC, so recorded data can be reprocessed after re-calibrating intrinsics. |
| Time base | One system time: PTP/TAI nanoseconds disciplined to the sync master. Every observation timestamp is the **mid-exposure instant** in this time base. |

---

## 4. Camera station (edge)

### 4.1 Processes

| Process | Language | Real-time? | Responsibility |
|---|---|---|---|
| `idps-edge` | C++17 | Yes (pinned cores, `SCHED_FIFO`) | Capture, detection, timestamping, UDP transmit |
| `idps-agent` | Python 3 | No | Control API, configuration, gimbal driver, snapshots, health, firmware version |
| `ptp4l`/`phc2sys` or `chrony` | system | — | Clock discipline |

The two processes are split so the real-time path never depends on Python, the GIL or garbage
collection. Commands from `idps-agent` reach `idps-edge` through a small shared-memory
parameter block or a Unix socket.

### 4.2 `idps-edge` pipeline

```text
 core 1                 core 2                 core 3                 core 0
 +-----------+  ring   +-------------+  ring  +-------------+         OS, agent,
 | Capture   |-------->| Detect      |------->| Publish     |-> UDP   ptp4l
 | libcamera |  (N=4)  | blobs +     |  (N=8) | serialise + |
 | raw Y8/Y10|         | centroids   |        | sendmmsg()  |
 +-----------+         +-------------+        +-------------+
   ^ frame buffers are returned to libcamera as soon as Detect releases them
```

* **Capture:**
  * libcamera with a raw sensor stream (no ISP processing).
  * Fixed exposure, gain and frame duration; auto-exposure is disabled.
  * Each frame records the sensor frame counter and `SensorTimestamp`.
* **Detect:** a two-pass sparse algorithm designed for the 8.33 ms frame budget.
  1. *Seed pass:* scan every k-th row (k ≈ 2–4, chosen from the minimum blob size) with NEON
     compares against threshold `T`.
  2. *Region pass:* flood-fill / connected components in a local window around each seed,
     using a lower threshold `T_low` (hysteresis).
  3. *Measure:* background-subtracted, intensity-weighted centroid (subpixel x, y); total
     intensity; peak; area; second moments (elongation flags merged blobs); and flags for
     saturated or edge-touching blobs.
  4. *Optional ROI mode (later):* the engine sends predicted windows, and only those plus a
     sparse full-frame scan are processed.
* **Publish:**
  * Packs observations into the binary format in PROTOCOL.md.
  * Splits frames whose markers exceed one MTU.
  * Sends exactly one packet per frame, even when it contains zero markers; a missing packet
    therefore always means loss, never "nothing seen".
* **Timestamping:**
  * `t_mid = SensorTimestamp + offset(mode) − exposure/2`, converted from `CLOCK_MONOTONIC`
    to the disciplined system time base.
  * `offset(mode)` is **measured** in Phase 2 with an LED-strobe timing rig and stored per
    sensor mode, never assumed.
* **Back-pressure:** rings are fixed-size. On overflow, drop the oldest entry and increment
  `drops_capture` / `drops_detect`. Per-frame processing time is measured and reported.

### 4.3 Benchmark harness (`edge/bench`)

The harness establishes the Pi 4B capability *before* any design depends on it:

* Raw capture rate at each sensor mode: 1280×800, 640×400, and cropped modes.
* Detection time distribution (p50/p99/max) versus marker count (0, 1, 10, 50, 200).
  Inputs are real captures plus synthetic images injected from memory.
* End-to-end exposure-to-packet latency, measured with an LED strobe on a GPIO plus a
  photodiode or a second camera.
* Thermal behaviour over a 30-minute soak: CPU frequency, throttling flags, frame drops.
* Timestamp jitter, measured against a GPIO-driven LED with known timing.

### 4.4 Gimbal

* The `idps-agent` drives the gimbal over an abstract `GimbalDriver` interface, with
  backends for servo, stepper and simulated hardware.
* After any gimbal motion, the station is marked **UNCALIBRATED**. The engine ignores
  its observations until re-posing (§6.4) succeeds.
* Station health reports gimbal state as one of `moving`, `settled` or `locked`.

---

## 5. Central positioning engine (`idps-engine`)

### 5.1 Stage pipeline

```text
UDP sockets ──> Ingest ──> TimeAlign ──> Frame Grouping ──> Association ──> Triangulation
                 │                                              ^               │
                 │                                              │ predicted     v
                 │                                           Tracker <──── 3D measurements
                 │                                              │
                 │                         Marker ID decoder <──┤ (per-track intensity series)
                 │                                              v
                 └──> Health/QA metrics ─────────────> State Publisher (ZeroMQ PUB) ──> UI / recorder
```

| Stage | Responsibility | Key outputs |
|---|---|---|
| **Ingest** | Validate packets, de-duplicate, reassemble split frames, detect loss from `seq`/`frame_id` gaps, undistort points using the current intrinsics | `CameraFrame` (undistorted normalised rays and metadata) |
| **TimeAlign** | Convert station time to system time (apply the measured residual offsets) and hold a short reorder buffer (≈ 2–3 frame periods) so late Wi-Fi packets are still used | Time-ordered frame stream |
| **Frame Grouping** | *Sync mode:* group frames sharing a trigger epoch. *Free-run mode:* define engine epochs at 120 Hz and, for each epoch, take each camera's nearest frame with its time offset | `FrameSet` |
| **Association** | *Tracked markers:* predict the 3D position at each camera's timestamp, project it, and gate 2D detections with a Mahalanobis distance, solved by Hungarian/GNN. *Untracked detections:* epipolar matching across camera pairs, hypothesis triangulation, and consensus voting (RANSAC-style) | Matched observation sets |
| **Triangulation** | DLT/midpoint initialisation, then a Gauss-Newton refinement that minimises reprojection error. Outputs the position, its 3×3 covariance (propagated pixel and calibration noise), per-camera residuals, and an outlier-rejection flag | `Measurement3D` |
| **Tracker** | One filter per marker track, with track management (tentative → confirmed → coasting → deleted). Starts as a constant-velocity Kalman filter; later moves to constant acceleration or IMM. Handles out-of-order measurements by timestamp | `MarkerTrack` (state, covariance, quality) |
| **Asynchronous update** *(Phase 5+)* | An EKF that ingests **each camera's 2D observation individually at its own timestamp**, as a bearing update. This removes the need for simultaneous capture and degrades gracefully with imperfect sync | Same as Tracker |
| **Marker ID decoder** | Normalises each track's fused per-camera intensity series by its running mean, recovers the symbol clock, decodes the frame, checks CRC, and votes the ID onto the track | `marker_id`, confidence |
| **Rigid body** *(later)* | Groups 2+ marker tracks with known geometry into a drone pose using Kabsch/Umeyama with a pose filter | `DroneState` (position, attitude) |
| **QA** | Per camera: reprojection residuals and their drift, beacon residuals, FPS, loss, latency. Per track: covariance and number of cameras | `HealthReport` |

### 5.2 Concurrency

* **Phase 0–4: a single-threaded asyncio event loop** for ingest, plus one processing thread.
  This keeps the engine deterministic and easy to replay. Numerical kernels use NumPy;
  hot loops use Numba where profiling shows a need.
* Hot paths move to C++ through pybind11 **only after profiling shows it is necessary**.
  Candidates are association, triangulation batches and the per-camera EKF. Module interfaces
  are designed so each stage can be swapped independently.
* The engine never blocks on the UI. Its publisher uses ZeroMQ PUB with a high-water mark,
  so slow subscribers drop messages rather than stall the engine.

### 5.3 Engine throughput target

The worst case is 50 drones × 3 markers × up to ~25 cameras seeing each marker × 120 Hz.
That is about 450k 2D observations/s into association and 18k triangulations/s.

* NumPy-vectorised Python handles this only if per-observation work stays O(1) with
  prediction gating.
* The Phase 4 benchmark measures it with `idps-sim` at full scale. If it misses the
  120 Hz budget, association and triangulation get a C++ port.

---

## 6. Calibration subsystem (`idps-calib`)

| Step | Method | Output and quality metric |
|---|---|---|
| **6.1 Intrinsics** (per camera, once, in the lab) | ChArUco board viewed through the IR filter: backlit IR board, or filter removed with a fixed focus. Lens focus and aperture are locked with thread-locker before calibrating | `K`, distortion, RMS reprojection error (target < 0.1 px), coverage of the image corners |
| **6.2 Extrinsics: wand** (initial, after installation) | A rigid wand with 3 collinear coded LEDs at known spacing is moved through the volume. Pairwise essential matrices give the initial guess; sparse bundle adjustment (`scipy.optimize.least_squares` with Jacobian sparsity, later Ceres) jointly refines all poses, optionally intrinsics, and wand points. Scale comes from the wand length | `T_CW` for all cameras with covariance, residual statistics, wand-length error distribution |
| **6.3 World frame** | Fixed reference beacons at **surveyed** positions (total station) or an L-frame on the floor. A similarity transform aligns the bundle solution to the survey | Alignment residuals per beacon |
| **6.4 Re-pose after a gimbal move** | The station must see ≥ 4 well-spread fixed beacons. PnP (EPnP + LM refinement) gives the new pose; it is cross-checked against the gimbal's commanded angles and refined with live marker co-observations | Pose covariance; the station returns to service only if the beacon residual < threshold |
| **6.5 Continuous monitoring** | Beacons are always visible and re-measured every second. Drift in their reprojection residual raises a per-camera alarm. Online refinement of extrinsics from tracked markers runs in the background and is *never applied automatically* without passing validation | Drift plot per camera in the UI |

At hall scale (100 m), waving a wand by hand through the whole volume is impractical. The plan
is to fly a **drone-carried wand**, together with a dense set of surveyed beacons (see RISKS R9).

**Calibration as data:** every calibration is an immutable, versioned file
(`calib/<timestamp>-<hash>.json`). It carries its inputs, method, quality report and parent
version. Recordings reference the calibration version they were captured with.

---

## 7. Coverage and accuracy simulator (`idps.coverage`)

The simulator shares the camera model and noise model with the engine.

* **Inputs:**
  * Volume geometry, obstacles and occluders (simple meshes or boxes).
  * Station positions, lens and sensor parameters, gimbal limits.
  * Noise assumptions: centroid σ, calibration σ, timing σ, marker speed.
* **Per voxel outputs:**
  * Number of cameras seeing the voxel, within range and not occluded.
  * Best and median ray-intersection angles.
  * **Predicted 3D covariance**, from the Fisher information of the visible cameras (the same
    maths as `tools/feasibility/error_budget.py`).
  * Minimum marker image size in pixels, and expected SNR from a radiometric model.
* **Optimisation:**
  * Greedy or CMA-ES search over candidate mounting points and aim angles.
  * Objective: maximise the fraction of volume with p95 error ≤ target and ≥ k cameras,
    subject to station count and cost.
* **Outputs:** heatmaps (UI and HTML report), plus a CSV of per-station contributions that
  answers "what does adding or moving this station buy us?".

---

## 8. Live dashboard (`idps-ui`)

* **Technology:** PySide6 (Qt 6) with `pyqtgraph.opengl` for the 3D view, running as a
  separate process.
* **Data flow:**
  * The UI subscribes to engine state over ZeroMQ, decimated to the 30–60 Hz render rate.
  * It sends commands (gimbal moves, exposure changes, recording start/stop, calibration
    jobs) over a ZeroMQ REQ/REP control API.
  * The UI is never in the data path, so a UI crash cannot affect tracking.
* **Panels:**
  * *3D view:* volume, drones with IDs, trails, uncertainty ellipsoids, camera frusta, and a
    coverage overlay loaded from the simulator.
  * *Camera table:* FPS, drops, packet loss, latency, PTP offset, temperature, calibration
    status and age, beacon residual.
  * *Track table:* ID, position, velocity, σ, number of cameras, ID confidence, age.
  * *Calibration:* wand capture wizard, re-pose, validation report viewer.
  * *Gimbal control:* jog or go-to, with an auto-recalibrate button.
  * *Snapshot viewer:* raw image with detected blobs overlaid.
  * *Recording and playback:* timeline scrubber.
* **Why not Open3D:** its GUI is a poor fit for dense widget dashboards. A web UI (three.js
  over WebSocket) is a reasonable later alternative, because the engine API is
  transport-neutral.

---

## 9. Recording and replay (`idps-record`, `idps-replay`)

* **Format:** [MCAP](https://mcap.dev). It is an open, indexed, append-only, timestamped,
  multi-channel container with C++ and Python libraries, and it is viewable in Foxglove.
* **Channels:**
  * `raw/udp/<station>`: every received packet byte-for-byte, with its arrival time.
  * `calib/version`
  * `health/<station>`
  * `engine/tracks`
  * `engine/measurements`
  * `events` (operator annotations, gimbal moves).
* **Replay:** feeds the `raw/udp` channels back into the engine's ingest stage at 1× or as
  fast as possible. The engine output is then diffed against the recorded `engine/*`
  channels, which serves as a regression test.
* **Size estimate:** 100 cameras × 120 Hz × ~200 B ≈ 2.4 MB/s ≈ 8.6 GB/hour (uncompressed;
  zstd chunk compression helps).

---

## 10. Technology stack evaluation

| Concern | Options considered | Recommendation | Rationale |
|---|---|---|---|
| Edge capture and detection | Python + picamera2; C++ + libcamera; V4L2 direct | **C++17 + libcamera** (V4L2 fallback) | Python cannot guarantee 120 FPS with a sub-8 ms budget on a Cortex-A72. libcamera exposes `SensorTimestamp` and raw modes. |
| Edge vision library | OpenCV; custom | **Custom NEON detector**, with OpenCV for tests only | A sparse blob detector is about 300 lines and much faster than full-frame OpenCV `connectedComponents`. |
| Edge control agent | C++; Python | **Python** | Not real-time, and much faster to develop. |
| Central engine | Python; C++; Rust | **Python 3.12 + NumPy/SciPy/OpenCV/Numba**, with C++ hot paths via pybind11 if profiling requires | Fastest iteration on the algorithms that need the most experimentation. Strict module interfaces keep later porting cheap. |
| Data-plane transport | UDP; TCP; ZeroMQ; DDS/ROS 2 | **Raw UDP** with a custom binary format | Lowest latency, no head-of-line blocking, trivial to implement on the Pi, and loss is measurable. |
| Control and UI IPC | gRPC; ZeroMQ; HTTP/REST | **ZeroMQ** (REQ/REP + PUB/SUB, msgpack payloads) | Lightweight, native in both C++ and Python, and handles slow subscribers well. |
| Time sync | NTP/chrony; PTP (linuxptp); hardware trigger; radio sync | **PTP over Ethernet** as the baseline, **with a hardware or radio trigger** evaluated in Phase 5 | Wi-Fi time sync is ms-class jitter, which costs mm–cm of error at 4 m/s (RISKS R2). |
| Recording | Custom binary; HDF5; MCAP; rosbag | **MCAP** | Indexed, streaming-safe, cross-language, existing tooling. |
| 3D UI | PyQtGraph; Open3D; VisPy; three.js | **PySide6 + pyqtgraph.opengl** | Mature desktop widgets with an adequate 3D scene for hundreds of objects. |
| Calibration optimisation | OpenCV; SciPy; Ceres; GTSAM | **SciPy** sparse least squares first, **Ceres** if scale needs it | Starts with zero build friction. Ceres is the standard for large bundle adjustment. |
| Configuration | YAML; TOML; JSON | **YAML** (system config), **JSON** (calibration artefacts) | Readable configuration plus machine-written, hashable calibration files. |
| Build and CI | — | CMake (edge), `uv`/`pyproject` (Python), GitHub Actions running unit tests and a simulator-in-the-loop smoke test | |

ROS 2 was considered and rejected for the core: DDS adds latency and deployment weight on a
1 GB Pi, and nothing in IDPS needs its ecosystem. A ROS 2 *bridge* that publishes engine
output can be added later for robotics consumers.

---

## 11. Repository layout (proposed)

```text
IPS/
├── docs/                    Architecture, protocol, risks, validation, roadmap
├── protocol/                Wire format: packet definitions, constants, golden test vectors
├── edge/                    C++ camera station (CMake)
│   ├── src/{capture,detect,net,timebase,params}/
│   ├── bench/               Throughput, latency, jitter, thermal benchmarks
│   └── tests/
├── agent/                   Python station agent (control API, gimbal, health)
├── idps/                    Python package (central)
│   ├── protocol/            Pack/unpack, mirrors protocol/ definitions
│   ├── core/                Types, camera model, geometry, time base
│   ├── engine/              ingest, timealign, grouping, assoc, triangulate, tracking, markerid, qa
│   ├── calib/               intrinsics, wand BA, PnP re-pose, world alignment, monitoring
│   ├── coverage/            Coverage and uncertainty simulator, layout optimiser
│   ├── sim/                 Virtual stations, drone trajectories, LED modulation, noise
│   ├── record/              MCAP record and replay
│   └── api/                 ZeroMQ state publisher and control server
├── ui/                      PySide6 dashboard
├── tools/feasibility/       One-off analysis scripts (error budget, radiometry)
└── tests/                   Python unit + integration (sim-in-the-loop) tests
```

---

## 12. Performance requirements

Values marked † are targets to be *verified* by benchmark, not assumed capabilities.

| ID | Requirement | Target | Verified in |
|---|---|---|---|
| P1 | Station sustained frame rate, no backlog | ≥ 120 FPS for 30 min, drops < 0.01 % † | Phase 2 |
| P2 | Station detection time per frame | p99 ≤ 4 ms at ≤ 50 blobs † | Phase 2 |
| P3 | Exposure-to-packet-sent latency | p99 ≤ 6 ms † | Phase 2 |
| P4 | Packet loss, wired | < 0.01 % | Phase 3 |
| P5 | Packet loss, Wi-Fi, 3 stations | < 1 %, measured and reported | Phase 3 |
| P6 | Timestamp accuracy (mid-exposure, system time base) | σ ≤ 100 µs (≤ 0.4 mm at 4 m/s); stretch ≤ 20 µs | Phase 5 |
| P7 | Engine update rate | 120 Hz with no backlog at the design load (sim) † | Phase 4 / 10 |
| P8 | Exposure-to-position latency (engine output) | p99 ≤ 20 ms wired, ≤ 50 ms Wi-Fi | Phase 7 |
| P9 | UI render rate | 30–60 FPS while the engine meets P7 | Phase 7 |
| P10 | Centroid precision, static marker, high SNR | ≤ 0.05 px RMS † | Phase 1 |
| P11 | Intrinsic calibration RMS reprojection | ≤ 0.1 px | Phase 4 |
| P12 | Extrinsic rotation uncertainty after calibration | ≤ 0.05 mrad (hall scale) / ≤ 0.2 mrad (lab) | Phase 6 / 8 |
| P13 | Marker ID acquisition time | ≤ 1.5 s free-running LEDs; ≤ 0.3 s synchronised LEDs | Phase 4 / 10 |
| P14 | Marker ID error rate (wrong ID asserted) | < 1e-4 per track-acquisition | Phase 10 |
| P15 | Positioning accuracy | p95 3D error ≤ 10 mm, *as defined in VALIDATION.md* | Phase 8 |
