# IDPS Implementation Roadmap

This roadmap extends the brief's 10 phases with a **Phase 0** (simulator-first software). It
also front-loads the measurements that retire the highest risks (RISKS.md). Each phase has an
exit criterion that must be *measured*, normally as a test from VALIDATION.md.

| Phase | Objective | Hardware needed | Key deliverables | Exit criterion |
|---|---|---|---|---|
| **0** | Simulator-first core | PC only | `protocol` (C++ header + Python), golden vectors; `idps.core` camera model; `idps.sim` virtual stations emitting real UDP (including servo jitter and fixed-yaw drones); `idps.engine` ingest → triangulate → constant-velocity KF; MCAP record/replay; `idps.coverage` v1 + **`idps-plan` planner** (volume + truss lines with usable/missing segments and tripod zones up to 20 m → camera count, positions, pan/tilt, tape-measure instructions, ≥ 3-camera and accuracy maps, beacons; worst-case robust optimiser); minimal UI (3D points + station table); CI with sim-in-the-loop test | Sim with 4 virtual cameras: recovered positions match truth within the predicted covariance (NEES OK); replay is deterministic; **the optimiser finds a hall layout meeting p95 ≤ 10 mm in every voxel under the tight error budget, with station count and cost** (first full-volume feasibility gate) |
|  | *Status* | | **Done:** protocol (Python + C header + golden vectors), planner (`idps-plan`) + dashboard Planner tab with Advisor, simulator (`idps-sim`), live-monitoring brain (`idps/monitor`, Live tab on a simulated show). **Next:** engine ingest → triangulate → track, record/replay, Live tab | |
| **1** | Single-camera IR detection | 1 station, LEDs, 850 nm filter | `idps-edge` capture + detector (initially at any FPS); snapshot tool; LED modulator firmware (Mode A) on a small MCU | T1, T2; Mode A decoding of a single marker on the bench; centroid bias-versus-defocus characterised |
| **2** | Camera performance | 1 station, fan/heatsink, LED-strobe rig | Edge pipeline at 120 FPS with ring buffers; `edge/bench`; exposure-midpoint timestamp offset measured | T3: P1–P3 met on the first Pi 5 station before ordering the rest (docs/HARDWARE.md), or a documented decision (crop / 90 FPS) |
| **3** | Wireless communication | 1–3 stations, AP, switch | Binary UDP streaming; HEALTH; agent control API; Wi-Fi vs Ethernet characterisation; PTP set up on wired | T4; T5 on S0 vs S1; decision D4 confirmed by data |
| **4** | Multi-camera positioning | ≥ 3 (4 recommended) stations, wand, ChArUco board | Intrinsics tool; wand bundle adjustment; world-frame alignment; association + triangulation on live data; ID decoding on 3D tracks | T6 (lab, preliminary), T8, T11 (2–3 markers); measured error within 1.5× the sim prediction |
| **5** | Synchronisation | Sync master (PTP grandmaster and/or trigger generator) | S1 in production; S2 or S3 prototype; async per-camera EKF; G2 rotating-arm rig | T5 σ ≤ 100 µs; T7 at 4 m/s: dynamic error ≤ 1.3× static |
| **6** | Automated calibration | Servo pan/tilt gimbals (PCA9685), calibration wand, calibration square, ≥ 16 fixed beacons | Servo gimbal driver and kinematic model; **one-click aim to plan** (coarse + fine); wand (walkable layer) + calibration-square calibration; **continuous calibration** (drone-marker bundle adjustment, camera bootstrapping); **station restore** after a bump; beacon self-survey; **live rotation re-pose from beacons**; movement detection; as-built versus plan report; calibration versioning | T9, T10. If servo stability fails T10, the worm-gear/brake upgrade is evaluated |
| **7** | Live UI | — | Full dashboard (ARCHITECTURE §8); planner UI (maps, what-if, printable install plan); guided site workflow (WORKFLOW.md); calibration wizard; recording/playback UI; latency/FPS/loss panels | P8, P9 with 4 live stations; operator runbook |
| **8** | Accuracy testing | Total station (rental) or equivalent, G1–G3 rigs | Validation report generator; full T6/T7/T13 campaign in the lab volume | **±1 cm acceptance (VALIDATION §1) in the lab volume**; gate for flight work |
| **9** | Long-range testing | Large hall access, long-focal lenses, more beacons | Range curves; optics choice (defocus/diffuser); drone-carried wand pilot; full-volume zone test in a hall section, including a corner | T12; T6/T7 in every zone of the section; simulator recalibrated with measured noise. **Full-volume feasibility gate:** if the measured budget terms do not support ±1 cm everywhere, the owners get a quantified options report (more stations, better sensor/lenses, or a revised requirement) |
| **10** | Multi-drone scaling | Additional stations, multiple markers per drone, optional radio sync | Mode B synchronised LEDs; rigid-body tracking; zone sharding; C++ hot paths if profiling requires | Load test at design scale (sim) passes P7; hardware with ≥ 10 markers passes T11 and T6/T7 |

## Sequencing notes

* **Phases 0 and 1–2 run in parallel.** Phase 0 needs no hardware, so engine, UI and
  calibration code are well advanced before the first station is benchmarked.
* **Timing measurement (T5) starts in Phase 3, not Phase 5.** R2 is the cheapest high
  severity risk to quantify and affects the D4 network decision.
* **The coverage simulator is built in Phase 0.** Station purchasing and placement for
  Phases 4 and 9 should be driven by it, not by guesswork.
* **The servo gimbal is deferred to Phase 6, after fixed-camera accuracy is proven.** Phases
  4–5 use the same stations with the servos unpowered or clamped. This gives a clean
  baseline that the servo hold-stability test (T10) is then compared against.
* **Full-volume ±1 cm has three explicit feasibility gates:**
  * Phase 0: a layout must exist on paper.
  * Phases 1/5/6: each error-budget term must be measured at or below its allocation.
  * Phase 9: tests in the real hall must confirm the model.

## Phase 0 work breakdown (first coding milestone)

1. `protocol/`: packet definitions (`idps_protocol.h`), Python mirror, golden byte vectors,
   fuzz test.
2. `idps/core`: Camera (intrinsics + distortion + extrinsics), projection/undistortion,
   time-base helpers, YAML station config.
3. `idps/sim`:
   * trajectory generators (static, line, circle, random waypoints, swarm);
   * LED Mode A modulator;
   * camera noise model (centroid σ, pixel locking, dropout, false positives, reflections);
   * per-station clock offset and jitter;
   * UDP emitter.
4. `idps/engine`:
   * ingest;
   * free-run frame grouping;
   * epipolar association;
   * DLT + Gauss-Newton triangulation with covariance;
   * constant-velocity KF tracker;
   * Mode A decoder.
5. `idps/record`: MCAP writer and replay.
6. `idps/coverage`: grid evaluation, heatmaps, layout comparison (from
   `tools/feasibility/error_budget.py`).
7. `ui/`: 3D view of tracks and cameras, station health table.
8. CI: unit tests, plus a 10-second sim-in-the-loop test asserting NEES and accuracy bounds.

**Phase 0 definition of done:** `idps-sim --scenario lab4` combined with `idps-engine` and
`idps-ui` shows drones with correct IDs and uncertainty ellipsoids, and the automated test
reports the error statistics.
