# IDPS: Indoor Drone Positioning System

A low-cost, modular optical positioning system for indoor drones. It is built from
Raspberry Pi 5 camera stations (one per OV9281 global-shutter camera, powered over Ethernet), coded active 850 nm IR
markers, and a central PC engine with a live 3D dashboard.

**Status:** Phase 0 (no hardware needed). Done: protocol, planner + desktop dashboard, simulator, show pipeline
and PX4 module, positioning engine (C++ core), and the closed loop: simulated cameras and the engine flying PX4
simulator drones through a show.

| Document | Contents |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | System architecture, module design, technology stack evaluation, performance requirements, repository layout |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | Binary UDP observation format, health/control/engine APIs, time sync modes, optical marker ID code |
| [docs/RISKS.md](docs/RISKS.md) | Error-budget findings, critical risk register, decisions needed from the owners |
| [docs/VALIDATION.md](docs/VALIDATION.md) | Accuracy metric definition, ground-truth methods, test catalogue, flight gate |
| [docs/WORKFLOW.md](docs/WORKFLOW.md) | Operator workflow: planner → install → one-click aim → wand + calibration square → fly |
| [docs/ENGINE.md](docs/ENGINE.md) | The positioning engine: camera dots → matched, triangulated, tracked and identified drones → each drone's position; simulator results |
| [docs/SHOW.md](docs/SHOW.md) | Show design in Blender (Skybrush Studio) → `idps-show` check and per-drone export → upload to each drone → synchronised start |
| [px4/README.md](px4/README.md) | The `idps_show` PX4 module: each drone flies its own file from its SD card; build, set-up and simulator test |
| [docs/HARDWARE.md](docs/HARDWARE.md) | Station parts list (Pi 5 + OV9281), PoE+ power budget, what to verify on the first station |
| [docs/NETWORK_TIMING.md](docs/NETWORK_TIMING.md) | Pi-per-camera network, clock synchronisation options (simulated), ptp4l setup |
| [docs/ADVISOR.md](docs/ADVISOR.md) | Health monitoring and guided fixes (re-aim / move / add camera) from planning to show |
| [docs/ROADMAP.md](docs/ROADMAP.md) | Phase 0–10 plan with measurable exit criteria |

## Desktop dashboard (macOS-first)

The **Planner** tab is the first part of the IDPS dashboard. With it you can:
* draw the venue's floor outline (click the vertices) and set the height to extrude to;
* draw trusses, tripod areas and no-go areas;
* add a "tripod strip around the volume";
* press **Plan Cameras (⌘R)**.

The plan shows each camera's mount and aim, coverage maps (cameras per point or predicted
accuracy at each altitude), the install table, and export to `plan.yaml`, `cameras.csv` and
`report.html`.

```bash
# On a Mac (Apple Silicon or Intel), Python 3.11+:
python3 -m venv .venv && source .venv/bin/activate
pip install -e '.[ui]'
idps-dashboard                                   # or: idps-dashboard examples/sites/venue_50x20x20.yaml
bash packaging/macos/build_app.sh                # optional: build a double-clickable dist/IDPS.app
```

**macOS behaviour:**
* native menu bar with standard ⌘ shortcuts: ⌘N new, ⌘O open, ⌘S save, ⌘Z / ⇧⌘Z undo/redo,
  ⌘R plan, ⌘E export, ⌘0 fit;
* the edited-dot and document proxy icon in the title bar, and save prompts as sheets;
* light/dark mode follows the system;
* Retina rendering.

**Advisor tab:**
* shows system health and the most critical camera;
* **Find fixes** ranks re-aim / move / add-camera changes by problem points removed, each with
  tape-measure install text, **Show** and **Apply**;
* **Auto-fix plan** gives an ordered to-do list that reaches the requirement.

**Simulate tab:** after placing the cameras, check the layout before installing anything.
* **Run simulation** films drones with exactly the planned cameras (pixels, wire packets, PTP
  clocks, packet loss, and a calibration error you choose; the default is the 0.05 mrad
  target). Marker brightness is matched to the venue's camera distances. The real positioning
  engine then works out where each drone is, and the result is scored against the truth.
* A 20 s run with 12 drones takes about 12 s for the lab (9 cameras) and about 19 s for the
  50 × 20 × 20 m venue (17 cameras). Worst drone p95: 3.8 mm and 5.0 mm.
* Drones: a formation, random paths, or **your show file** (`.skyc` or a CSV zip from
  Blender). Show drones stand on their spots for 5 s first so the engine can name them.
* Results:
  * the worst drone's p95 against the requirement, how many drones were identified (and any
    wrong IDs), and how much of the time they were tracked;
  * a table per drone;
  * a 3D replay with each marker coloured by its error;
  * picking a drone shows the lines of sight from every camera that has it in frame, and that
    camera's 1280 × 800 image with the blinking dots.

**Install measurements:** the **Dimensions** overlay (⌘D) and the **Measure** tool (M); export
adds `measurements.csv`.

**On the canvas:**
* two-finger scroll pans and pinch zooms; ⌘+scroll also zooms.
* Tool keys: V select, P volume outline, T truss, A tripod area, N no-go.
* ⇧ constrains to horizontal/vertical; ⌥ places a point without snapping.
* Return, double-click or clicking V0 closes an outline.
* ⌫ deletes the selection; double-clicking an edge inserts a vertex.
* The inspector's vertex table takes exact coordinates.

## Planner (`idps-plan`)

The first piece of software. It takes a site file and works out:
* how many cameras are needed;
* where each one is mounted (truss + distance along it, or tripod position and mast height);
* the pan/tilt each camera should point at.

It also predicts which parts of the volume have 3+ cameras and meet the accuracy target.

```bash
pip install -e '.[dev]'                       # numpy, pyyaml, matplotlib, PySide6, pytest
idps-plan optimise examples/sites/lab.yaml  -o out/lab
idps-plan optimise examples/sites/venue_50x20x20.yaml -o out/venue --verify-spacing 2
idps-plan optimise examples/sites/hall.yaml -o out/hall --verify-spacing 5
# What-if: edit out/hall/plan.yaml (move/remove a camera), then
idps-plan evaluate out/hall/plan.yaml -o out/hall-edited
```

**Site file contents** (see `examples/sites/*.yaml`):
* the flight volume, drawn as floor XY vertices and extruded in Z, so any venue shape works
  (50 × 20 × 20 m, L-shaped, …);
* trusses as lines with height, usable segments (gaps = missing truss) and extra trusses;
* tripod zones as a strip outside the drawn outline (optionally only along chosen edges) or
  as floor polygons, with a height range (up to 20 m);
* a design margin (e.g. ≥ 4 cameras, 7 mm) that the optimiser aims for;
* no-go zones;
* lens options;
* requirements (minimum cameras, p95 mm, error budget);
* candidate density.

**Outputs:**
* `plan.yaml`, the plan used later by the one-click aim step;
* `cameras.csv`, the install sheet;
* `report.html`:
  * coverage maps of 0/1/2/3+ cameras and predicted-accuracy maps for every altitude;
  * a plan view;
  * the camera table with base heading, pan, tilt and neighbour spacing;
  * which points fail if any one camera is lost.

Run the tests with `python3 -m pytest`.

## Simulator (`idps-sim`)

Virtual camera stations that speak the real wire protocol, so the positioning engine and
dashboard can be built and tested without hardware.

* **Cameras:** taken from a planner `plan.yaml`.
* **Drones:** hover, line, circle, figure-8, waypoint or random-swarm trajectories at a
  fixed yaw. Tilt follows acceleration, so marker lever-arm effects are real.
* **Markers:** coded LEDs.
* **Imperfections modelled:**
  * lens distortion;
  * brightness falling with distance, and saturation;
  * centroid noise;
  * occlusion by other drones and by the drone's own body;
  * merged blobs;
  * reflections and false blobs;
  * frame drops;
  * free-running versus PTP versus hardware-triggered timing, with clock offsets;
  * network loss and latency;
  * calibration error.

```bash
idps-sim run examples/scenarios/lab_three_drones.yaml -o out/sim            # fast, to files
idps-sim listen --port 47600 &                                              # check the stream
idps-sim run examples/scenarios/venue_swarm.yaml --dest 127.0.0.1:47600 --realtime
```

**Outputs:**
* `packets.idpsraw`: every packet with its send time, for replay;
* `truth.csv`: true drone reference point, marker position, tilt and LED level at 120 Hz;
* `calibration.yaml`: the true camera poses, and the *estimated* poses the engine is given
  (perturbed by the scenario's calibration error).

`examples/scenarios/lab_pi5_ethernet.yaml` models the chosen network: Pi 5 stations on a
gigabit switch with hardware PTP, the clocks simulated message by message (`clocks.csv`).
`lab_wifi.yaml` is the same lab on Wi-Fi with the binned sensor mode, for comparison.
`idps-sim clocks` compares clock-sync options (docs/NETWORK_TIMING.md).

## Positioning engine (`idps-engine`)

```
idps-engine replay out/sim/packets.idpsraw --calib out/sim/calibration.yaml --truth out/sim/truth.csv
idps-engine live --port 5600 --calib calibration.yaml --site venue.yaml --link 1=udpout:192.168.50.101:14550 ...
```

Matches every camera's dots to marker tracks, triangulates, filters and reads each marker's
blink code. Positions go out in NED, to each drone by its ID. The per-slot work runs in a C++
core (built by `pip install -e .`, with the Python version as reference and fallback). In
the simulator, 50 drones and 17 cameras: p95 ≤ 4.8 mm, all identified, no wrong IDs, 5×
real time, 22 ms delay (p99) from exposure to output (docs/ENGINE.md).

Closed loop with PX4's simulator: `idps-sim px4` films the SITL drones in real time, and the
engine's positions fly them through a show (px4/README.md).

The wire format is in `protocol/idps_protocol.h`; the Python mirror is `idps/protocol/`;
golden vectors are in `protocol/vectors/`.

## Key findings so far

These come from `tools/feasibility/error_budget.py`, a first-order model. They are not
measurements.

* **±1 cm is plausible in a lab-scale volume** (about 5×5×3 m with 3–4 cameras), *if* camera
  timestamps are accurate to about 0.1 ms. Wi-Fi-class timing (ms) alone pushes the p95 error
  to about 2 cm at 4 m/s.
* **Requirement (decided): within 1 cm throughout the entire 100×100×30 m volume.** In the
  model, the interior meets this with about 120 stations, but only under a *tight* error
  budget:

  | Error source | Required |
  |---|---|
  | Centroid noise | ≤ 0.05 px |
  | Camera orientation | ≤ 0.03–0.05 mrad |
  | Camera position | ≤ 2 mm |
  | Timestamp | ≤ 0.05 ms |

  The wall bands and upper corners still fail in every simple layout. They need dedicated
  close-range cameras, found by the Phase 0 optimiser. Expect about 150–260 stations. Full-
  volume feasibility is *not yet shown*; it is checked at gates in Phases 0, 6 and 9.
* **Mounting on trusses and tripods (decided).** This layout works better than walls:
  tripods just outside the volume, aimed upward, cover the upper edges and corners. A
  hand-made 148-camera layout meets the spec at 99 % of test points under the tight budget;
  the planner's optimiser must close the remaining corners.
* **Calibration (decided): OptiTrack-style wand + 1 m ground square at the origin.** At hall
  scale, a 1 m square alone leaves 30–100 mm error at 70 m from the origin. A few floor
  markers placed 40–90 m along the X and Y lines and measured with a laser distance meter
  fix this.
* **Servo pan/tilt gimbals (decided)** are workable only with **live re-pose**: every camera
  continuously measures its own rotation from about 9–16 fixed beacons, and a station that
  moves or jitters drops out automatically. Servo hold stability is measured in Phase 6.
* **Fixed yaw (decided):** drones keep their takeoff yaw (normally facing North) but still tilt.
* **Coordinates (decided):** North-East-Down from the calibration square's corner (the origin
  placed in the planner), as flight controllers use.
  A marker offset 5 cm from the drone's reference point causes about 13 mm of error at 15°
  tilt. The marker should sit at the reference point, or 2–3 markers should be fitted.
* **Ethernet with PoE and PTP** is recommended beyond the first few prototype stations.

Run the error budget:

```bash
python3 tools/feasibility/error_budget.py           # layout scenarios (requires numpy)
python3 tools/feasibility/error_budget.py --sweep   # full-volume requirement sweep
python3 tools/feasibility/error_budget.py --beacons # camera pose precision from beacons
python3 tools/feasibility/error_budget.py --trusses # truss + tripod layouts
```
