# IDPS: Indoor Drone Positioning System

A low-cost, modular optical positioning system for indoor drones. It is built from
Raspberry Pi 4B camera stations with OV9281 global-shutter sensors, coded active 850 nm IR
markers, and a central PC engine with a live 3D dashboard.

**Status:** architecture proposal. No production code has been written yet.

| Document | Contents |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | System architecture, module design, technology stack evaluation, performance requirements, repository layout |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | Binary UDP observation format, health/control/engine APIs, time sync modes, optical marker ID code |
| [docs/RISKS.md](docs/RISKS.md) | Error-budget findings, critical risk register, decisions needed from the owners |
| [docs/VALIDATION.md](docs/VALIDATION.md) | Accuracy metric definition, ground-truth methods, test catalogue, flight gate |
| [docs/WORKFLOW.md](docs/WORKFLOW.md) | Operator workflow: planner → install → one-click aim → wand + calibration square → fly |
| [docs/ROADMAP.md](docs/ROADMAP.md) | Phase 0–10 plan with measurable exit criteria |

## Planner (`idps-plan`)

The first piece of software. It takes a site file and works out:
* how many cameras are needed;
* where each one is mounted (truss + distance along it, or tripod position and mast height);
* the pan/tilt each camera should point at.

It also predicts which parts of the volume have 3+ cameras and meet the accuracy target.

```bash
pip install -e '.[dev]'                       # numpy, pyyaml, matplotlib, pytest
idps-plan optimise examples/sites/lab.yaml  -o out/lab
idps-plan optimise examples/sites/hall.yaml -o out/hall --verify-spacing 5
# What-if: edit out/hall/plan.yaml (move/remove a camera), then
idps-plan evaluate out/hall/plan.yaml -o out/hall-edited
```

**Site file contents** (see `examples/sites/*.yaml`):
* the flight volume;
* trusses as lines with height, usable segments (gaps = missing truss) and extra trusses;
* tripod floor zones with a height range (up to 20 m);
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
* **Fixed yaw (decided):** drones keep their takeoff yaw (normally facing +X) but still tilt.
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
