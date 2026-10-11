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
| [docs/ROADMAP.md](docs/ROADMAP.md) | Phase 0–10 plan with measurable exit criteria |

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
```
