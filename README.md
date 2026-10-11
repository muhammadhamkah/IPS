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
* **±1 cm at p95 over 100×100×30 m is not reached** by any simulated layout under realistic
  noise assumptions, even with about 90–130 stations. It also needs per-camera orientation held
  to about 0.05 mrad and sub-0.1 ms timing. Calibration stability, not resolution, dominates
  at range.
* **Motorised gimbals are coarse aiming tools only.** Every move must be followed by locking
  and beacon-based re-calibration.
* **Ethernet with PoE and PTP** is recommended beyond the first few prototype stations.

Run the error budget:

```bash
python3 tools/feasibility/error_budget.py    # requires numpy
```
