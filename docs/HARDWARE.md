# Hardware (decision D16)

No hardware has been bought yet. Buy **one complete station first**, run the Phase 2 benchmark
(RISKS R4: full-resolution detection at 120 FPS, hardware PTP timestamps), then order the rest.

## Per camera station

| Item | Choice | Notes |
|---|---|---|
| Computer | **Raspberry Pi 5, 2 GB** | Detection needs CPU, not memory. 4 GB only if you want headroom for experiments |
| Cooling | Official Pi 5 Active Cooler | Needed at sustained 120 FPS; without it the Pi 5 throttles |
| Camera | **OV9281 global-shutter module** (1280×800, 120 FPS, mono) | Choose a module that exposes the **external trigger (FSIN) pin** (RISKS R13). M12 lens mount |
| Lens | M12, 2.8 mm (or as the planner chooses per camera) | The planner's lens options are set per venue |
| IR filter | 850 nm band-pass on the lens | Passes the drone LEDs, blocks visible light |
| Camera cable | **22-pin to 15-pin camera cable**, 30–50 cm | The Pi 5 uses the smaller 22-pin connector; most OV9281 modules have 15-pin. The length lets the Pi stay fixed while only the camera moves on the gimbal (RISKS R3/R4) |
| Gimbal | Pan + tilt servos and bracket (decision D6) | Servo driver board, e.g. PCA9685 (ROADMAP phase 6) |
| Power + network | **One PoE+ cable**: a Pi 5 PoE+ HAT (802.3at), or a PoE+ splitter with regulators (below) | One cable up the truss or mast carries data and power |
| Storage | microSD 32 GB, A2 class | Read-mostly; the station software logs to the PC |
| Enclosure | Ventilated, with a mount for the Pi beside the gimbal | Keep the cooler's airflow open |

## Shared

| Item | Choice | Notes |
|---|---|---|
| Switch | **Gigabit PoE+ (802.3at) switch** with one port per station plus uplinks | PoE budget below. Managed is nicer (port power cycling), not required |
| PTP grandmaster | Camera 1's Pi 5 (or a spare Pi 5) | Runs `ptp4l -H` as master (docs/NETWORK_TIMING.md). macOS cannot run `ptp4l`, so the Mac is not the master |
| PC | The Mac (or any PC) with gigabit Ethernet | Runs the positioning engine and dashboard |
| Cables | Cat6, up to 100 m per run | Enough for a 50 m venue with trusses or 20 m masts |
| Calibration | Wand + 1 m L-square with IR LEDs (WORKFLOW, decision D12) | |

## Power budget per station

| Load | Typical | Peak |
|---|---|---|
| Pi 5 at sustained detection (4 cores partly busy) | 6–8 W | 10 W |
| Active cooler | 0.5 W | 1 W |
| OV9281 module | 0.4 W | 0.5 W |
| Two pan/tilt servos | 1–2 W holding | 10 W while moving |
| Conversion losses (~10 %) | 1 W | 2 W |
| **Station total** | **about 10–12 W** | **about 20 W** |

* **Per port.** PoE+ delivers up to 25.5 W at the device, so the peak fits.
* **Not plain PoE.** 802.3af's 12.95 W is too little once the servos move.
* **Switch budget.** Allow 15 W per station on average:

  | Stations | Switch PoE budget |
  |---|---|
  | 4 (prototype, decision D3) | ≥ 60 W |
  | 16 | ≥ 240 W |
  | 24 | ≥ 360 W |

  Check the switch's *total* PoE budget, not just its per-port rating.

**Keep the servos off the Pi's 5 V rail.** A moving or stalling servo draws spikes that can
brown out the Pi. The cleanest single-cable setup is a **PoE+ splitter with a 12 V output**
feeding two regulators:

* 5 V / 5 A for the Pi 5, through its USB-C input;
* 6 V / 3 A for the servos, through the servo driver.

A Pi 5 PoE+ HAT also works if the servos get their own regulator rather than the HAT's 5 V
output. Check the HAT's current rating (5 V at 4.5 A or more).

## What to verify on the first station (Phase 2)

* **Detection:** full-resolution detection at 120 FPS with p99 processing ≤ 6 ms over 30 minutes,
  cooler running, inside the enclosure.
* **Hardware PTP:** `ethtool -T eth0` lists hardware timestamping, and `ptp4l -H` locks to the
  grandmaster.
* **Camera:** the OV9281 modes and `SensorTimestamp` behaviour, and whether the FSIN trigger
  input works.
* **Power:** peak draw with both servos moving, measured at the PoE input.
