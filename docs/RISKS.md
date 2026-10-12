# IDPS Critical Technical Risks

Status: **v0.1**. Each risk lists:

* the evidence behind it;
* the earliest test that retires or confirms it;
* the decision the result drives.

Severity scale: **H** = could make the ±1 cm / 100 m objective unreachable; **M** = needs
design work or budget; **L** = manageable.

---

## 0. Headline findings from the error budget

`tools/feasibility/error_budget.py` propagates per-camera angular noise through linearised
multi-view triangulation. Its error sources are centroid noise, extrinsic rotation and
position error, and timing error at 4 m/s. It is a **first-order, optimistic** model: no
occlusion, no marker-geometry error, and independent errors across cameras. "3D RMS" is
√trace(Σ). For roughly isotropic errors, the 95th-percentile radius ≈ 1.6 × 3D RMS, so
**±1 cm at 95 % requires 3D RMS ≲ 6 mm.**

| Scenario | Cameras | Median cams/point | 3D RMS median | 3D RMS p95 |
|---|---|---|---|---|
| Lab 5×5×3 m, 3.6 mm lens, 0.1 px / 0.1 mrad / 0.1 ms | 4 | 4 | **1.4 mm** | 3.0 mm |
| Same lab, Wi-Fi-class timing (σ = 2 ms) | 4 | 4 | 8.5 mm | 18.5 mm |
| Same lab, only 3 cameras | 3 | 3 | 1.6 mm | 2.7 mm (88 % coverage) |
| Hall 100×100×30 m, perimeter every 25 m, 2 heights, *realistic* | 32 | 6 | 11.9 mm | 33.9 mm |
| Hall, perimeter every 12.5 m, 3 heights, *realistic* | 96 | 22 | 5.5 mm | 17.0 mm |
| Hall, ceiling grid 20 m + 2 perimeter rings, *realistic* | 89 | 23 | 4.7 mm | 17.0 mm |
| Hall, best layout above, *optimistic* (0.05 px / 0.05 mrad / 0.05 ms) | 89–96 | 22 | 2.2–2.5 mm | 8.3 mm |
| Hall, any layout, *degraded* (0.2 px / 0.3 mrad / 1 ms) | 89–128 | 23 | 9–14 mm | 45 mm |

*Realistic* means 0.1 px centroid, 0.1 mrad rotation, 3 mm position and 0.1 ms timing.

**Conclusions:**

1. **±1 cm is plausible in a lab-scale volume** with 3–4 cameras, *provided timing is wired-
   class (≤ 0.1 ms)*. Wi-Fi-class timing alone breaks the budget.
2. **Calibration stability, not pixel count, is the dominant term at long range.** A
   0.1 mrad rotation error equals 7 mm at 70 m. Adding cameras dilutes it only if the
   cameras' errors are independent, and that remains unproven.

### 0.1 Full-volume requirement (decision D1: ±1 cm everywhere)

The owners have decided that accuracy must be **within 1 cm throughout the entire flight
volume**, with no zone grading. "Within 1 cm" is read as **p95 3D error ≤ 10 mm at every
location** (VALIDATION.md §1). For a roughly isotropic Gaussian error, that means
**3D RMS ≤ 6.2 mm at every point**, including corners, wall bands, near the floor and near
the ceiling.

`python3 tools/feasibility/error_budget.py --sweep` evaluates a 9×9×7 grid. The grid is inset
1 m from the walls and ceiling, runs from 0.2 m to 29 m altitude, and includes the corners.
The table shows the share of grid points that meet 6.2 mm:

| Layout | Cameras | Realistic (0.1 px, 0.1 mrad, 3 mm, 0.1 ms) | Tight (0.05 px, 0.05 mrad, 2 mm, 0.05 ms) | Very tight (0.05 px, 0.03 mrad, 1 mm, 0.02 ms) |
|---|---|---|---|---|
| Ceiling grid 20 m + perimeter 12.5 m × 3 rings | 121 | 63 % (worst 38 mm) | 77 % (19 mm) | 86 % (14.5 mm) |
| Ceiling grid 12.5 m + perimeter 12.5 m × 3 rings | 160 | 72 % | 82 % | 89 % |
| Ceiling grid 10 m + perimeter 10 m × 4 rings | 260 | 84 % (22 mm) | 94 % (11 mm) | 98 % (8.3 mm) |
| 121-camera layout + 40 ceiling cameras aimed at the walls | 161 | 81 % | 91 % | 95 % |

**What this means:**

* **The interior is achievable on paper.** It meets the spec everywhere with about 120 cameras
  under the *tight* budget. Here "interior" means more than 10 m from the walls and below 26 m.
* **Every failing point lies in the wall band, mostly in the upper corners.** Cameras there
  either cannot see those points at all, or see them only from about 140 m away across the
  hall. Fixing this needs **dedicated close-range cameras for the corners and wall bands**,
  not more cameras in general. That is a placement-optimisation problem for the Phase 0
  coverage optimiser (ARCHITECTURE §7). None of the simple hand-made layouts above solves it.
* **The per-camera error budget must be "tight" or better:**

  | Error source | Required |
  |---|---|
  | Centroid noise | ≤ 0.05 px |
  | Camera orientation known and held to | ≤ 0.03–0.05 mrad (6–10 arcsec) |
  | Camera position known to | ≤ 1–2 mm |
  | Mid-exposure timestamp | ≤ 0.02–0.05 ms |

  Each of these is at or beyond what low-cost systems normally achieve. Each one needs its own
  measured proof (Phases 1, 5, 6). Wi-Fi timing is ruled out for the final system.
* **Expected scale:** roughly **150–260 camera stations**, plus beacons, a survey and wired
  networking.
* **This model is optimistic.** It assumes no occlusion between drones, independent errors
  across cameras, and no marker-geometry error. Real margins will be smaller.

### 0.2 Truss + tripod mounting (decision D11)

There are no wall mounts. Cameras go on **overhead trusses** (modelled at 31.5 m, above the
30 m volume) and on **tripods** (modelled 4 m tall, 2 m outside the volume boundary).
`error_budget.py --trusses` evaluates these hand-made layouts:

| Layout | Cameras | ≥ 3 cameras | Meets spec: realistic | Meets spec: tight | Meets spec: very tight |
|---|---|---|---|---|---|
| A: truss grid 20 m + perimeter tripods 12.5 m | 57 | 91.5 % | 65 % | 83 % | 85 % |
| B: A + cameras on the boundary trusses looking down the edges | 89 | 95.1 % | 78 % | 89 % | 91 % |
| C: B + 3 tripod cameras per corner aimed up into the corner | 101 | 98.2 % | 85 % | 98 % | 99 % (worst 7.5 mm) |
| D: truss grid 12.5 m + tripods 10 m + edge + corners | 148 | **100 %** | 92 % | **99 %** (worst 11.5 mm) | **99.5 %** (worst 7.5 mm) |

**Trusses plus tripods work better than the earlier wall layouts.** Tripods just outside the
volume, aimed upward, see the upper edges and corners from about 25–35 m. Wall cameras could
not see the region directly above them. With about 150 cameras and the *tight* budget, the
remaining failures are a handful of corner points. The planner's optimiser should close these,
rather than hand placement. The "realistic" budget still fails about 8 % of the volume, so the
tight error budget remains essential.

### 0.3 Optimised layouts (`idps-plan`)

The hand-made layouts above were far from optimal. The planner's optimiser finds much smaller
layouts that meet the requirement *in the model*.

**Example hall (100 × 100 × 30 m, tight budget):**

| Optimiser target | Cameras | Meets 3 cameras / 10 mm (5 m check grid) | Worst point | Points that fail if one camera is lost |
|---|---|---|---|---|
| Exactly the requirement (3 cameras, 10 mm) | 18 | 99.95 % | 10.7 mm | up to 236 per camera |
| **Design margin: 4 cameras, 7 mm** | **28** | **100 %** | **8.4 mm** | **at most 2 per camera** |
| 4 cameras, 7 mm, sightlines limited to 60 m | 66–85 | 99.5–99.9 % | 16–70 mm | (optimised on too coarse a grid) |

* A 50 × 20 × 20 m venue needs about 16 cameras with the design margin.
* An L-shaped 40 × 30 × 15 m tripod-only venue needs about 10.

**Caveats:**

* These small counts rely on wide lenses (2.8 mm) and **sightlines up to 100 m+**. That is
  where the model is most optimistic: centroid precision at long range is assumed, not
  measured (R7), and there is no occlusion model.
* The planner therefore always reports single-camera-loss robustness, and the examples use a
  **design margin** (≥ 4 cameras, 7 mm).
* Phase 1/9 measurements of centroid precision versus range must be fed back into the noise
  model before a venue's camera count is trusted.

**Status: ±1 cm across the full 100×100×30 m volume is not yet shown to be feasible.** It is
not ruled out by geometry either. The plan keeps it as the requirement, and checks it at
explicit gates:

* **Phase 0:** the optimiser must find a layout that meets the spec under the tight budget.
  It now does, in the model (§0.3); what remains is validating the model's long-range
  assumptions.
* **Phases 1/5/6:** each budget term must be measured at or below its allocation.
* **Phase 9:** hall-scale tests must confirm the model.

If any gate fails, the owners get a quantified decision: more stations, a better sensor or
lenses, or a revised requirement.

---

## 1. Risk register

### R1: Extrinsic calibration accuracy and stability at range (H)

* **Evidence:** §0. Each camera's orientation must be known and *held* to ~0.05–0.1 mrad.
  Camera position must be known to a few mm.
* **Contributors:**
  * thermal expansion of poles and trusses (an aluminium 10 m pole changes length by
    0.23 mm/°C);
  * building vibration;
  * mount creep;
  * lens focus shift with temperature.
* **Mitigation:**
  * rigid mounts;
  * lenses locked with adhesive;
  * continuous beacon monitoring (ARCHITECTURE §6.5);
  * online drift alarms;
  * periodic bundle re-calibration;
  * temperature logging per station.
* **Earliest test:** Phase 6. Leave a calibrated station observing fixed beacons for 24 h and
  plot beacon reprojection drift against temperature. **Pass:** drift ≤ 0.05 mrad over the
  operating temperature range.

### R2: Time synchronisation over Wi-Fi (H)

* **Evidence:**
  * At 4 m/s, a 1 ms timing error equals 4 mm of motion.
  * In the lab simulation, 2 ms timing σ raises the median error from 1.4 mm to 8.5 mm and
    p95 to 18.5 mm.
  * Wi-Fi software timestamping commonly shows 0.1–5 ms jitter, with power-save and
    retransmission spikes.
* **Mitigation:**
  * Ethernet + PTP as the baseline (S1);
  * a hardware or radio trigger (S2/S3);
  * an asynchronous per-camera EKF, which tolerates unaligned capture *if timestamps are
    accurate*;
  * acquisition timestamps (never arrival times);
  * the Wi-Fi reorder buffer.
* **Simulation** (`idps.sim.clocksync`, docs/NETWORK_TIMING.md):
  * Wi-Fi with chrony: about 2 ms skew (p95), about 5 mm at 5 m/s.
  * Wired software PTP: about 12 µs (Pi 5), 60 µs (Pi 3B).
  * Wired hardware PTP on the Pi 5 (the chosen setup): under 1 µs.
* **Earliest test:** Phase 3/5. The LED-strobe timing rig measures per-station timestamp
  error under S0, S1 and S2. **Decision:** Wi-Fi remains a development convenience only,
  unless it achieves σ ≤ 100 µs in that test.

### R3: Servo pan/tilt gimbals: holding the camera still (H)

**Decision D6:** the owners have chosen hobby-class **servo motors for pan and tilt**. The
architecture supports this, with the constraints below.

* **Evidence:**
  * Typical hobby servos have about 0.2–1° deadband and backlash (3–17 mrad).
  * They have no position read-back.
  * They can hunt (small oscillations) under load or vibration.
  * The full-volume budget requires the camera orientation known to ≤ 0.03–0.05 mrad. That
    is **100–500× finer** than the servo's own positioning.
* **Consequences:**
  1. **The commanded servo angle is never used as the camera pose.** It only seeds the
     beacon-based pose solve.
  2. **Every camera continuously measures its own orientation from fixed reference beacons
     (live re-pose).**
     * The camera position comes from the gimbal kinematic model plus the bundle adjustment.
     * The 3 rotation angles are re-estimated continuously.
     * The engine uses time-varying extrinsics (ARCHITECTURE §6.4).
  3. **Beacon requirements**, from `error_budget.py --beacons` plus a rotation-only study:

     | Beacons in view | Beacon range | Averaging | Rotation 1σ |
     |---|---|---|---|
     | 4 | 20–60 m | 1 frame | 0.17 mrad |
     | 9 | 40–100 m | 0.1 s | 0.05 mrad |
     | 16 | 40–100 m | 0.1 s | 0.04 mrad |
     | 16 | 40–100 m | 1 s | 0.03 mrad |

     (Survey error 2 mm per beacon; camera position known.)

     To meet the budget, each camera needs **≥ 9–16 beacons spread across its view, mostly
     ≥ 40 m away**, surveyed to ≤ 2 mm and refined jointly in the bundle adjustment. It must
     also average over ≥ 0.1–1 s. **Live re-pose therefore works only while the servo is
     actually still.** Servo motion faster than the averaging window cannot be corrected.
  4. **Detect movement and stop trusting the camera:**
     * Beacon residuals are monitored every frame.
     * A rotation change above 0.03 mrad that is not explained by a commanded move marks the
       station `UNCALIBRATED` until re-pose converges again.
     * A camera that jitters therefore drops out rather than corrupting positions.
* **Servo-specific mitigations:**
  * Choose metal-gear digital servos with a small deadband, and drive them from a hardware
    PWM source (e.g. a PCA9685), never software PWM.
  * Mount the camera close to both rotation axes and balance it, to minimise holding torque.
  * Use a stiff bracket.
  * Measure the camera position offset against pan/tilt (the optical centre is not on the
    axes), so the kinematic model can predict the camera centre after each move.
  * **Measure whether holding torque or switching off PWM gives the stiller camera**
    (Phase 6). A friction clamp or brake remains the fallback.
* **Earliest test (Phase 6):**
  * (a) 30-minute beacon log with the servo powered and holding: image-space jitter
    spectrum, and fraction of time within 0.03 mrad.
  * (b) 50 move-return cycles: settle time, and re-pose accuracy against a full bundle
    adjustment.
  * **Pass:** stable within 0.03 mrad for ≥ 99 % of tracking time.
  * **If it fails:** upgrade the same gimbal design to worm-gear servos or add a brake. The
    software does not change.

### R4: Raspberry Pi 5 sustained 120 FPS (M)

**Station hardware (decision D16): Raspberry Pi 5 (2 GB) + OV9281 per camera**, chosen over the
Pi 3B before any hardware was bought. The reasons:

* **CPU.** The Pi 5's Cortex-A76 cores are roughly 2–3× a Pi 4B and far beyond a Pi 3B, so
  full-resolution detection is expected to fit.
* **Fewer cameras.** A Pi 3B would have needed the OV9281's binned 640×400 mode, which halves the
  angular resolution. Planned layouts with full resolution need about 4 fewer cameras:
  * a 30 × 16 × 9 m venue: 14 instead of 18;
  * the 50 × 20 × 20 m example: 17 instead of 21.
* **Network.** Native gigabit Ethernet with **hardware PTP timestamps**: simulated skew between
  stations is about 0.8 µs p95, against 60 µs for a Pi 3B (docs/NETWORK_TIMING.md).
* **Power.** A PoE+ HAT gives one cable per station (docs/HARDWARE.md). The Pi 3B has no PoE
  header.

**The 120 FPS evidence:**

* **Sensor and pixel rate.** The OV9281 supports 1280×800 at up to 120 FPS. A 1 MP frame at
  120 FPS is 123 Mpix/s.
* **Detector time.** One full-frame read at the Pi 5's memory bandwidth takes a fraction of a
  millisecond, so a sparse NEON detector should fit in 1–3 ms of the 8.33 ms budget.
* **Remaining risks:**
  * unpacking RAW10;
  * buffer-copy overhead;
  * **thermal throttling**: the Pi 5 needs its active cooler, especially in an enclosure;
  * interrupt latency affecting timestamps.

**Mitigation:**
* 8-bit raw mode if the driver offers it;
* zero-copy dmabuf access;
* core pinning;
* the active cooler;
* cropped or binned modes at long focal lengths.

**Mounting the Pi on the gimbal** adds moving mass, heat and cable stiffness to the servos (R3).
Put only the camera on the gimbal, with a 30–50 cm camera cable to a fixed Pi (the Pi 5 uses
the smaller 22-pin camera connector: docs/HARDWARE.md).

**Earliest test:** Phase 2 benchmark on the first Pi 5 + OV9281, *before ordering the rest*
(ARCHITECTURE §4.3). **Decision:** if p99 processing exceeds 6 ms, or drops exceed 0.01 % over
30 minutes, choose a cropped mode or 90 FPS. Also confirm `ethtool -T eth0` lists hardware
timestamping. If it does not, the stations run software PTP, which simulates to about 12 µs
and is still fine.

### R5: Network scale with 150–260 stations (M)

* **Evidence:**
  * Aggregate data is small (~0.35 Mbit/s per station at 20 markers, ~90 Mbit/s total for
    260 stations).
  * However, 260 stations × 120 packets/s = 31,200 small packets/s.
  * On shared Wi-Fi airtime, contention causes retries and latency spikes. Range across a
    100 m hall needs several APs.
* **Mitigation:**
  * **Ethernet with PoE** (powers the Pi through a PoE HAT, and enables PTP) for any
    deployment beyond the prototype;
  * Wi-Fi limited to ≤ 5–10 stations per AP, on the 5 GHz band, with power-save disabled.
* **Earliest test:** Phase 3, with a station emulator flooding the AP with N virtual stations
  from a few Pis or a PC.

### R6: Marker ID decoding robustness and latency (M)

* **Evidence:**
  * Free-running Manchester code at 30 chips/s needs ~1–2 s to acquire.
  * Failure causes: intensity changes from LED emission angle and drone attitude during the
    frame, partial occlusion, merged blobs, and clock drift.
  * Brightness ratio 2.5:1: the LOW state must remain detectable at maximum range.
* **Mitigation:**
  * Decode on 3D tracks, fusing all cameras' intensity series.
  * Normalise by the running mean.
  * Use a CRC and a minimum Hamming distance of 3.
  * Assert an ID only after 2 frames and re-verify continuously.
  * Mode B synchronised LEDs (0.23 s).
* **Earliest test:**
  * Phase 1 bench: decode success rate against range, angle and exposure.
  * Phase 4: ID swap rate in the simulator under crossing trajectories.

### R7: Optics and long-range detection (M)

* **Radiometry (encouraging):**
  * Setup: an LED at 0.5 W/sr, at 100 m, through a 4 mm f/1.4 lens with 1 ms exposure.
  * It delivers ~1.4 M photons, about 2.7×10⁵ e⁻ at 20 % QE (QE at 850 nm to be confirmed).
  * That is far above the noise floor; saturation is the bigger problem.
* **Real problems:**
  1. **Undersampling:**
     * At long range, a point source images to less than 1 px.
     * Centroiding then suffers "pixel locking" bias (errors up to ~0.1–0.2 px).
     * Fix: slight deliberate defocus or a diffuser, to keep the spot at about 3–5 px
       diameter.
  2. **LED emission pattern:** viewed off-axis, a narrow-beam LED dims sharply. Use wide-angle
     LEDs, several LEDs per marker, or a diffusing cap, and characterise the result.
  3. **Diffusing cap geometry:**
     * The centroid tracks the *emitting surface*, not the LED die.
     * The cap's shape biases the centroid with viewing angle (mm-level).
     * Use a small spherical diffuser and calibrate the marker centre.
  4. **Saturation:** exposure and threshold must be set per station. Detection reports
     saturation, and the engine down-weights saturated blobs.
* **Earliest test:** Phase 1 (centroid precision and bias versus defocus) and Phase 9 (25–30 m
  and beyond).

### R8: Ambient IR, reflections and ghost markers (M)

* **Sources of false or corrupted detections:**
  * sunlight through windows;
  * halogen and incandescent lighting;
  * specular reflections of markers off floors, walls and drone bodies (ghost markers);
  * other cameras' IR illuminators (if any).
* **Mitigation:**
  * an 850 nm bandpass filter (strongly recommended, not optional);
  * short exposure (≤ 0.5 ms);
  * static-mask learning per camera (stationary blobs are masked or classified as beacons);
  * multi-view consistency to reject reflections (a ghost does not triangulate consistently);
  * matte surfaces near the volume where possible.
* **Earliest test:** Phase 1 in the actual room; Phase 4 with multi-view rejection.

### R9: Calibration logistics at 100 m scale (H)

* **Evidence:**
  * A hand-held wand covers only the walkable layer (about 0–2.5 m) of a 30 m volume. That is
    the normal OptiTrack practice and is enough for cameras that see that layer.
  * In the truss + tripod layout, 32 of 148 cameras (upward-aimed tripods) never see the
    walkable layer.
  * Accuracy in the upper volume depends on calibration quality there, and wanding alone
    does not measure it.
* **Mitigation (decision D13):**
  * wanding the walkable layer;
  * a **continuous-calibration flight** that covers the whole volume, plus continuous
    calibration during operation (ARCHITECTURE §6.6). Two markers at a known spacing on the
    calibration drone add a scale check at altitude;
  * tilt-down wanding as a fallback for cameras that cannot see the walkable layer;
  * a planner constraint: every camera must either see the walkable layer or co-observe
    the calibration-flight path with ≥ 2 wand-calibrated cameras;
  * a dense network of surveyed fixed beacons (also used for R1 monitoring);
  * staged calibration (zones, then a global bundle adjustment);
  * Ceres for large problems.
* **Earliest test:** Phase 6 (lab wand), then a Phase 9 hall pilot.

### R10: Independent ground truth for validating ±1 cm (H)

* **Evidence:**
  * Validating ±1 cm needs a reference about 3–5× better, i.e. ≤ 2–3 mm.
  * At hall scale that means a total station (static) or a laser tracker (dynamic, costly).
* **Mitigation (low-cost methods):**
  * a known-length wand (scale consistency);
  * a precision linear rail with encoder;
  * a **rotating arm** with a marker at a known radius (dynamic: 4 m/s at r = 0.5 m needs
    76 rpm; circle-fit residuals give the error);
  * surveyed static points.
* **Status:** see VALIDATION.md. **Decision D2:** budget or rental for a total station.

### R11: Occlusion and merged blobs with 50 drones (M)

* **Evidence:**
  * In dense formations, markers from different drones overlap along rays (merged or
    elongated blobs).
  * Drone bodies occlude markers.
  * The combinatorics of association grow.
* **Mitigation:**
  * the `elongated` flag;
  * prediction-gated association;
  * multiple markers per drone (rigid-body fallback);
  * camera counts large enough that ≥ 2 unoccluded views remain;
  * simulator scenarios with swarms.
* **Earliest test:** Phase 4 simulator, Phase 10 hardware.

### R12: Budget versus required station count (H, programmatic)

* **Evidence:**
  * S$200 covers roughly **one** camera station plus markers. Exact local prices to be
    confirmed.
  * Phase 4 requires ≥ 3 stations; ≥ 4 is recommended.
  * The full-volume ±1 cm requirement (§0.1) points to roughly 150–260 stations, plus PoE
    switching, mounts, servo gimbals, beacons and a survey.
* **Mitigation:**
  * Phases 1–3 need only one station.
  * Use the coverage simulator to cost layouts before buying.
  * Consider higher-resolution sensors for the hall: fewer stations, though more pixels per
    frame to process.
* **Decision D3:** prototype budget for 4 stations (Phase 4).

### R13: OV9281 / Raspberry Pi driver capabilities (M)

* **To verify on real hardware:**
  * a raw 8-bit or 10-bit mode at 120 FPS through libcamera on the Pi 5;
  * the camera cable: the Pi 5 has 22-pin camera connectors, and most OV9281 modules ship with a
    15-pin cable, so a 22-to-15-pin cable is needed;
  * hardware PTP timestamps on the Pi 5's Ethernet (`ethtool -T eth0`);
  * **external trigger (FSIN) support** in the kernel driver and module (some vendor modules
    expose a trigger pin, others do not);
  * accuracy and latency of `SensorTimestamp`;
  * lens mount (M12 is preferable for interchangeable lenses and filters).
* **Mitigation:** choose the module variant with an exposed trigger; keep a V4L2 fallback.
* **Earliest test:** Phase 1 / 2.

### R14: Eye safety of 850 nm emitters (M, safety)

* **Evidence:**
  * 850 nm is barely visible, so people do not blink or avert their eyes.
  * High-power IR LEDs at close range can exceed IEC 62471 exposure limits.
* **Mitigation:**
  * Classify the marker LED drive current under IEC 62471.
  * Use diffusers, and limit current to what the range budget needs (§R7 shows large margin).
  * Label equipment.
  * Never look into markers at close range.

### R15: Central engine performance in Python (L)

* **Evidence:** ARCHITECTURE §5.3 estimates ~450k observations/s at full scale.
* **Mitigation:**
  * vectorised NumPy;
  * Numba;
  * pybind11 C++ for association and triangulation;
  * engine stages can be sharded by spatial zone if needed.
* **Earliest test:** Phase 4 / 10 simulator load test.

### R16: Drone reference point versus marker position (M)

* **Context:** drones take off at a specified yaw (normally 0, i.e. facing North) and do not
  yaw in flight. They still **roll and pitch** to accelerate, typically 10–25° at 4 m/s
  manoeuvres.
* **Evidence:**
  * IDPS measures the *marker*, not the drone's centre.
  * A marker mounted a distance `d` from the drone reference point moves sideways by about
    `d·sin(tilt)` when the drone tilts.
  * With d = 5 cm at 15° tilt, that is **13 mm**, which by itself exceeds the 1 cm budget.
* **Mitigation:**
  * Mount the marker as close as possible to the reference point used by flight control, or
    declare the marker centre to *be* the reference point.
  * If the offset is unavoidable, measure roll and pitch optically with 2–3 markers.
    * Fixed yaw makes this easier: the pose has only roll, pitch and XYZ unknowns, and the
      marker constellation's orientation in the image is predictable.
    * Alternatively, flight control applies the lever arm using its own IMU attitude.
  * Either way, the reference point and offset are part of the drone configuration
    (ARCHITECTURE §3).
* **Earliest test:** Phase 4. Rotate a marker rig about a known pivot and verify that the
  reported reference point stays fixed.

### R18: World-frame axes and scale at hall scale (H)

* **Evidence (OptiTrack-style calibration):** the wand gives scale and the ground square
  gives origin and axes. That is ideal for rooms, but at 100 m the tools are short compared
  with the distances. Errors in axis angle and scale grow with distance from the origin.

  | Source | Error at 70 m from origin |
  |---|---|
  | 1 m square, each LED located to 0.3 mm (averaged) | axis tilt 0.42 mrad → **30 mm** |
  | 1 m square, LED to 1 mm | 1.4 mrad → **99 mm** |
  | 1 m wand length known to 0.1 mm (scale 1e-4) | **7 mm** |
  | Floor markers 50–90 m from origin, LED to 1 mm | 0.016–0.028 mrad → 1–2 mm |
  | Laser distance meter ±1 mm over 80 m (scale 1.3e-5) | 0.9 mm |

* **Does it matter?** A rotated or scaled world frame does not change distances *between*
  nearby drones much. It does move every absolute position relative to the real room: floor,
  landing pads, ceiling, obstacles. "Within 1 cm throughout the volume" refers to the real
  room, so it matters.
* **Mitigation:**
  * Keep the 1 m square for the origin.
  * Add long-baseline floor markers along North and East, measured with a laser distance meter,
    for axes, floor plane and scale (WORKFLOW step 4, item 3).
  * Verify with the independent total-station check (VALIDATION G4).
* **Earliest test:** Phase 9. Compare the scale and axes from the square plus wand against
  the long-baseline markers.

### R17: Installation-to-plan fidelity (M)

* **Evidence:**
  * Coarse aiming sends servo angles that are relative to the gimbal base. A base that is not
    level, or is rotated a few degrees from the planned direction, aims the camera equally
    far off.
  * Physical mounts are rarely exactly where planned (obstructions, truss positions).
  * **Tripods** can be bumped, settle on their legs, or be moved between sessions. Unlike a
    servo jitter, this changes the camera *position*, which rotation-only live re-pose does
    not correct. Movement detection flags it; recovery is a full 6-DOF re-pose from beacons
    (lower accuracy, ARCHITECTURE §6.4) or a wand re-calibration.
  * Accuracy itself is unaffected, because calibration measures the real pose. Coverage,
    however, can degrade.
* **Mitigation:**
  * The planner uses robust optimisation with per-camera install tolerances, and flags
    tight-tolerance cameras.
  * Install instructions are given relative to physical features.
  * Tripods are sandbagged, their feet positions taped, and they are placed outside
    walkways.
  * Bases are levelled and pan-zero oriented to the plan.
  * Snapshot overlay of expected versus seen beacons after coarse aim.
  * Fine aim after calibration.
  * An as-built coverage and accuracy report before flight.
* **Earliest test:** Phase 6. Install 4 stations from a plan, measure the aim error after
  coarse aim and after fine aim, and compare the as-built coverage with the plan.

### R19: Tall tripod (up to 20 m) sway and settling (M)

* **Evidence:**
  * A 20 m mast is a long, flexible lever. Air movement from HVAC or drone downwash, people,
    and the gimbal's own servo moves excite sway.
  * Sway of the mast top moves the camera *position* (mm–cm), which rotation-only live
    re-pose does not track. It also tilts the camera, which re-pose does track.
* **Mitigation:**
  * guy lines on tall masts;
  * a measured settle time after each servo move;
  * movement detection on beacon residuals;
  * a planner preference for trusses or lower tripods when they give similar coverage.
    In the model, raising tripods from 4 m to 8–20 m improved the worst point only from 11.5
    to about 9 mm; mixing 4 m and 16 m tripods (180 cameras) reached 99.6 % of points with
    worst 6.5 mm.
* **Earliest test:** Phase 9. Accelerometer + beacon log on a guyed 20 m mast: sway amplitude
  and spectrum, and settle time after a servo move.

### R20: Drone heading without a magnetometer or a measured yaw (M)

* **Context (D20):** one IR marker per drone, so IDPS measures position but not heading.
  Drones never yaw and always face North, and the magnetometer is off (steel trusses and
  motors make it unreliable indoors).
* **Evidence:**
  * PX4's EKF2 needs a heading source: a magnetometer, GPS or external vision. Its
    no-magnetometer yaw estimator works from GPS velocity, not from vision.
  * A heading error `ψ` turns every correction the position controller makes by `ψ`. A few
    degrees costs a little tracking accuracy; tens of degrees make the drone circle around
    its setpoint.
* **Mitigation:**
  * Put each drone down **facing North** on its home pad. Each pad is marked with an arrow,
    or uses a jig, aligned to the calibration square.
  * The PC sends heading 0 (North) with every position in `ODOMETRY`, with an uncertainty of
    about 3° that covers the placement. PX4 fuses it as external-vision yaw (`EKF2_EV_CTRL`
    yaw bit), and its gyros hold the heading between updates.
  * **Heading monitor (to build):** IDPS knows each drone's commanded acceleration (the
    setpoints it sends) and measures the acceleration it actually gets. A consistent angle
    between the two is the heading error. Alert, or refuse take-off, above a threshold such
    as 10°.
* **Earliest test:** PX4 SITL with the magnetometer off. Fly a show with heading errors of
  0, 5, 15 and 30° injected at placement, and measure the tracking error. On the first drone,
  log the heading drift over a 10-minute hover.

---

## 2. Decisions

| ID | Decision | Status |
|---|---|---|
| D1 | Accuracy metric and scope | **Decided:** within 1 cm throughout the entire flight volume, read as p95 3D error ≤ 10 mm at every location (see §0.1 for consequences) |
| D2 | Ground-truth equipment (total station rental, rotating-arm rig, linear rail) | Open. Recommended: rotating arm + rail now; total station for Phase 8/9 |
| D3 | Prototype station count and budget beyond S$200 | Open. Recommended: 4 stations for Phase 4 |
| D4 | Ethernet/PoE for anything beyond 3 Wi-Fi prototype stations | Open. Recommended: yes, and **mandatory for the full-volume requirement** (timing ≤ 0.05 ms) |
| D5 | Markers per drone | **Decided:** 1 marker (D20). Mount it at the drone's reference point (top centre, over the centre of mass); any remaining offset is declared to PX4 (`EKF2_EV_POS_X/Y/Z`), which corrects it with its own attitude (R16) |
| D6 | Gimbal type | **Decided:** servo pan/tilt, with live beacon re-pose and movement detection (R3). Upgrade path: worm-gear or braked servo if Phase 6 stability fails |
| D7 | Drone yaw convention | **Decided:** fixed yaw set at takeoff (normally 0 = facing North, the square's long arm); no yaw in flight |
| D8 | Drone reference point and marker offset | Open. See R16 |
| D9 | Site workflow | **Decided:** planner → install from the plan → one-click aim to plan → wand + calibration square at origin (0,0,0; long arm = North) → fly. See WORKFLOW.md |
| D10 | Minimum cameras per point | **Decided:** ≥ 3 everywhere (planner constraint, alongside p95 ≤ 10 mm) |
| D11 | Mounting options | **Decided:** overhead trusses and tripods (up to 20 m); no wall mounts |
| D13 | Calibrating the upper volume | **Decided:** wand only the walkable height (OptiTrack practice), then a continuous-calibration drone flight over the whole volume; continuous calibration may stay on during operation |
| D14 | Bumped tripod recovery | **Decided:** return the tripod to its taped marks → gimbal to the saved angles → reload the saved calibration → automatic 6-DOF beacon re-pose and verification; no re-wand |
| D16 | Station hardware | **Decided:** Raspberry Pi 5 (2 GB, active cooler) + OV9281 per camera, the camera on the servo pan/tilt and the Pi fixed beside it; powered and networked by one PoE+ cable to a gigabit PoE+ switch; PTP with hardware timestamps, camera 1's Pi as grandmaster. Chosen over the Pi 3B (no hardware bought yet): full-resolution sensor (about 4 fewer cameras), hardware PTP, PoE. See R4, docs/HARDWARE.md, docs/NETWORK_TIMING.md |
| D15 | Planner mounting inputs | **Decided:** volume; truss lines with height, usable and missing segments, additional trusses; tripod floor zones with height up to 20 m |
| D18 | Coordinate frame | **Decided:** North-East-Down (NED) from the origin = the calibration square's corner; North = its long arm = drone yaw 0; E 90° clockwise; D down. The planner asks for the performance area first, then the origin (dragged on the plan, N arrow turned to North). Everything shown, exported and sent to drones is NED; headings are from North, clockwise |
| D19 | Show design and drone firmware | **Decided:** shows designed in Skybrush Studio for Blender (.skyc / CSV export); drones run PX4. IDPS checks shows against the venue and coverage, exports one NED trajectory file per drone (by drone ID, D21), and each drone flies its own (D22, docs/SHOW.md) |
| D20 | Drone onboard set-up | **Decided:** no companion computer. The flight controller has Wi-Fi itself (or a few-gram Wi-Fi MAVLink bridge), and the PC talks to every drone over Wi-Fi: positions in (`ODOMETRY`, marked VISION so EKF2 fuses it), clock sync, start and abort; the drone flies its own file (D22). One IR marker per drone. Drones never yaw and face North (+X in NED), so the PC sends heading 0 as their heading source and the magnetometer stays off (R20) |
| D21 | Drone identity | **Decided:** every drone has a fixed ID 1–255. The same number is its PX4 `MAV_SYS_ID`, its IR marker ID and its label. The show's "Drone N" is flown by the drone with ID N, from `drone_NNN.trj`, standing on Drone N's home spot. Before take-off IDPS reads the marker IDs on the ground and blocks arming if a drone is missing, on another drone's spot or off its own, or if an unknown marker is in the area; every `idps-show` drone command refuses a link whose `MAV_SYS_ID` doesn't match, and each drone refuses to start more than 0.5 m from its own spot |
| D22 | Show playback | **Decided:** each drone flies its own show file from its SD card. A PX4 module (`px4/src/modules/idps_show`, built in as an external module, stock PX4 unchanged) loads `idps_show.trj`, checks its CRC and drone ID, and on the PC's start command counts down on the PC's clock: OFFBOARD at −3 s, arm at −2 s, fly at 0, land at the end. The PC uploads files over MAVLink FTP (`idps-show upload`), sends clock sync 5 times a second and one start time (`idps-show start`; if any drone refuses, all abort), and can abort (all land in place). Chosen over streaming setpoints from the PC: half the Wi-Fi packets, and the path survives short Wi-Fi drops. Tested in PX4 v1.16 SITL with 3 drones (px4/README.md) |
| D17 | Cameras vs drones height | **Decided:** drones always fly below the cameras. Every camera is at or above the flight ceiling (plus an optional margin) and looks level or down; trusses and tripod masts that can't reach the ceiling are not used. A volume taller than the 20 m tripods (e.g. the 30 m hall) is trusses only |
| D12 | Calibration tools | **Decided:** OptiTrack-style wand + ground square at origin. Proposed: 1 m × 1 m L-frame with 4 LEDs (origin, N 0.5, N 1.0, E 1.0). **Proposed for the hall:** long-baseline floor markers + laser distance meter (R18) |
