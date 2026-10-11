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

**Status: ±1 cm across the full 100×100×30 m volume is not yet shown to be feasible.** It is
not ruled out by geometry either. The plan keeps it as the requirement, and checks it at
explicit gates:

* **Phase 0:** the optimiser must find a layout that meets the spec under the tight budget.
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

### R4: Raspberry Pi 4B sustained 120 FPS (M)

* **Evidence:**
  * The OV9281 sensor supports 1280×800 at up to 120 FPS; whether the Pi 4B libcamera
    raw path sustains it with our processing on top must be **benchmarked**.
  * A 1 MP frame at 120 FPS is 123 Mpix/s. One full-frame read at ~4 GB/s effective memory
    bandwidth is about 0.3 ms, so a sparse NEON detector should fit in 2–4 ms of the
    8.33 ms budget.
  * Risks: unpacking RAW10, buffer-copy overhead, thermal throttling in an enclosure, and
    interrupt latency affecting timestamps.
* **Mitigation:**
  * 8-bit raw mode if the driver offers it;
  * zero-copy dmabuf access;
  * core pinning;
  * a heatsink and fan;
  * cropped or binned modes at long focal lengths;
  * Pi 5 or CM4 as a drop-in upgrade, since the edge code is portable.
* **Earliest test:** Phase 2 benchmark (ARCHITECTURE §4.3). **Decision:** if p99 processing
  exceeds 6 ms, or drops exceed 0.01 % over 30 minutes, choose among a cropped mode, a Pi 5,
  or 90 FPS.

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
  * A hand-held wand cannot reach most of a 100×100×30 m volume.
  * Bundle adjustment needs well-distributed co-observations.
  * A survey-grade world frame needs a total station.
* **Mitigation:**
  * a drone-carried wand flown on a coverage pattern;
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
  * a raw 8-bit or 10-bit mode at 120 FPS through libcamera on the Pi 4B;
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

* **Context:** drones take off at a specified yaw (normally 0, i.e. facing +X) and do not
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

---

## 2. Decisions

| ID | Decision | Status |
|---|---|---|
| D1 | Accuracy metric and scope | **Decided:** within 1 cm throughout the entire flight volume, read as p95 3D error ≤ 10 mm at every location (see §0.1 for consequences) |
| D2 | Ground-truth equipment (total station rental, rotating-arm rig, linear rail) | Open. Recommended: rotating arm + rail now; total station for Phase 8/9 |
| D3 | Prototype station count and budget beyond S$200 | Open. Recommended: 4 stations for Phase 4 |
| D4 | Ethernet/PoE for anything beyond 3 Wi-Fi prototype stations | Open. Recommended: yes, and **mandatory for the full-volume requirement** (timing ≤ 0.05 ms) |
| D5 | Markers per drone | Open. Depends on R16: 1 marker if it can sit at the reference point; otherwise 2–3 to measure roll and pitch |
| D6 | Gimbal type | **Decided:** servo pan/tilt, with live beacon re-pose and movement detection (R3). Upgrade path: worm-gear or braked servo if Phase 6 stability fails |
| D7 | Drone yaw convention | **Decided:** fixed yaw set at takeoff (normally 0 = facing +X); no yaw in flight |
| D8 | Drone reference point and marker offset | Open. See R16 |
