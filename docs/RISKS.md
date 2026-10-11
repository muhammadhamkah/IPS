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
2. **±1 cm at p95 over 100×100×30 m is not met by any simulated layout under "realistic"
   assumptions.** It needs about 90–130 stations *and* calibration held to about 0.05 mrad
   (≈ 10 arcsec) per camera *and* ≤ 0.05 ms timing. Corners and the band near the ceiling
   remain the worst regions.
3. **Calibration stability, not pixel count, is the dominant term at long range.** A
   0.1 mrad rotation error equals 7 mm at 70 m. Adding cameras dilutes it only if the
   cameras' errors are independent, and that remains unproven.
4. **Recommendation:** treat ±1 cm as the lab-scale acceptance target first (Phase 8). For
   the hall, define a *graded* requirement (e.g. ±1 cm in a core zone, ±3 cm elsewhere),
   pending Phase 9 data. **The owners must confirm this (decision D1).**

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

### R3: Motorised gimbal repeatability and stability (H for accuracy, L for coverage)

* **Evidence:**
  * Hobby servos have about 0.2–1° deadband and jitter, i.e. 3–17 mrad. That is **50–300×**
    the 0.05 mrad stability budget.
  * Even good stepper and worm-gear stages hold to ~0.01–0.05° (0.2–0.9 mrad) without
    re-calibration.
* **Consequence:** the gimbal can only be a **coarse aiming tool**:
  * Every move must be followed by mechanical locking and beacon-based re-posing.
  * Commanded angles can seed PnP but never replace it.
  * Servos must be de-energised or braked during tracking, because servo hunting at
    0.1–1 Hz makes the camera pose time-varying.
* **Mitigation:**
  * self-locking worm-gear or braked stepper gimbals;
  * per-station beacon visibility as a placement constraint;
  * UNCALIBRATED gating in the engine.
* **Earliest test:** Phase 6. Move, return and re-pose 50 times. Report re-pose accuracy
  against a full bundle-adjustment reference and image-space jitter of a fixed beacon with
  the gimbal powered versus locked.

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

### R5: Network scale with 90–130 stations (M)

* **Evidence:**
  * Aggregate data is small (~0.35 Mbit/s per station at 20 markers, ~45 Mbit/s total).
  * However, 130 stations × 120 packets/s = 15,600 small packets/s.
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
  * The hall-scale simulation suggests about 90–130 stations, plus PoE switching, mounts,
    beacons and a survey.
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

---

## 2. Decisions needed from the project owners

| ID | Decision | Recommended default |
|---|---|---|
| D1 | Exact accuracy metric and whether hall-scale accuracy may be graded by zone | p95 3D error ≤ 10 mm (lab); hall target graded, revisit after Phase 9 |
| D2 | Ground-truth equipment (total station rental, rotating-arm rig, linear rail) | Rotating arm + rail (cheap) now; total station for Phase 8/9 |
| D3 | Prototype station count and budget beyond S$200 | 4 stations for Phase 4 |
| D4 | Ethernet/PoE for anything beyond 3 Wi-Fi prototype stations | Yes |
| D5 | Number of markers per drone | 1 for Phases 1–9; 2–3 for rigid-body pose in Phase 10 |
| D6 | Gimbal type | Self-locking worm-gear or braked stepper; servos only for bench experiments |
