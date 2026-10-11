# IDPS Accuracy Definition and Validation Plan

The ±1 cm requirement is an **engineering target**. It is not a demonstrated capability until
the tests in this document pass against independent ground truth.

## 1. Error metric (proposed; decision D1 in RISKS.md)

* **Primary metric:** the 3D Euclidean error `e = ‖p_IDPS − p_truth‖` of the **marker centre**.
  It is evaluated over a stated test volume, speed range and camera configuration.
* **Acceptance (proposed):**
  * p95(e) ≤ 10 mm **and** max(e) ≤ 25 mm, excluding samples the system itself flags as
    invalid.
  * The invalid-sample rate must also be reported, and must be ≤ 1 %.
* **Static and dynamic results are reported separately.** Dynamic results are binned by speed:
  < 1, 1–2, 2–4 m/s.
* **Every result also states:**
  * RMS per axis (X, Y, Z separately; Z often behaves differently);
  * bias (mean error vector);
  * the calibration version;
  * the number of cameras;
  * the sync mode;
  * the test-volume definition.
* **Consistency check:** the engine's reported covariance must match the observed errors. The
  normalised estimation error squared (NEES) averaged over a test should be ≈ 3 (the
  χ²₃ mean). A significantly larger value means the system is over-confident and must be
  fixed before flight use.

Note that "±1 cm per axis" and "1 cm 3D" differ by about √3. The requirement must name one.
This plan uses 3D.

## 2. Ground-truth methods

| ID | Method | Measures | Truth accuracy | Cost |
|---|---|---|---|---|
| G1 | **Fixed-length wand**: two markers on a carbon rod, known length (measured with calipers) | Scale and relative accuracy anywhere in the volume, static and moving | ≤ 0.1 mm length | Very low |
| G2 | **Rotating arm**: marker on an arm at radius r, driven by a motor with an encoder | Dynamic error up to 4 m/s, from circle-fit residuals; timing errors appear as phase lag | ~0.5 mm | Low |
| G3 | **Linear rail**: motorised stage with an encoder | Dynamic error along one axis; latency | ~0.1 mm | Low–medium |
| G4 | **Surveyed static points**: total station or laser distance meter on a tripod-mounted marker grid | Absolute accuracy in the world frame | 1–3 mm (total station) | Rental |
| G5 | **Cross-validation**: hold out k cameras, compare the position from the remaining cameras | Calibration consistency; no external truth needed | — | Free |

G1, G2 and G5 cover most of Phases 4–7 without a total station. G4 is required for the Phase 8
sign-off.

## 3. Test catalogue

| Test | Phase | Procedure | Pass criterion |
|---|---|---|---|
| T1 Centroid precision | 1 | Static marker, 1000 frames, per range and defocus setting | σ ≤ 0.05 px at high SNR; report the bias-versus-subpixel-phase curve |
| T2 Detection reliability | 1 | Marker at various angles and ranges; ambient light on and off | Detection rate ≥ 99.9 %; false positives ≤ 1 per 10⁴ frames after masking |
| T3 Station throughput | 2 | 30-minute soak at 120 FPS with 1, 10 and 50 synthetic blobs | ARCHITECTURE P1–P3 |
| T4 Network | 3 | 1-hour stream, wired and Wi-Fi | Loss and latency per P4/P5/P8 |
| T5 Timing | 3, 5 | Sync-master GPIO strobes a marker LED at known times; every station observes it | Per-station timestamp offset known to σ ≤ 100 µs (P6) |
| T6 Static accuracy | 4, 8 | G4 grid of ≥ 20 points across the volume, 10 s each | §1 acceptance (static) |
| T7 Dynamic accuracy | 5, 8 | G2 at 1, 2 and 4 m/s; G3 traverses | §1 acceptance (dynamic) |
| T8 Scale consistency | 4, 6, 8 | G1 wand waved through the volume | Length error p95 ≤ 2 mm (lab) |
| T9 Calibration stability | 6 | 24 h beacon drift log with temperature | ≤ 0.05 mrad drift (hall), ≤ 0.2 mrad (lab) |
| T10 Re-pose after a gimbal move | 6 | 50 move-return cycles | Re-pose rotation error ≤ 0.1 mrad versus the bundle-adjustment reference |
| T11 ID decoding | 1, 4, 10 | Multiple markers, crossings, occlusions | Wrong-ID rate < 1e-4; acquisition time per P13 |
| T12 Long range | 9 | T1, T2, T6 and T7 repeated at 25–30 m and beyond | Report the curves; compare with the simulator's prediction |
| T13 Covariance consistency | 4–9 | NEES over T6/T7 | Mean NEES within the χ²₃ 95 % band for the sample size |

## 4. Simulator-versus-reality loop

For every hardware test, the matching scenario is run in `idps-sim` with the *measured* noise
parameters. A significant gap between predicted and measured error means the model is missing
a physical effect. The model must be updated before hardware-scale predictions (station count,
layout) are trusted.

## 5. Gate before any autonomous flight

Flight-control integration is not permitted until all of the following hold:

1. T6 and T7 pass in the intended flight volume.
2. T13 shows the reported uncertainty is honest.
3. Failure modes have been demonstrated to be detected: camera loss, calibration drift, ID
   swap, and occlusion. Detected means the output is flagged invalid within ≤ 100 ms.
