# IDPS Operator Workflow: Plan → Install → Aim → Calibrate → Fly

This document describes the owners' intended end-to-end workflow and how the software
supports each step. It is the user-facing view of ARCHITECTURE §4.4, §6 and §7.

```text
 PLANNING (office)          INSTALL (site)           AIM (site, 1 click)
 ┌──────────────────┐       ┌──────────────────┐     ┌────────────────────────┐
 │ Define flight     │       │ Mount cameras +  │     │ "Set cameras to planned │
 │ volume + mounts   │──────>│ beacons at the   │────>│  angles": every servo   │
 │ Optimiser outputs │ plan  │ planned spots    │     │  pans/tilts to the plan │
 │ install plan      │       │ (tape measure)   │     └───────────┬────────────┘
 └──────────────────┘       └──────────────────┘                 │
                                                                  v
 FLY / TRACK                 VERIFY                    CALIBRATE (site)
 ┌──────────────────┐       ┌──────────────────┐     ┌────────────────────────┐
 │ Live tracking,    │<──────│ Coverage + error │<────│ 1. Wand through volume │
 │ live re-pose from │       │ check against    │     │ 2. Square on floor at  │
 │ beacons           │       │ the plan         │     │    origin = (0,0,0)    │
 └──────────────────┘       └──────────────────┘     │ 3. Beacons self-survey │
                                                      └────────────────────────┘
```

---

## Step 1: Planning (software only, before going on site)

The **IDPS Planner** (`idps-plan`, built on the coverage simulator in ARCHITECTURE §7) takes
the following inputs.

| Input | Example | Why it is needed |
|---|---|---|
| Flight volume | **Drawn per venue:** the floor outline as XY vertices (any shape, e.g. a 50 × 20 m rectangle, an L-shape, a hexagon), extruded in Z to the flight ceiling (e.g. 0–20 m). Vertices are numbered V0, V1, …; edge Ek runs from Vk to Vk+1 | The region where every point must meet the accuracy and coverage target. Venues differ, so the volume is never assumed |
| **Trusses** | For each truss: its line (start/end XY), height, and **which segments are usable**. Missing sections are marked, and **additional trusses** can be added (e.g. a hired truss line). Drawn on a plan view or imported from CSV/DXF | Cameras may only be placed on usable truss segments. Walls are not used (decision D11) |
| **Tripod zones** | Either a **strip outside the drawn volume** (e.g. 1–3 m from its edges, optionally only along chosen edges), or explicit floor polygons. **Tripod height from 1 m up to 20 m** (per zone, or chosen by the optimiser within that range). Tripods are never placed inside the flight volume. **Cameras are always at or above the flight ceiling** (drones never fly higher than a camera, D17), so a mast must reach the volume's Top Z; shorter masts are not used | Used where there is no truss, looking level or down across the volume from its edges. A strip follows any venue shape automatically |
| Design margin | e.g. ≥ 4 cameras and p95 ≤ 7 mm | What the optimiser aims for, stricter than the requirement, so the plan tolerates occlusion and a lost camera |
| No-go zones | Doors, walkways, lights, emergency exits | Excluded from mounting |
| Obstacles | Columns, nets, hanging equipment | Occlusion |
| Hardware catalogue | Sensor (OV9281), lens options (2.8 / 3.6 / 4 / 6 / 8 mm), servo pan/tilt ranges, gimbal geometry | Field of view, resolution, aim limits |
| Requirements | p95 ≤ 10 mm everywhere; **≥ 3 cameras everywhere** (configurable) | Optimisation constraints |
| Origin and axes | Where the calibration square will sit and which way its X arm points | All planned coordinates are given in this frame, which is the same frame as tracking |

**Outputs (the install plan):**

1. **Camera count and lens per camera.** The minimum number of stations that meets the
   requirements.
2. **Camera table:** one row per camera (CSV + printable PDF/HTML).

   | Camera | Surface | X, Y, Z (m) | Measure from | Pan / Tilt (°) | Lens | Beacons in view |
   |---|---|---|---|---|---|---|
   | C01 | Truss T3 | 12.50, 40.00, 31.50 | Truss T3, 12.50 m from its west end | 34.0 / −62.5 | 2.8 mm | 14 |
   | C57 | Tripod | −52.00, 18.75, 4.00 | 2.00 m outside the west boundary, 18.75 m from the origin line, mast 4.00 m | 90.0 / 38.0 | 2.8 mm | 11 |
   | … | | | | | | |

   * The *Measure from* column turns coordinates into **tape-measure instructions** relative
     to the drawn outline: "1.50 m outside edge E2 (V2→V3), 12.00 m along it from V2", or
     "Truss T2, 18.00 m from its start end". Installers never need to work in abstract XYZ.
   * The plan also lists the **spacing to neighbouring cameras** along each wall or truss.
     It is the installer's cross-check while placing cameras.
3. **Beacon table:** position and tape-measure instructions for every fixed reference
   beacon. Beacons are required for servo live re-pose (ARCHITECTURE §6.4).
4. **Coverage maps:** horizontal slices (e.g. every 2 m of altitude), vertical sections and an
   interactive 3D view. Two layers can be toggled:
   * **Camera count:** colour-coded 0 / 1 / 2 / **≥ 3** cameras. Regions with fewer than 3
     cameras are shown in red and listed with their volume in m³.
   * **Predicted accuracy:** p95 error in mm per voxel; regions over 10 mm are highlighted.
     *Three cameras is necessary but not sufficient:* a point can be seen by 3 cameras with
     poor geometry (nearly parallel rays) and still miss 1 cm. Both layers are therefore
     always reported.
5. **Install tolerance per camera:** how far a camera can be off its planned spot (e.g.
   ±10 cm, ±2° base rotation) before coverage or accuracy drops below the requirement.
   * Cameras with tight tolerances are flagged, so installers take extra care with them.
   * The optimiser prefers layouts that stay compliant when every camera is somewhat off
     plan (robust optimisation).
6. **What-if tools:** add, remove or move a camera, then re-run and see the change in the
   coverage and accuracy maps (e.g. "what if camera C17 cannot be mounted?").
7. **Bill of materials:** stations, lenses, beacons, PoE switch ports, cable runs.

The plan is saved as a versioned project file (`plan.yaml`). The same file drives the
on-site aim step and is compared against the calibration result.

---

## Step 2: Install (on site)

1. Mount each station at its planned position using the tape-measure instructions: on the
   named truss at the given distance, or on a tripod at the marked floor spot and mast
   height. Exact
   placement is **not** needed for accuracy, because calibration measures where each camera
   really is. It only needs to be within the plan's install tolerance, so that coverage
   holds.
2. **Orient the gimbal base**, which is critical for automatic aiming:
   * Level the base with a bubble level.
   * Point its pan-zero mark in the direction the plan specifies (e.g. "pan-zero
     perpendicular to the wall", or toward a named reference point).
   * The servo can only aim relative to its base. A base rotated by 5° gives a camera aimed
     5° off.
3. **Tripods must not move during a session.**
   * Spread the legs fully, weight them with sandbags and tape the feet positions on the
     floor.
   * Keep tripods outside walkways and cordon them off if people move around.
   * **Tall tripods (up to 20 m) need guy lines.** A tall mast sways and rings after a knock
     or a servo move; the settle time is measured per mast type (RISKS R19).
   * If a tripod is bumped, the system detects it and drops that camera; recover it as
     described in Step 7 without re-wanding.
4. Mount beacons at their planned positions.
5. Connect PoE. Each station appears in the dashboard and the operator assigns it its plan ID
   (C01, C02, …). A label or QR code on each station speeds this up.

---

## Step 3: Aim: "Set cameras to optimised angles" (one click)

The dashboard sends each station its planned pan/tilt (`gimbal_move`). All servos move in
parallel. The dashboard then shows:

* each camera's live snapshot with the beacons it is *expected* to see overlaid on the beacons
  it *does* see;
* a warning if a camera's view is blocked or clearly misaimed (e.g. expected beacons are
  missing).

At this stage the aim is **coarse**: servo accuracy plus the base orientation error, about
1–3°. That is enough for coverage, and the final aim is refined after calibration (step 4, item 6).

---

## Step 4: Calibrate

1. **Wand (walkable height only, as in OptiTrack):** the operator walks the calibration wand
   (3 coded LEDs at fixed spacing) through the walkable layer, from the floor to about 2.5 m.
   The dashboard shows live wand-sample coverage per camera and per floor region. Collection
   stops when every camera that can see the walkable layer has enough well-spread samples.
   The upper volume is calibrated by drone flight (item 6 below).
   * Cameras aimed level across the top of the volume may **not see the walkable layer at all**,
     and those cannot be wand-calibrated. The planner lists them. They are calibrated by one of:
     * the **continuous-calibration flight** (item 6), from drone markers co-observed with
       already-calibrated cameras; or
     * **tilt-down wanding:** during wanding their servos tilt down to see the wand, and
       afterwards they return to the planned angle, with beacons re-measuring the rotation.
       This relies on the gimbal kinematic model for the small position change.
2. **Calibration square on the floor at the origin.** This is the same wand + ground-square
   method OptiTrack and Vicon use. It defines the world frame:
   * Its marked corner is **(0, 0, 0)**.
   * Its X arm defines **+X**. This is the direction drones face at yaw 0 (RISKS D7).
   * Its Y arm defines +Y.
   * The floor plane through it defines Z = 0, with +Z up.

   **Proposed square (decision D12):** an L-frame with **1 m arms along X and Y**, with
   4 IR LEDs:
   * one at the origin;
   * one at X = 0.5 m;
   * one at X = 1.0 m;
   * one at Y = 1.0 m.

   The extra LED at X = 0.5 m makes the X arm distinguishable from the Y arm. With equal arms
   and only 3 LEDs, the software could not tell them apart. The frame also needs levelling
   feet and a bubble level, a known LED height above the floor, and a rigid material such as
   aluminium box section.
3. **Axis and scale extension markers (hall scale only).** A 1 m square is enough for the
   origin, but too short to fix the axis directions and scale over 100 m. Small errors grow
   with distance from the origin (RISKS R18). For the hall:
   * Place 2 extra floor LEDs along the +X line, e.g. at about 40 m and 90 m. Place 1–2 along
     the +Y line. Line them up with a laser line from the square.
   * Measure their distances from the origin with a **laser distance meter** (±1 mm class).
   * The software uses these long baselines to fix +X and +Y, the floor plane (Z = 0) and the
     overall scale. The wand then provides only the camera-to-camera geometry.
   * In a lab-sized room, the 1 m square and the wand are sufficient on their own.
4. **Solve:** a bundle adjustment computes every camera's exact position and rotation, plus
   wand points. It is then aligned to the square's frame, with the extension markers fixing
   the axes and scale.
5. **Beacon self-survey:** with the cameras now calibrated, the system triangulates every
   fixed beacon, averaging many cameras over tens of seconds. These measured beacon positions
   are what live re-pose uses from then on.
   * This removes the need for a total station to survey beacons.
   * A total station is still recommended for the *independent accuracy check* in
     VALIDATION.md, because a system cannot validate itself.
6. **Continuous-calibration flight (the whole show volume).** Enable continuous calibration;
   a drone with markers takes off and flies a pattern covering the entire usable volume. The
   planner generates the pattern, weighted toward regions with few wand samples.
   * The drone's markers act as calibration points: the bundle adjustment refines every
     camera's pose from them, and calibrates the cameras that never saw the wand.
   * Calibrated cameras anchor the scale and world frame, so the drone needs no known
     geometry. Two markers at a known spacing on the drone add an extra scale check at
     altitude.
   * The dashboard shows per-camera calibration quality improving live. The result is saved
     as a new calibration version.
   * Continuous calibration can stay enabled during normal operation (ARCHITECTURE §6.6).
7. **Fine aim (optional, automatic):** now that each camera's true pose is known, the software
   computes the servo correction that brings each camera to its exact planned view direction.
   It moves the cameras, and live re-pose (from the beacons) updates their rotation; a full
   re-calibration is not needed.

---

## Step 5: Verify against the plan

The dashboard compares **as-built** against **as-planned**:

* Camera positions: measured versus planned, with the difference in cm.
* A re-computed coverage and accuracy map using the measured poses. Regions that dropped
  below 3 cameras or over 10 mm are highlighted.
* The calibration quality report: reprojection error, wand length error, square residuals.
* A pass/fail summary. The system is ready to fly only when every region meets the
  requirement, or the operator explicitly accepts the listed exceptions.

---

## Step 6: Fly / track

* Normal tracking runs with **live re-pose**: every camera keeps measuring its own rotation
  from the beacons.
* A camera that moves or jitters is automatically dropped and re-admitted.
* If the operator re-aims a camera later, it re-poses from the beacons without a new wand
  calibration.
* With continuous calibration enabled, every flight also refines the calibration. A full
  wand re-calibration is needed only after major changes (many stations moved, lens change).
* A bumped tripod is recovered without re-wanding (Step 7).

---

## Step 7: Recovering a bumped tripod (no re-wanding)

1. Move the tripod back to its taped floor marks and restore the mast height.
2. In the dashboard, select the station and click **"Restore"**. This sends the gimbal back
   to its planned (or last fine-aimed) pan/tilt and reloads the station's pose from the
   saved calibration file.
3. The system **checks the restored pose automatically** instead of trusting it. Tape marks
   usually return a tripod to within about 5–10 mm and a few tenths of a degree, but that is
   not 1 cm-grade.
   * A full 6-DOF re-pose against the self-surveyed beacons corrects the remaining position
     and rotation offset.
   * Continuous calibration, during the next drone flight, refines it further.
4. The station returns to service only when its beacon residuals pass the threshold. The
   dashboard shows how far the restored pose had to be corrected. If the correction is larger
   than the beacons can resolve, the dashboard asks for a short re-wand or a drone pass in
   that camera's view.
