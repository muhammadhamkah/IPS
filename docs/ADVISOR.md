# IDPS Advisor: health monitoring and guided fixes

**Goal:** the operator never has to guess. From planning, through install and calibration, to
the show itself, the system says what is wrong, why, and the exact fix. The fix is one of:
* re-aim a camera (motorised, one click);
* move a camera (with tape-measure dimensions);
* add a camera (where, how high, pointing where).

## Two layers

1. **Advisor engine (deterministic, `idps/plan/advisor.py`).** Recommendations are exact geometry
   from the same accuracy model as the planner. Every fix is scored by how many evaluation points
   stop failing the requirement, and can be previewed and applied. Nothing is guessed, so every
   claim can be checked; a test verifies the "points fixed" figure for each recommendation.
2. **AI assistant (planned).** A language model (Claude) sits on top of the engine. It:
   * explains the findings in plain language;
   * answers operator questions ("why is the north-east corner weak?");
   * walks the crew through a fix step by step.

   It only ever reads the engine's numbers and calls its functions; it never invents positions.
   Open questions for the owners:
   * Do venues have internet access, or is an offline/local model needed?
   * The Anthropic API key and cost limits.
   * Which actions may run automatically (e.g. a re-aim during calibration) and which always
     need operator confirmation.

## What it watches, by phase

| Phase | Inputs | Checks | Typical advice |
|---|---|---|---|
| **Planning** (done) | Venue, mounts, planned cameras, predicted coverage and accuracy | Points failing ≥ 3 cameras / ≤ 10 mm; most critical camera; single-camera-loss robustness | Add a camera at …, re-aim C004, raise the truss |
| **Install** | Planned positions; as-installed cameras from the crew checklist | Install tolerance per camera | "C012 is mounted 0.4 m off plan: still fine" or "re-aim it" |
| **Calibration** | Wand samples, bundle-adjustment residuals, as-built poses, beacon visibility | Unwanded regions, high-residual cameras, coverage recomputed with *actual* poses | "Wand more near V2 above 2 m", "C007 sees only 5 beacons: re-aim it to pan +12°" |
| **Show / flight** | Station health (FPS, drops, PTP offset, temperature), live re-pose drift, packets, tracks where drones are lost | Camera knocked, a station down, timing drift, a repeated blind spot | "C004 moved: run Restore", "drones lose tracking above 8 m near V2: add a camera at …" |

The same engine serves every phase; only its inputs change. During calibration and flight it is
fed *measured* data (actual poses, observed per-point coverage, dropped stations) instead of
predictions. Detecting problems from data that flows anyway is the job of the positioning
engine's QA stage (ARCHITECTURE §5.1).

## In the dashboard now (Planner → Advisor tab)

* **System health:** failing points and volume, and the most critical camera.
* **Find fixes:** the best re-aim and move for each camera, plus the best new camera, ranked by
  problem points removed. **Show** previews the change on the 2D plan and in 3D; **Apply**
  updates the layout and re-evaluates it.
* **Auto-fix plan:** apply the best fix, re-check, repeat. The result is an ordered to-do list
  (e.g. "3 changes take the failing points from 43 to 0"), applied with **Apply all**.
* **Fixes respect the venue's rules:** allowed mounts only, minimum camera spacing, and the
  servo pan range (a re-aim beyond ±90° asks for the gimbal base to be turned).
  Mounts drawn after planning, such as a tripod strip, are used too.

## Next steps

1. Feed calibration results (as-built poses, residuals) into `Advisor` in place of planned poses.
2. Live health rules in the engine QA stage, raised as advisor alerts with a fix attached.
3. AI assistant panel, after the owners decide on connectivity and keys.
