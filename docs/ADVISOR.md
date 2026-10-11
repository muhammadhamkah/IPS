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
| **Show / flight** (brain, simulated feed now) | Station health (FPS, drops, PTP offset, temperature), live re-pose drift, packets, tracks where drones are lost | Camera knocked, a station down, timing drift, a repeated blind spot | "C004 moved: run Restore", "drones lose tracking above 8 m near V2: add a camera at …" |

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

## The brain: live monitoring (`idps/monitor/`, Live tracking tab)

The operator does not describe problems: the brain watches every tick on its own and raises an
alert only for a *repeated* problem, with the evidence and a fix. The operator just decides:
**Accept fix** or **Live with it**.

| What it watches | How it decides | What it recommends |
|---|---|---|
| **Tracking loss** | Each drone seen by fewer than the required cameras for ≥ 0.2 s is a loss episode. Episodes are clustered in 2 m cells over the last 3 minutes. A *pattern* is ≥ 5 episodes from ≥ 2 drones (or ≥ 10 from one drone over ≥ 30 s), so a one-off occlusion by another drone stays silent. | A focused advisor run (below): re-aim / move / add, with tape-measure dimensions |
| **Camera offline** | No data for 1 s | Get it back online; a motorised stop-gap re-aim of a neighbour if the others don't cover its area (otherwise the alert is downgraded to a warning) |
| **Camera knocked** | Live re-pose differs from calibration by ≥ 0.5 mrad (critical ≥ 2 mrad) | **Restore**: the servos re-aim it to the calibrated view and the saved calibration is reloaded, with no re-wand |

**Why it happens.** For each pattern the brain compares what the model expected with what
happened:
* cameras that should see the region but see drones there far less often than elsewhere are
  called **blocked** (set pieces, rigging, a banner);
* cameras that are more than 15° below the drones are called out separately. The marker on top
  of a drone is hidden from them by the drone's own body, and cameras above that height fix it.

Losses while a camera was offline or knocked are explained by that fault and are not reported
twice.

**Focused fixes.** `Advisor(focus=Focus(...))` scores fixes for the problem region:
* The places where drones were actually lost become evaluation points.
* At those points, an existing camera that had the drone in view but missed it counts as
  blocked. This evidence is tied to the camera's position, so it survives a re-aim (same line of
  sight) and is cleared by a move.
* Every other point in the venue still counts, so a fix for one region cannot break another.
* If no change meets the requirement there, the brain asks for one camera of margin.
* When a fix only partly helps, the alert says so and names what else to do: clear the
  obstruction, or add a tripod zone or truss nearby.

**Follow-up.**
* *Accepted:* the plan, install sheet and Planner update. The brain ignores losses for 5 s
  while things settle, then confirms the fix once drones pass through the region for 90 s
  without a loss. If losses continue, it reopens the alert with the next step.
* *Two rounds without a cure:* the alert says something physical is probably in the way and
  names the lines of sight to walk. From then on it only suggests adding a camera from a new
  direction.
* *Lived with:* no more alerts unless the loss rate at least doubles.
* *Quiet:* issues that go quiet resolve themselves. An issue that comes back soon after is
  reopened rather than duplicated.

**Rehearsal.** Until the positioning engine exists, a simulated show feeds the brain
(`idps/monitor/simsource.py`):
* drones fly a repeating formation route inside the polygon volume;
* each station's detections are modelled per drone, covering field of view, brightness, the
  drone's own body, other drones and scenery boxes;
* **Inject fault** can hang obstructions where the drones fly, take a camera offline, or knock
  one.

## Next steps

1. Feed calibration results (as-built poses, residuals) into `Advisor` in place of planned poses.
2. Connect the brain to the positioning engine: `TrackSample` per drone per frame batch and
   `StationStatus` from the HEALTH packets and the live re-pose.
3. AI assistant panel, after the owners decide on connectivity and keys. It would explain alerts
   and answer questions about them, using the brain's numbers.
