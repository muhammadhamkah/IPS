#!/bin/bash
# The 5-drone choreography flown by PX4 SITL 60 g quads, scored against the show files.
#
#   PX4_DIR=~/PX4-Autopilot px4/sitl/run_loop.sh cameras   # closed loop: simulated cameras + engine
#   PX4_DIR=~/PX4-Autopilot px4/sitl/run_loop.sh vision    # engine-like positions inside PX4 instead
#
# cameras: needs a machine that runs 5 PX4 instances, 11 cameras at 60 fps and the engine in real
# time (8+ cores). Watch the cameras' "behind the clock" (out/loop/cams.log, want p99 < 30 ms)
# and the engine's "exposure to send" (out/loop/engine.log): EV_DELAY_MS should match the latter.
# Env: N (drones, 1-5), OUT (default out/loop), EV_DELAY_MS (default 28), PX4_BUILD,
#      IMU_NOISE (default 0.1, needs sih_noise_scale.patch), WORKERS (camera processes, default 3).
set -e
MODE=${1:-cameras}
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
cd "$ROOT"
N=${N:-5}; OUT=${OUT:-out/loop}; export EV_DELAY_MS=${EV_DELAY_MS:-28}
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}; B=$PX4_DIR/build/${PX4_BUILD:-px4_sitl_default}
rm -rf "$OUT"; mkdir -p "$OUT"
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done; pkill -x px4 || true; }
trap cleanup EXIT
trap 'exit 143' TERM INT           # stopped (e.g. from the dashboard): clean up too

echo "== show"
python examples/shows/five_drones_sphere_square.py "$OUT/five.zip" > "$OUT/show.txt"
python -m idps.show.cli export "$OUT/five.zip" --site examples/sites/show_8x8.yaml \
    --plan examples/plans/show_8x8.yaml --min-sep 0.9 -o "$OUT/show" | tail -1
python - "$OUT/show/manifest.json" "$N" <<'PY'
import json, sys
p, n = sys.argv[1], int(sys.argv[2])
m = json.load(open(p)); m["drones"] = [d for d in m["drones"] if d["drone_id"] <= n]
json.dump(m, open(p, "w"), indent=1)
PY

echo "== $N PX4 drones (micro60)"
if [ "$MODE" = vision ]; then export SITL_VISION="${SITL_VISION:-$EV_DELAY_MS 2 120}"; fi
if ! DRONE=micro60 PX4_DIR=$PX4_DIR PX4_PARAM_SDLOG_MODE=${PX4_PARAM_SDLOG_MODE:--1} \
    bash px4/sitl/run_sitl.sh "$N" > "$OUT/sitl.log" 2>&1; then
  echo "Starting the PX4 drones failed:"; tail -30 "$OUT/sitl.log"; exit 1
fi
L=""; L14=""
for ((i=0;i<N;i++)); do
  L="$L --link $((i+1))=udpin:127.0.0.1:$((14540+i))"; L14="$L14 --link $((i+1))=udpin:127.0.0.1:$((14030+i))"
done

if [ "$MODE" = vision ]; then
  echo "== positions inside PX4: 2 mm noise, 120 Hz, $EV_DELAY_MS ms late"
  python px4/sitl/idps_stand_in.py "$OUT/show/manifest.json" "$N" "$OUT/truth.csv" log-only > "$OUT/standin.log" 2>&1 &
  PIDS+=($!); disown $!; TRACK=$OUT/truth.track.csv; sleep 8
else
  echo "== cameras and engine"
  python -m idps.sim px4 examples/scenarios/show_8x8_micro60.yaml --manifest "$OUT/show/manifest.json" \
      --duration 600 --workers "${WORKERS:-3}" -o "$OUT" > "$OUT/cams.log" 2>&1 &
  PIDS+=($!); disown $!
  while [ ! -s "$OUT/calibration.yaml" ]; do sleep 0.5; done; sleep 1
  python -m idps.engine.cli live --port 5600 --calib "$OUT/calibration.yaml" --site examples/sites/show_8x8.yaml \
      $L14 -o "$OUT/positions.csv" > "$OUT/engine.log" 2>&1 &
  PIDS+=($!); disown $!; TRACK=$OUT/track.csv
  echo "   waiting 30 s for the cameras to identify every drone"; sleep 30
  tail -1 "$OUT/cams.log"; tail -1 "$OUT/engine.log"
fi

echo "== upload and fly (about 80 s)"
python -m idps.show.cli upload "$OUT/show/manifest.json" $L --sitl | grep -c OK | sed 's/$/ drone(s) uploaded/'
python -m idps.show.cli start "$OUT/show/manifest.json" $L --lead 15 > "$OUT/start.txt" 2>&1 || true
grep -E "denied|refused|Not started" "$OUT/start.txt" || true
for ((i=0;i<N;i++)); do
  f=$(grep -hE "Failsafe|stopped" "$B/rootfs/$i/out.log" | head -1 || true)
  [ -n "$f" ] && echo "   drone $((i+1)): $f"
done
[ "$MODE" = cameras ] && { grep "behind the clock" "$OUT/cams.log" | tail -1; grep "exposure" "$OUT/engine.log" | tail -1; }

echo "== result (distance from the show file)"
python px4/sitl/score_track.py "$TRACK"
