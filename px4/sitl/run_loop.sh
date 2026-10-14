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
PY=${PYTHON:-python}                    # the IDPS python (the dashboard passes its own)
"$PY" -c "import idps" 2>/dev/null || { echo "$PY cannot import idps: activate the IDPS venv (pip install -e .)"; exit 1; }
N=${N:-5}; OUT=${OUT:-out/loop}; export EV_DELAY_MS=${EV_DELAY_MS:-28}
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}; B=$PX4_DIR/build/${PX4_BUILD:-px4_sitl_default}
rm -rf "$OUT"; mkdir -p "$OUT"
PIDS=()
cleanup() {
  for p in "${PIDS[@]}"; do pkill -P "$p" 2>/dev/null || true; kill "$p" 2>/dev/null || true; done
  # a stopped show start first aborts every drone: give it up to 5 s while PX4 still answers
  for ((t=0;t<20;t++)); do
    alive=0; for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [ $alive = 0 ] && break; sleep 0.25
  done
  for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null || true; done
  pkill -x px4 || true
}
trap cleanup EXIT
trap 'exit 143' TERM INT           # stopped (e.g. from the dashboard): clean up too

# the drones' links must be free: a leftover run would take their messages
"$PY" - "$N" <<'PY' || exit 1
import socket, sys
busy = []
n = int(sys.argv[1])
for port in [14800 + i for i in range(n)] + [14540 + i for i in range(n)] + [14030 + i for i in range(n)] + [5600]:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   # truth links, drone links, engine links, engine
    try:
        s.bind(("127.0.0.1" if port != 5600 else "0.0.0.0", port))
    except OSError:
        busy.append(port)
    s.close()
if busy:
    print(f"udp port(s) {busy} are in use, probably by an earlier run still going. Stop it first, e.g.:\n"
          "  pkill -f 'idps.sim px4'; pkill -f idps_stand_in; pkill -f 'idps.engine.cli live'; pkill -x px4")
    sys.exit(1)
PY
echo "== show"
"$PY" examples/shows/five_drones_sphere_square.py "$OUT/five.zip" > "$OUT/show.txt"
"$PY" -m idps.show.cli export "$OUT/five.zip" --site examples/sites/show_8x8.yaml \
    --plan examples/plans/show_8x8.yaml --min-sep 0.9 -o "$OUT/show" | tail -1
"$PY" - "$OUT/show/manifest.json" "$N" <<'PY'
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
  "$PY" px4/sitl/idps_stand_in.py "$OUT/show/manifest.json" "$N" "$OUT/truth.csv" log-only > "$OUT/standin.log" 2>&1 &
  PIDS+=($!); disown $!; TRACK=$OUT/truth.track.csv
  # every drone's truth link must reach the logger before the show (else no tracking for it)
  for ((t=0;;t++)); do
    missing=$("$PY" - "$OUT/truth.csv" "$N" <<'PY'
import sys
seen = set()
try:
    for line in open(sys.argv[1]).read().splitlines()[1:]:
        seen.add(line.split(",")[1])
except OSError:
    pass
print(" ".join(str(d) for d in range(1, int(sys.argv[2]) + 1) if str(d) not in seen))
PY
)
    [ -z "$missing" ] && break
    if [ $t -ge 60 ]; then
      echo "No data from drone(s) $missing on their truth links (udp 14800+): see $OUT/sitl.log and $OUT/standin.log"
      exit 1
    fi
    sleep 0.5 & wait $!
  done
else
  echo "== cameras and engine"
  "$PY" -m idps.sim px4 examples/scenarios/show_8x8_micro60.yaml --manifest "$OUT/show/manifest.json" \
      --duration 600 --workers "${WORKERS:-3}" -o "$OUT" > "$OUT/cams.log" 2>&1 &
  PIDS+=($!); disown $!
  while [ ! -s "$OUT/calibration.yaml" ]; do sleep 0.5; done; sleep 1
  "$PY" -m idps.engine.cli live --port 5600 --calib "$OUT/calibration.yaml" --site examples/sites/show_8x8.yaml \
      $L14 -o "$OUT/positions.csv" > "$OUT/engine.log" 2>&1 &
  PIDS+=($!); disown $!; TRACK=$OUT/track.csv
  echo "   waiting 30 s for the cameras to identify every drone"; sleep 30 & wait $!
  tail -1 "$OUT/cams.log"; tail -1 "$OUT/engine.log"
fi

echo "== upload and fly (about 80 s)"
"$PY" -m idps.show.cli upload "$OUT/show/manifest.json" $L --sitl | grep -c OK | sed 's/$/ drone(s) uploaded/'
# in the background and waited for, so a stop (TERM) is handled at once: the trap aborts the
# show and stops every helper (a foreground command would delay it to the end of the show)
"$PY" -m idps.show.cli start "$OUT/show/manifest.json" $L --lead 15 > "$OUT/start.txt" 2>&1 &
PIDS+=($!); wait $! || true
grep -E "denied|refused|Not started" "$OUT/start.txt" || true
for ((i=0;i<N;i++)); do
  f=$(grep -hE "Failsafe|stopped" "$B/rootfs/$i/out.log" | head -1 || true)
  [ -n "$f" ] && echo "   drone $((i+1)): $f"
done
[ "$MODE" = cameras ] && { grep "behind the clock" "$OUT/cams.log" | tail -1; grep "exposure" "$OUT/engine.log" | tail -1; }

grep -h "nothing on udp" "$OUT/standin.log" 2>/dev/null || true
echo "== result (distance from the show file)"
"$PY" px4/sitl/score_track.py "$TRACK"
