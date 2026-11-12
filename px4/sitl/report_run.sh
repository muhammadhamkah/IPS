#!/usr/bin/env bash
# What happened in a full-loop flight, in one paste: px4/sitl/report_run.sh [run folder]
# (default: the newest folder in out/px4_runs). Prints the start-up messages, the engine's and the
# cameras' status over the flight, each PX4's warnings (from its own log, which the next flight
# overwrites: run this before flying again), and the score.
set -u
cd "$(dirname "$0")/../.."
RUN=${1:-$(ls -td out/px4_runs/*/ 2>/dev/null | head -1)}
RUN=${RUN%/}
[ -d "$RUN" ] || { echo "no run folder (give one: px4/sitl/report_run.sh out/px4_runs/<folder>)"; exit 1; }
B=${PX4_DIR:-$HOME/PX4-Autopilot}/build/${PX4_BUILD:-px4_sitl_default}
PY=${PYTHON:-python3}
echo "== run: $RUN"
echo "== start"
cut -c1-300 "$RUN/start.txt" 2>/dev/null | grep -v "^show time 0" | tail -40
echo "== engine (every 5th status line, then the last 3)"
grep "exposure to send" "$RUN/engine.log" 2>/dev/null | awk 'NR%5==1' | sed -E 's/drones with positions: \[[^]]*\]/drones: ../' | cut -c1-230
grep "exposure to send" "$RUN/engine.log" 2>/dev/null | tail -3 | cut -c1-330
echo "== cameras (worst 'behind the clock' lines, then the last)"
grep "behind the clock" "$RUN/cams.log" 2>/dev/null | sort -t, -k3 -n | awk '{print}' | sed -E 's/.*median ([0-9.]+), p99 ([0-9.]+), max ([0-9.]+).*/\3 &/' | sort -rn | head -3 | cut -d' ' -f2- | cut -c1-160
grep "behind the clock" "$RUN/cams.log" 2>/dev/null | tail -1 | cut -c1-160
for f in fell_behind engine_behind; do [ -f "$RUN/$f" ] && echo "   $f: $(cat "$RUN/$f") ms"; done
echo "== each PX4 (warnings and show messages, in order)"
N=$(ls "$RUN/show"/drone_*.trj 2>/dev/null | wc -l | tr -d ' ')
for ((i=0;i<N;i++)); do
  f="$B/rootfs/$i/out.log"
  [ -f "$f" ] || continue
  echo "   drone $((i+1)): $(grep -E "WARN|ERROR|idps_show\]|Armed|Disarmed|Takeoff|Landing" "$f" | grep -v "Preflight Fail: horizontal position unstable\|loaded, crc" \
      | sed -E 's/^(INFO|WARN|ERROR) +//; s/\x1b\[[0-9;]*m//g; s/[[:space:]]+$//' | tail -14 | tr '\n' '|' | cut -c1-900)"
done
echo "== score"
T="$RUN/track.csv"; [ -f "$T" ] || T="$RUN/truth_track.csv"
"$PY" px4/sitl/score_track.py "$T" 2>/dev/null | tail -30
[ -f "$RUN/positions.csv" ] && { "$PY" px4/sitl/explain_run.py "$RUN" 2>/dev/null | head -40 | cut -c1-300; }
