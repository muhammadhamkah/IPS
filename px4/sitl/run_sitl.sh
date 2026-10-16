#!/bin/bash
# Start N headless PX4 SITL quadcopters (SIH, no Gazebo) set up like IDPS show drones:
# position, height and heading only from external vision (IDPS), no GPS, compass, baro or rangefinder.
# Instance i has MAV_SYS_ID i+1 (= drone ID), MAVLink on udp 14540+i (from instance 10: 15540+i,
# as 14550 is every instance's ground-station port), and a ground-truth link
# on 14800+i for the IDPS stand-in (idps_stand_in.py) or the simulated cameras (idps-sim px4).
# The engine reaches each drone on PX4's onboard payload link (udp 14030+i).
# The IDPS position is the marker's, 8 cm above the drone's centre (EKF2_EV_POS_Z).
#
#   PX4_DIR=~/PX4-Autopilot px4/sitl/run_sitl.sh 3
#   EV_DELAY_MS=20 px4/sitl/run_sitl.sh 3       # closed loop: camera exposure -> engine -> PX4 delay
#   DRONE=micro60 px4/sitl/run_sitl.sh 5         # 60 g show quads instead of 1 kg
#   SITL_VISION=20 EV_DELAY_MS=20 px4/sitl/run_sitl.sh 5   # exact positions inside PX4, 20 ms late
#   SITL_VISION="28 2 120" EV_DELAY_MS=28 DRONE=micro60 px4/sitl/run_sitl.sh 5   # like the engine: 2 mm, 120 Hz
#   PX4_BUILD=px4_sitl_nolockstep ...          # PX4 on the real clock (camera loop; see px4/README.md)
# EKF2 fuses in the past by EKF2_DELAY_MAX and predicts forward with the IMU; it is kept just above
# the vision delay (PX4's 200 ms default would leave centimetres of IMU drift in every position).
# In the camera loop EV_DELAY_MS is what idps-engine live reports as "exposure to send" (plus the
# network on real drones); EV_HORIZON_MS overrides the horizon.
set -e
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
B=$PX4_DIR/build/${PX4_BUILD:-px4_sitl_default}
N=${1:-1}
[ -x "$B/bin/px4" ] || { echo "build first: make -C $PX4_DIR ${PX4_BUILD:-px4_sitl_default} EXTERNAL_MODULES_LOCATION=$(cd "$(dirname "$0")/.." && pwd)"; exit 1; }
echo "idps_show start" > "$B/etc/init.d-posix/airframes/10040_sihsim_quadx.post"
# PX4s left from an earlier flight hold /tmp/px4_lock-<i> ("PX4 server already running"): stop them,
# wait until they are gone (shutting down takes a few seconds), then force the stragglers.
pkill -x px4 2>/dev/null || true
for _ in $(seq 50); do pgrep -x px4 >/dev/null || break; sleep 0.2; done
pkill -9 -x px4 2>/dev/null || true
for _ in $(seq 25); do pgrep -x px4 >/dev/null || break; sleep 0.2; done
if pgrep -x px4 >/dev/null; then
  echo "earlier PX4 processes would not stop (pids: $(pgrep -x px4 | tr '\n' ' ')): stop them first, e.g. kill -9 $(pgrep -x px4 | tr '\n' ' ')" >&2
  exit 1
fi
sleep 0.5
export PX4_PARAM_EKF2_EV_CTRL=11 PX4_PARAM_EKF2_HGT_REF=3 PX4_PARAM_EKF2_GPS_CTRL=0 PX4_PARAM_EKF2_BARO_CTRL=0 PX4_PARAM_EKF2_RNG_CTRL=0 \
       PX4_PARAM_EKF2_MAG_TYPE=5 PX4_PARAM_SYS_HAS_MAG=0 PX4_PARAM_EKF2_EVP_NOISE=0.01 PX4_PARAM_EKF2_EVA_NOISE=0.05 \
       PX4_PARAM_EKF2_EV_POS_Z=-0.08 PX4_PARAM_EKF2_EV_DELAY=${EV_DELAY_MS:-0} \
       PX4_PARAM_EKF2_ASP_DELAY=0 PX4_PARAM_EKF2_OF_DELAY=0 PX4_PARAM_EKF2_RNG_DELAY=0 PX4_PARAM_EKF2_AVEL_DELAY=0 \
       PX4_PARAM_EKF2_DELAY_MAX=${EV_HORIZON_MS:-$(( ${EV_DELAY_MS:-0} + 10 ))}
# SIH's default drag (1 N per m/s on a 1 kg quad) is far above a real quad's; tighter position
# gains, as a tuned show drone would have, and a short take-off ramp (PX4 eases in over 3 s by
# default, while the show file climbs from its first second).
export PX4_PARAM_SIH_KDV=0.2 PX4_PARAM_MPC_XY_P=1.8 PX4_PARAM_MPC_XY_VEL_P_ACC=4.0 PX4_PARAM_MPC_XY_VEL_I_ACC=1.5 \
       PX4_PARAM_MPC_Z_P=1.5 PX4_PARAM_MPC_Z_VEL_P_ACC=6.0 \
       PX4_PARAM_MPC_TKO_RAMP_T=0.5 PX4_PARAM_SIM_BAT_MIN_PCT=100 PX4_PARAM_COM_LOW_BAT_ACT=0 \
       PX4_PARAM_SYS_HAS_BARO=0   # simulated battery never drains or warns into a failsafe; baro off (IDPS height)
# DRONE=micro60: a 60 g indoor show quad (65 mm frame: motors 2.3 cm from the axes, thrust/weight
# 3.5, 20 ms motors, inertia 3e-5 / 3e-5 / 5e-5 kg m^2) instead of SIH's 1 kg default, with
# show-drone limits (5 m/s up, 4 m/s down, 60 degree tilt) and rate gains scaled down for its
# ~10x larger angular authority.
# SIH inverts the inertia matrix only if det(100 I) >= FLT_EPSILON, which a real 60 g quad fails
# (the airframe then never rotates). Rotation depends only on torque / inertia, so inertia and
# torque arms (SIH_L_*, SIH_Q_MAX, SIH_KDW) are both 10x: identical dynamics, valid inverse.
# Tuned to track a show file within 2 cm (px4/README.md): EKF2 trusts the 200 Hz IDPS positions
# (fast output-predictor correction, IMU noise as flown), fast attitude and rate loops (attitude
# P 20, rate K 0.25: with P 10 the attitude lagged ~110 ms and the position loop circled after any
# jolt), a damped velocity loop (P 8, D 0.15, small I) and a vertical I term that carries the drag
# in long climbs and descents. IDPS_ACC_LEAD: the show's acceleration is fed ahead by this much.
# SIH adds a fixed-wing's in-flight IMU noise; IMU_NOISE scales it (needs px4/sitl/sih_noise_scale.patch).
if [ "${DRONE:-}" = micro60 ]; then
  export SIH_NOISE_SCALE=${IMU_NOISE:-0.1}
  export PX4_PARAM_SIH_MASS=0.06 PX4_PARAM_SIH_T_MAX=0.515 PX4_PARAM_SIH_T_TAU=0.02 PX4_PARAM_SIH_KDV=0.012 \
         PX4_PARAM_SIH_L_ROLL=0.23 PX4_PARAM_SIH_L_PITCH=0.23 PX4_PARAM_SIH_Q_MAX=0.03 \
         PX4_PARAM_SIH_IXX=0.0003 PX4_PARAM_SIH_IYY=0.0003 PX4_PARAM_SIH_IZZ=0.0005 PX4_PARAM_SIH_KDW=0.0003 \
         PX4_PARAM_MPC_THR_HOVER=0.29 PX4_PARAM_MC_ROLLRATE_K=${RATE_K:-0.25} PX4_PARAM_MC_PITCHRATE_K=${RATE_K:-0.25} \
         PX4_PARAM_MC_ROLLRATE_D=0.0015 PX4_PARAM_MC_PITCHRATE_D=0.0015 PX4_PARAM_IMU_DGYRO_CUTOFF=50 \
         PX4_PARAM_MC_YAWRATE_K=${YAW_K:-0.08} \
         PX4_PARAM_MPC_Z_VEL_MAX_UP=5 PX4_PARAM_MPC_Z_VEL_MAX_DN=4 PX4_PARAM_MPC_TILTMAX_AIR=60 \
         PX4_PARAM_MPC_ACC_HOR_MAX=15 PX4_PARAM_MPC_ACC_UP_MAX=10 PX4_PARAM_MPC_ACC_DOWN_MAX=8 \
         PX4_PARAM_FD_FAIL_R=80 PX4_PARAM_FD_FAIL_P=80 \
         PX4_PARAM_MC_ROLLRATE_MAX=720 PX4_PARAM_MC_PITCHRATE_MAX=720 PX4_PARAM_MC_ROLL_P=20 PX4_PARAM_MC_PITCH_P=20 \
         PX4_PARAM_MPC_XY_P=${XY_P:-4} PX4_PARAM_MPC_XY_VEL_P_ACC=${XY_VP:-8} PX4_PARAM_MPC_XY_VEL_I_ACC=${XY_VI:-0.5} \
         PX4_PARAM_MPC_XY_VEL_D_ACC=0.15 PX4_PARAM_MPC_Z_P=${Z_P:-4} PX4_PARAM_MPC_Z_VEL_P_ACC=${Z_VP:-12} \
         PX4_PARAM_MPC_Z_VEL_I_ACC=${Z_VI:-4} \
         PX4_PARAM_EKF2_EVP_NOISE=${EVP:-0.01} PX4_PARAM_EKF2_ACC_NOISE=1.5 \
         PX4_PARAM_EKF2_TAU_POS=0.1 PX4_PARAM_EKF2_TAU_VEL=0.1 PX4_PARAM_IDPS_ACC_LEAD=${ACC_LEAD:-0.1}
fi
link_port() { if [ $1 -lt 10 ]; then echo $((14540+$1)); else echo $((15540+$1)); fi; }   # drone index
launch() {   # drone index: start (or restart) that PX4 instance
  mkdir -p "$B/rootfs/$1"; cd "$B/rootfs/$1"
  rm -f parameters*.bson dataman
  # PX4 logging off (run_loop.sh's default): old flight logs are stale, and 2+ GB per drone fills a disk
  if [ "${PX4_PARAM_SDLOG_MODE:-}" = "-1" ]; then rm -rf log; fi
  PX4_SYS_AUTOSTART=10040 PX4_SIM_MODEL=sihsim_quadx nohup "$B/bin/px4" -i $1 -d "$B/etc" > out.log 2>&1 &
  PIDS[$1]=$!
}
T0=$SECONDS
for ((i=0;i<N;i++)); do launch $i; sleep 0.3; done      # a moment apart: many booting at once can hang one
# Configure each drone only once its whole startup script has run (the airframe's .post file,
# which starts idps_show, comes last), and check each setting took: on a slower machine (a Mac)
# a command sent too early reaches nothing and, before, failed silently.
# Run a PX4 command-line client, at most 5 s (they answer in ms): a client talking to a PX4 that hangs never returns
# (macOS has no `timeout`).
# Starting or stopping a MAVLink link waits on the simulated clock, which with many drones on a
# busy machine takes seconds: PX4C_S=30 for those.
px4c() {
  "$@" & local pid=$!
  ( for _ in $(seq $((${PX4C_S:-5} * 10))); do kill -0 $pid 2>/dev/null || exit 0; sleep 0.1; done
    kill -9 $pid 2>/dev/null ) &
  local watch=$!
  local rc=0; wait $pid || rc=$?
  kill $watch 2>/dev/null || true; wait $watch 2>/dev/null || true
  return $rc
}
link_up() {   # drone index, local port, remote port: PX4's log says that link runs (the log is
  local k      # written in chunks, so the line may come a moment late)
  for k in $(seq 10); do
    grep -q "udp port $2 remote port $3" "$B/rootfs/$1/out.log" 2>/dev/null && return 0
    sleep 0.5
  done
  return 1
}
fail() {   # drone index, message
  echo "PX4 drone $(($1+1)) (instance $1): $2. The end of its log ($B/rootfs/$1/out.log):" >&2
  tail -25 "$B/rootfs/$1/out.log" >&2
  exit 1
}
# All drones start at once; on a busy machine one now and then hangs in PX4's own startup (in
# lockstep, simulated time only moves once every part is up). Such a drone is started again (up
# to twice); its stuck log is kept as out.stuck.log.
for ((i=0;i<N;i++)); do
  tries=0
  for ((t=0;;t++)); do
    grep -q "Startup script returned" "$B/rootfs/$i/out.log" 2>/dev/null && break
    kill -0 "${PIDS[$i]}" 2>/dev/null || fail $i "PX4 stopped while starting"
    if [ $t -ge 60 ]; then
      [ $tries -ge 2 ] && fail $i "did not finish starting within 30 s, 3 times"
      tries=$((tries+1)); t=0
      echo "PX4 drone $((i+1)) did not finish starting within 30 s: starting it again ($tries of 2)"
      kill -9 "${PIDS[$i]}" 2>/dev/null || true; sleep 1
      cp "$B/rootfs/$i/out.log" "$B/rootfs/$i/out.stuck.log" 2>/dev/null || true
      launch $i
    fi
    sleep 0.5
  done
  for ((t=0;;t++)); do
    px4c "$B/bin/px4-idps_show" --instance $i status 2>/dev/null | grep -q "state:" && break
    [ $t -ge 20 ] && fail $i "the idps_show module is not running (is PX4 built with EXTERNAL_MODULES_LOCATION=.../IPS/px4?)"
    sleep 0.5
  done
  echo "PX4 drone $((i+1)) up after $((SECONDS - T0)) s"
done
for ((i=0;i<N;i++)); do
  if [ $i -ge 10 ]; then
    # PX4's startup sends the offboard link of every instance from 10 up to the same port (14549);
    # point it at this drone's own 15540+i (PX4 allows only 6 links, so it is restarted, not added)
    PX4C_S=30 px4c "$B/bin/px4-mavlink" --instance $i stop -u $((14580+i)) >/dev/null 2>&1 || true
    for ((t=0;;t++)); do
      PX4C_S=30 px4c "$B/bin/px4-mavlink" --instance $i start -x -u $((14580+i)) -r 4000000 -f -m onboard \
        -o $((15540+i)) >/dev/null 2>&1 && break
      link_up $i $((14580+i)) $((15540+i)) && break           # it started; the client just took long
      [ $t -ge 10 ] && fail $i "could not move its offboard link to udp $((15540+i))"
      sleep 0.5
    done
  fi
  # The engine's link (onboard payload, udp 14030+i): PX4 starts it at 4000 B/s full of telemetry,
  # which leaves its clock sync requests (TIMESYNC) under 2 a second. Only those and the heartbeat
  # here, so idps-engine live --timesync can ask for 50 a second.
  # (stopping takes a moment: until then the start fails with the port in use, so retry; the log
  # line to look for is the new link's, as the old one's says the same ports)
  PX4C_S=30 px4c "$B/bin/px4-mavlink" --instance $i stop -u $((14280+i)) >/dev/null 2>&1 || true
  for ((t=0;;t++)); do
    PX4C_S=30 px4c "$B/bin/px4-mavlink" --instance $i start -x -u $((14280+i)) -r 20000 -f -m custom \
      -o $((14030+i)) >/dev/null 2>&1
    grep -q "Custom, data rate: 20000 B/s on udp port $((14280+i)) remote port $((14030+i))" \
      "$B/rootfs/$i/out.log" 2>/dev/null && break
    [ $t -ge 20 ] && fail $i "could not restart its engine link (udp $((14030+i)))"
    sleep 0.5
  done
  PX4C_S=30 px4c "$B/bin/px4-mavlink" --instance $i start -u $((14700+i)) -o $((14800+i)) -m custom -r 400000 \
    >/dev/null 2>&1 || link_up $i $((14700+i)) $((14800+i)) \
    || fail $i "could not open its truth link (MAVLink on udp $((14700+i)))"
  for s in "HIL_STATE_QUATERNION 100" "DEBUG_VECT 200" "DEBUG_FLOAT_ARRAY 100"; do   # truth; tracking error
    set -- $s
    for ((t=0;;t++)); do
      px4c "$B/bin/px4-mavlink" --instance $i stream -u $((14700+i)) -s $1 -r $2 >/dev/null 2>&1 && break
      [ $t -ge 10 ] && fail $i "could not stream $1 on its truth link"
      sleep 0.5
    done
  done
  # SITL_VISION="<delay ms> [noise mm [rate Hz]]": IDPS inside PX4 (idps_show sitl_vision) instead of over MAVLink
  if [ -n "${SITL_VISION:-}" ]; then
    for ((t=0;;t++)); do
      px4c "$B/bin/px4-idps_show" --instance $i sitl_vision on $SITL_VISION >/dev/null 2>&1
      px4c "$B/bin/px4-listener" --instance $i vehicle_visual_odometry -n 1 2>/dev/null | grep -q "position" && break
      [ $t -ge 10 ] && fail $i "positions (sitl_vision) did not start"
      sleep 0.5
    done
  fi
done
echo "$N drone(s) up. Links: $(for ((i=0;i<N;i++)); do printf -- '--link %d=udpin:127.0.0.1:%d ' $((i+1)) $(link_port $i); done)"
