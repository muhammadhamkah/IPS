#!/bin/bash
# Start N headless PX4 SITL quadcopters (SIH, no Gazebo) set up like IDPS show drones:
# position, height and heading only from external vision (IDPS), no GPS, compass or baro.
# Instance i has MAV_SYS_ID i+1 (= drone ID), MAVLink on udp 14540+i, and a ground-truth link
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
pkill -x px4 2>/dev/null || true; sleep 1
export PX4_PARAM_EKF2_EV_CTRL=11 PX4_PARAM_EKF2_HGT_REF=3 PX4_PARAM_EKF2_GPS_CTRL=0 PX4_PARAM_EKF2_BARO_CTRL=0 \
       PX4_PARAM_EKF2_MAG_TYPE=5 PX4_PARAM_SYS_HAS_MAG=0 PX4_PARAM_EKF2_EVP_NOISE=0.01 PX4_PARAM_EKF2_EVA_NOISE=0.05 \
       PX4_PARAM_EKF2_EV_POS_Z=-0.08 PX4_PARAM_EKF2_EV_DELAY=${EV_DELAY_MS:-0} \
       PX4_PARAM_EKF2_ASP_DELAY=0 PX4_PARAM_EKF2_OF_DELAY=0 PX4_PARAM_EKF2_RNG_DELAY=0 PX4_PARAM_EKF2_AVEL_DELAY=0 \
       PX4_PARAM_EKF2_DELAY_MAX=${EV_HORIZON_MS:-$(( ${EV_DELAY_MS:-0} + 10 ))}
# SIH's default drag (1 N per m/s on a 1 kg quad) is far above a real quad's; tighter position
# gains, as a tuned show drone would have, and a short take-off ramp (PX4 eases in over 3 s by
# default, while the show file climbs from its first second).
export PX4_PARAM_SIH_KDV=0.2 PX4_PARAM_MPC_XY_P=1.8 PX4_PARAM_MPC_XY_VEL_P_ACC=4.0 PX4_PARAM_MPC_XY_VEL_I_ACC=1.5 \
       PX4_PARAM_MPC_Z_P=1.5 PX4_PARAM_MPC_Z_VEL_P_ACC=6.0 \
       PX4_PARAM_MPC_TKO_RAMP_T=0.5 PX4_PARAM_SIM_BAT_MIN_PCT=100   # simulated battery never drains (no low-battery failsafe)
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
for ((i=0;i<N;i++)); do
  mkdir -p "$B/rootfs/$i"; cd "$B/rootfs/$i"
  rm -f parameters*.bson dataman
  PX4_SYS_AUTOSTART=10040 PX4_SIM_MODEL=sihsim_quadx nohup "$B/bin/px4" -i $i -d "$B/etc" > out.log 2>&1 &
done
sleep 10
for ((i=0;i<N;i++)); do
  "$B/bin/px4-mavlink" --instance $i start -u $((14700+i)) -o $((14800+i)) -m custom -r 400000 >/dev/null 2>&1
  "$B/bin/px4-mavlink" --instance $i stream -u $((14700+i)) -s HIL_STATE_QUATERNION -r 100 >/dev/null 2>&1
  "$B/bin/px4-mavlink" --instance $i stream -u $((14700+i)) -s DEBUG_VECT -r 200 >/dev/null 2>&1   # exact truth (idps_show)
  "$B/bin/px4-mavlink" --instance $i stream -u $((14700+i)) -s DEBUG_FLOAT_ARRAY -r 100 >/dev/null 2>&1   # tracking error
  # SITL_VISION="<delay ms> [noise mm [rate Hz]]": IDPS inside PX4 (idps_show sitl_vision) instead of over MAVLink
  [ -n "${SITL_VISION:-}" ] && "$B/bin/px4-idps_show" --instance $i sitl_vision on $SITL_VISION >/dev/null 2>&1
done
echo "$N drone(s) up. Links: $(for ((i=0;i<N;i++)); do printf -- '--link %d=udpin:127.0.0.1:%d ' $((i+1)) $((14540+i)); done)"
