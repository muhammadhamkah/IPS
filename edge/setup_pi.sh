#!/usr/bin/env bash
# Set up a Raspberry Pi 5 as an IDPS camera station (docs/STATION.md). Run on the Pi, from a
# copy of the IPS repository, on Raspberry Pi OS Lite (64-bit, Bookworm or later):
#
#   sudo edge/setup_pi.sh --id 1 --engine 192.168.1.10 --grandmaster   # the station that keeps the time
#   sudo edge/setup_pi.sh --id 2 --engine 192.168.1.10                  # every other station
#
# It installs the build tools, libcamera and linuxptp; enables the OV9281 camera; builds and
# installs idps-station; sets up PTP clock sync (ptp4l + phc2sys); and installs the station as a
# service that starts on boot. Reboot afterwards (the camera overlay needs it).
#
# Options: --id N (required)  --engine HOST[:PORT] (required)  --grandmaster  --cam0 (camera on the
#          CAM0 connector, default CAM1)  --iface eth0  --no-service (build and set up only)
set -euo pipefail

ID="" ENGINE="" GM=0 CAM="" IFACE=eth0 SERVICE=1
while [ $# -gt 0 ]; do
  case "$1" in
    --id) ID="$2"; shift 2 ;;
    --engine) ENGINE="$2"; shift 2 ;;
    --grandmaster) GM=1; shift ;;
    --cam0) CAM=",cam0"; shift ;;
    --iface) IFACE="$2"; shift 2 ;;
    --no-service) SERVICE=0; shift ;;
    -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
[ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }
[ -n "$ID" ] && [ -n "$ENGINE" ] || { echo "needs --id N and --engine HOST[:PORT]" >&2; exit 2; }
REPO=$(cd "$(dirname "$0")/.." && pwd)
CONFIG=/boot/firmware/config.txt
[ -f "$CONFIG" ] || CONFIG=/boot/config.txt

echo "== packages"
apt-get update -q
apt-get install -y -q build-essential cmake pkg-config libcamera-dev linuxptp ethtool

if [ -n "$CAM" ]; then CAMNAME=CAM0; else CAMNAME=CAM1; fi
echo "== camera: OV9281 on the $CAMNAME connector"
if ! grep -q "^dtoverlay=ov9281" "$CONFIG"; then
  sed -i 's/^camera_auto_detect=1/camera_auto_detect=0/' "$CONFIG"
  grep -q "^camera_auto_detect=0" "$CONFIG" || echo "camera_auto_detect=0" >> "$CONFIG"
  echo "dtoverlay=ov9281${CAM}" >> "$CONFIG"
  NEED_REBOOT=1
fi

echo "== build idps-station"
cmake -S "$REPO/edge" -B "$REPO/edge/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$REPO/edge/build" -j4
cmake --install "$REPO/edge/build" --prefix /usr/local
/usr/local/bin/idps-station version

echo "== clock sync (PTP on $IFACE)"
if ethtool -T "$IFACE" | grep -q "hardware-transmit"; then TS=hardware; else TS=software; fi
echo "   $IFACE timestamps: $TS"
mkdir -p /etc/linuxptp
if [ "$GM" = 1 ]; then
  # the grandmaster: its clock (from NTP/the internet, or the Mac) is everyone's time; a low
  # priority1 makes it win the election
  cat > /etc/linuxptp/idps-ptp4l.conf <<EOF
[global]
priority1 64
time_stamping $TS
EOF
  PHC2SYS_ARGS="-a -r -r"          # system clock -> the network port's clock
else
  cat > /etc/linuxptp/idps-ptp4l.conf <<EOF
[global]
slaveOnly 1
time_stamping $TS
EOF
  PHC2SYS_ARGS="-a -r"             # the network port's clock -> system clock
fi
cat > /etc/systemd/system/idps-ptp4l.service <<EOF
[Unit]
Description=IDPS PTP clock sync (ptp4l)
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/idps-ptp4l.conf -i $IFACE
Restart=always

[Install]
WantedBy=multi-user.target
EOF
cat > /etc/systemd/system/idps-phc2sys.service <<EOF
[Unit]
Description=IDPS system clock <-> PTP clock (phc2sys)
After=idps-ptp4l.service
Requires=idps-ptp4l.service

[Service]
ExecStart=/usr/sbin/phc2sys $PHC2SYS_ARGS
Restart=always

[Install]
WantedBy=multi-user.target
EOF
if [ "$GM" = 1 ]; then
  # the Mac (engine) sets its clock from the grandmaster over NTP
  apt-get install -y -q chrony
  grep -q "^allow" /etc/chrony/chrony.conf || echo "allow all" >> /etc/chrony/chrony.conf
  grep -q "^local stratum" /etc/chrony/chrony.conf || echo "local stratum 8" >> /etc/chrony/chrony.conf
  systemctl restart chrony
else
  # the system clock now follows PTP: no NTP fighting it
  systemctl disable --now systemd-timesyncd 2>/dev/null || true
  systemctl disable --now chrony 2>/dev/null || true
fi
systemctl daemon-reload
systemctl enable --now idps-ptp4l idps-phc2sys

# full CPU speed all the time (frame timing must not wait for the clock to ramp up)
cat > /etc/systemd/system/idps-performance.service <<EOF
[Unit]
Description=IDPS: CPU governor performance

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > \$g; done'

[Install]
WantedBy=multi-user.target
EOF
systemctl enable --now idps-performance

if [ "$SERVICE" = 1 ]; then
  echo "== station service (station $ID -> $ENGINE)"
  cat > /etc/default/idps-station <<EOF
# idps-station options (idps-station --help); restart with: sudo systemctl restart idps-station
IDPS_ARGS="--engine $ENGINE --id $ID --realtime --cpu 3"
EOF
  cat > /etc/systemd/system/idps-station.service <<EOF
[Unit]
Description=IDPS camera station
After=idps-phc2sys.service network-online.target
Wants=network-online.target

[Service]
EnvironmentFile=/etc/default/idps-station
ExecStart=/usr/local/bin/idps-station run \$IDPS_ARGS
Restart=always
RestartSec=2
Nice=-10

[Install]
WantedBy=multi-user.target
EOF
  systemctl daemon-reload
  systemctl enable idps-station
  [ -n "${NEED_REBOOT:-}" ] || systemctl restart idps-station
fi

echo
echo "Done. Station $ID sends to $ENGINE."
[ -n "${NEED_REBOOT:-}" ] && echo "Reboot now (sudo reboot): the camera overlay was added to $CONFIG."
echo "Check:   idps-station snapshot --out /tmp/frame.pgm      (stop the service first: sudo systemctl stop idps-station)"
echo "         idps-station bench --seconds 1800              (the 30-minute go/no-go)"
echo "         journalctl -u idps-station -f                  (the running station)"
