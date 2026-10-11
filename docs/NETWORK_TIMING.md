# Network and clock synchronisation

## Topology (prototype)

```
 OV9281 ─CSI─ Pi 3B ─┐
 OV9281 ─CSI─ Pi 3B ─┤  Ethernet cables (the Pi 3B's port runs at 100 Mbit)
 OV9281 ─CSI─ Pi 3B ─┼──── gigabit switch ──── PC (positioning engine + dashboard)
        …            │                    └─── PTP grandmaster (a Pi, or a Linux box)
```

* **One Pi per camera.** It detects the LED blobs and sends the centroids to the PC
  (`FRAME_OBS` packets, docs/PROTOCOL.md). It does not compute positions.
* **The PC** triangulates and tracks, then sends drone positions on, in NED (D18).
* **Bandwidth is not a concern.** The simulated wired lab sends about 0.17 Mbit/s per station
  with 3 drones; a full swarm stays under about 1 Mbit/s per station. The Pi 3B's 100 Mbit port
  and the switch are nearly idle, so the 3B's slow port does not matter for bandwidth.

## Why clocks matter

Each Pi stamps every frame with **its own clock**, at mid-exposure. The engine pairs
observations by those timestamps, never by arrival time.

* **What matters is the difference between stations.** If two stations' clocks disagree by
  `dt`, a drone moving at `v` is placed about `v·dt/2` off. An offset shared by every station
  only adds latency.
* **Example:** at 5 m/s, 100 µs of skew costs about 0.25 mm, and 2 ms costs about 5 mm.

## Simulated options (`idps-sim clocks`)

Results for 16 stations over 900 s from power-on. Skew is between any two stations; the error
column is the timing part of the position error at 5 m/s.

| Profile | Settled after | Skew p50 | Skew p95 | Skew p99 | Error at 5 m/s |
|---|---|---|---|---|---|
| **Pi 3B, wired, PTP, software timestamps** (`pi3b_ptp_sw`) | ~35 s | 19 µs | 60 µs | 91 µs | **0.15 mm** |
| Same, grandmaster with hardware timestamps | ~27 s | 16 µs | 53 µs | 83 µs | 0.13 mm |
| Pi 3B, wired, chrony/NTP (2 s polling) | slow (minutes) | 30 µs | 107 µs | 170 µs | 0.27 mm |
| Pi 5 / CM4, hardware PTP | < 5 s | 0.3 µs | 0.8 µs | 1.1 µs | < 0.01 mm |
| Pi 3B on 2.4 GHz Wi-Fi, chrony | ~5 min | 650 µs | 2.2 ms | 3.0 ms | **5.4 mm** |
| No synchronisation | never | 0.3 s and growing | | | metres |

**What the model includes** (`idps/sim/clocksync.py`):

* **Crystals:** each Pi's crystal is about 20 ppm off and wanders slowly.
* **Message exchange:** each two-way PTP or NTP exchange is simulated with:
  * the Pi 3B's Ethernet port, which inside the board is connected to the processor through a
    USB 2.0 chip (SMSC LAN9514), so packets reach the CPU in 125 µs USB polling steps;
  * software-timestamp noise;
  * late-timestamp spikes when the CPU is busy with image processing;
  * switch delay variation;
  * a fixed **send/receive path asymmetry** per station.
* **Servo:** a `ptp4l`-style PI servo with start-up frequency estimation and a path-delay
  median filter.

**Path asymmetry is a hard limit.** It biases each clock by half the asymmetry, and the
protocol cannot detect it: the offset `ptp4l` reports stays near zero while the true error
does not (see the test `test_path_asymmetry_is_invisible_to_ptp`). Only hardware timestamps,
or a hardware trigger, remove it.

**Conclusions for the prototype:**

* **Wired, software PTP is enough.** About 60 µs p95 between stations is inside the planner's
  *tight* error budget (σ_t = 50 µs), and costs about 0.15 mm at 5 m/s.
* **Wi-Fi is not.** About 2 ms skew costs about 5 mm, half the 10 mm budget.
* **Wait for the clocks to settle.** After power-on, give PTP about a minute before
  calibrating or flying. The dashboard will show each station's sync state from its HEALTH
  packets.

## Setting up the real Pis

**Every camera Pi (slave):**

```
sudo apt install linuxptp
sudo ptp4l -i eth0 -S -s -m          # -S software timestamps (the Pi 3B has no hardware PTP), -s slave only
```

With software timestamps `ptp4l` steers the system clock directly (no `phc2sys`). The station
software converts each frame's `SensorTimestamp` (a monotonic kernel clock) to the
PTP-disciplined `CLOCK_REALTIME` at every frame.

**Grandmaster.** One machine on the switch runs `ptp4l -i eth0 -S -m` as master (priority set
so it wins):

* **A Pi is fine.** The table above uses a Pi as grandmaster. A Pi 4/5 or a Linux box with an
  Intel i210 (hardware timestamps) is a small improvement.
* **macOS cannot run `ptp4l`.** If the PC is a Mac, use a Pi as grandmaster. The Mac does not
  need to be in the PTP domain: the engine works in the cameras' shared timebase.
* **The grandmaster's own clock accuracy does not matter.** Only the stations' agreement with
  each other does.

**Settings to start from:** `logSyncInterval -2` (4 Hz), `delay_filter_length 10`, and the
software servo defaults (`pi_proportional_const 0.1`, `pi_integral_const 0.001`). These are
what the `pi3b_ptp_sw` profile models.

## Upgrades, in order of value

1. **Hardware trigger:** one GPIO line from a master to every OV9281's FSIN pin (RISKS R13).
   All cameras then expose at the same instant regardless of clock error, and timestamps only
   need to identify the frame.
2. **Pi 5 or CM4 stations** with hardware PTP: under 1 µs, plus the CPU headroom of R4.

## Try it

```
idps-sim clocks                                   # compare every profile, 16 stations, 15 min
idps-sim clocks --stations 30 --profile pi3b_ptp_sw --speed 8
idps-sim run examples/scenarios/lab_pi3b_ethernet.yaml -o out/eth
                                                  # full simulation; out/eth/clocks.csv holds each
                                                  # Pi's true clock error and the offset ptp4l reports
```
