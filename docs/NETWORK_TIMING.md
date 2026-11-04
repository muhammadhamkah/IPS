# Network and clock synchronisation

## Topology (decision D16: Raspberry Pi 5 stations)

```
 OV9281 ─CSI─ Pi 5 ─┐
 OV9281 ─CSI─ Pi 5 ─┤  one Ethernet cable each: gigabit data + power (PoE+)
 OV9281 ─CSI─ Pi 5 ─┼──── gigabit PoE+ switch ──── PC (positioning engine + dashboard)
        …           │
 (camera 1's Pi 5 is also the PTP grandmaster, or a spare Pi 5)
```

* **One Pi 5 per camera.** It detects the LED blobs and sends their centroids to the PC
  (`FRAME_OBS` packets, docs/PROTOCOL.md). It does not compute positions.
* **The PC** triangulates and tracks, then sends drone positions on, in NED (D18).
* **Bandwidth is not a concern.** The simulated lab sends about 0.17 Mbit/s per station with 3
  drones; a full swarm stays under about 1 Mbit/s per station. The Pi 5's gigabit port and the
  switch are nearly idle.
* **Power.** Each station is powered over its Ethernet cable (PoE+, see docs/HARDWARE.md for the
  power budget), so a camera on a truss or a 20 m mast needs a single cable.

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
| **Pi 5, wired, PTP with hardware timestamps** (`pi5_ptp_hw`, chosen) | < 5 s | 0.3 µs | **0.8 µs** | 1.1 µs | **< 0.01 mm** |
| Pi 5, wired, PTP with software timestamps (fallback, `pi5_ptp_sw`) | ~40 s | 3.6 µs | 12 µs | 19 µs | 0.03 mm |
| Pi 5, wired, chrony/NTP (2 s polling) | minutes | 8 µs | 33 µs | 49 µs | 0.08 mm |
| Pi 5 on Wi-Fi, chrony | ~5 min | 540 µs | 2.0 ms | 2.9 ms | **5.0 mm** |
| *For comparison: Pi 3B, wired, software PTP* | ~35 s | 19 µs | 60 µs | 91 µs | 0.15 mm |
| No synchronisation | never | 0.3 s and growing | | | metres |

**What the model includes** (`idps/sim/clocksync.py`):

* **Crystals:** each Pi's crystal is about 20 ppm off and wanders slowly.
* **Message exchange:** each two-way PTP or NTP exchange is simulated with:
  * timestamp noise where the timestamp is taken: tens of ns in hardware (the Pi 5's Ethernet
    MAC), microseconds in software (kernel and interrupt latency);
  * late-timestamp spikes when the CPU is busy with image processing (software timestamps);
  * switch delay variation;
  * a fixed **send/receive path asymmetry** per station.
* **Servo:** a `ptp4l`-style PI servo with start-up frequency estimation and a path-delay
  median filter.

**Path asymmetry is a hard limit.** It biases each clock by half the asymmetry, and the
protocol cannot detect it: the offset `ptp4l` reports stays near zero while the true error
does not (see the test `test_path_asymmetry_is_invisible_to_ptp`). Hardware timestamps shrink
it to nanoseconds, which is one reason to use them.

**Conclusions:**

* **Hardware PTP on the Pi 5** puts the cameras within about 1 µs of each other: timing stops
  being part of the error budget.
* **If hardware timestamping turns out to be unavailable** on the first board (driver or kernel),
  software PTP on the Pi 5 still gives about 12 µs, which is fine.
* **Never Wi-Fi.** About 2 ms skew costs about 5 mm, half the 10 mm budget.
* **After power-on**, wait for every station's sync state to show *locked* before calibrating or
  flying: seconds with hardware PTP, about a minute with software PTP. The dashboard will show
  it from the HEALTH packets.

## Setting up the real Pis

**First check the board supports hardware timestamps:**

```
ethtool -T eth0          # want: hardware-transmit, hardware-receive and a "PTP Hardware Clock" index
```

**Every camera Pi (slave):**

```
sudo apt install linuxptp
sudo ptp4l -i eth0 -H -s -m                          # -H hardware timestamps, -s slave only
sudo phc2sys -s eth0 -c CLOCK_REALTIME -w -m         # copy the Ethernet clock to the system clock
```

With hardware timestamps `ptp4l` disciplines the Ethernet chip's own clock, and `phc2sys` keeps
the system clock on it (well under 1 µs on the same board). The station software converts each
frame's `SensorTimestamp` (a monotonic kernel clock) to that PTP-disciplined time at every
frame. Fallback without hardware timestamps: `ptp4l -i eth0 -S -s -m` and no `phc2sys`.

**Grandmaster.** One Pi runs `ptp4l -i eth0 -H -m` as master (set `priority1` lower so it wins):

* **Camera 1's Pi can do it.** It costs almost nothing extra. A spare Pi 5 also works and keeps
  the master running when camera 1 is swapped out.
* **macOS cannot run `ptp4l`.** The Mac does not need to be in the PTP domain: the engine works
  in the cameras' shared timebase.
* **The grandmaster's own clock accuracy does not matter.** Only the stations' agreement with
  each other does.

**Settings to start from:** `logSyncInterval -3` (8 Hz) and the hardware servo defaults
(`pi_proportional_const 0.7`, `pi_integral_const 0.3`). These are what the `pi5_ptp_hw`
profile models.

## Further upgrade

**Hardware trigger:** one GPIO line from a master to every OV9281's FSIN pin (RISKS R13).

* All cameras then expose at the same instant. With free-running cameras, the engine has to
  interpolate between frames taken up to half a frame apart (about 4 ms at 120 fps).
* With 1 µs clocks this is a refinement, not a necessity. Choose camera modules that expose
  the trigger pin so the option stays open.

## Try it

```
idps-sim clocks                                   # compare every profile, 16 stations, 15 min
idps-sim clocks --stations 30 --profile pi5_ptp_hw --profile pi5_ptp_sw --speed 8
idps-sim run examples/scenarios/lab_pi5_ethernet.yaml -o out/eth
                                                  # full simulation; out/eth/clocks.csv holds each
                                                  # Pi's true clock error and the offset ptp4l reports
idps-sim run examples/scenarios/lab_wifi.yaml -o out/wifi      # the same lab on Wi-Fi, for comparison
```
