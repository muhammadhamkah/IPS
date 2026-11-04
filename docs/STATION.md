# The camera station (`edge/`, `idps-station`)

A camera station is a Raspberry Pi 5 with an OV9281 global-shutter camera (docs/HARDWARE.md).
Its program, `idps-station`, does four things for every frame, 120 times a second:

1. **Capture:** a raw frame from the camera (libcamera, no image processing), at a fixed
   exposure, gain and frame rate.
2. **Detect:** find the LED dots and measure each one's centre to a few hundredths of a pixel
   (`edge/src/detect.hpp`).
3. **Timestamp:** the middle of the exposure, in the shared PTP time of all stations.
4. **Send:** one `FRAME_OBS` packet to the engine on the PC (docs/PROTOCOL.md), even with no
   dots, plus a `HEALTH` packet once a second.

The engine cannot tell a real station from the simulated ones: they send the same packets.

## What is in `edge/`

| File | What |
|---|---|
| `src/detect.hpp` | The dot detector. `idps/station/detect.py` is the same algorithm in Python; the tests check they agree exactly |
| `src/packet.hpp` | `FRAME_OBS` / `HEALTH` packets, byte for byte the same as `idps/protocol/packets.py` |
| `src/source_libcamera.cpp` | The camera: raw stream (8-bit `R8` preferred; 10-bit packed or 16-bit as fallbacks), auto exposure off |
| `src/source_test.hpp` | Frames without a camera: synthetic dots, or a frame file rendered on the PC |
| `src/station.cpp` | The program: `run`, `bench`, `snapshot`, `focus` |
| `src/sysinfo.hpp` | Temperature, throttling, CPU load, PTP state, hardware timestamps |
| `setup_pi.sh` | Turns a fresh Raspberry Pi OS Lite into a station |
| `CMakeLists.txt` | Build (camera support when `libcamera-dev` is installed) |

On the PC side, `idps/station/` has the Python detector, a renderer for synthetic camera images
(`render.py`), and `filming.py`, which makes the simulator render real images and run them
through the station's C++ detector.

## Setting up a station

1. **Flash** Raspberry Pi OS Lite (64-bit) onto the SD card (Raspberry Pi Imager: set the
   hostname, user and SSH there). Connect the camera ribbon to the **CAM1** connector (or use
   `--cam0`), Ethernet to the switch.
2. **Copy the repository** to the Pi (`git clone`), then:
   ```
   sudo edge/setup_pi.sh --id 1 --engine <Mac's IP> --grandmaster    # station 1 keeps the time
   sudo edge/setup_pi.sh --id 2 --engine <Mac's IP>                  # every other station
   sudo reboot
   ```
   It installs the build tools, libcamera and linuxptp, enables the camera, builds and installs
   `idps-station`, sets up PTP (`ptp4l` + `phc2sys` in automatic mode), and starts the station
   on boot.
3. **Set the Mac's clock from the grandmaster:** System Settings → General → Date & Time →
   "Set time and date automatically" with the source set to the grandmaster Pi's IP, or once:
   `sudo sntp -sS <grandmaster IP>`. The engine compares packet times with its own clock (it
   drops positions that are too old, and stamps them for the drones), so the Mac must be within
   a millisecond or two of the stations.

## The first day with the first station

Stop the service first (`sudo systemctl stop idps-station`), then:

1. **Does the camera work?** `idps-station snapshot --out frame.pgm` saves one raw image and
   lists the dots it found. Open the `.pgm` on the Mac. With the IR filter on, the room should be
   nearly black and the LEDs bright dots.
2. **Focus and exposure:** `idps-station focus` prints, every second, how many dots it sees,
   their size and brightness. Turn the lens until the dots are about **3–5 px across**, and set
   `--exposure-us` / `--gain` so the peaks stay below clipping (255) at the closest marker
   distance. Then lock the lens (lock screw or a drop of thread-locker).
3. **Go / no-go:** `idps-station bench --seconds 1800` (30 minutes, the Phase 2 soak test). It
   reports:
   * detection time against the number of dots (target: 99 % of frames within 4 ms, up to 50 dots);
   * the real frame rate and dropped frames (targets: 120 fps, under 0.01 % dropped);
   * the time from a frame's timestamp to its packet (target: 99 % within 6 ms);
   * whether the camera driver stamps frames in `CLOCK_MONOTONIC` (as the station assumes);
   * CPU temperature and any throttling (power supply or cooling);
   * whether the network port has hardware PTP timestamps, and the PTP state.

   It ends with **GO** or **NO-GO**. Send the whole printout: it decides whether to order the
   other stations, or to change the plan (90 fps, cropping, two cameras per Pi).

Without a camera, any of these run on synthetic frames: add `--source synthetic` (or
`--source file:frames.idpsfrm` for frames rendered with `idps.station.render.write_frames`).

## Running

`idps-station run --engine <Mac IP> --id N` (the service runs this). Useful options:

| Option | Default | What |
|---|---|---|
| `--fps`, `--exposure-us`, `--gain` | 120, 500, 1.0 | Capture |
| `--bits 8\|10` | 8 | 8-bit raw is half the data; 10-bit if the dots need the range |
| `--threshold`, `--threshold-low` | 60 / 30 (8-bit) | Seed and dot levels, raw DN above zero |
| `--ts-offset-us` | 0 | Driver timestamp → end of exposure (see below) |
| `--sync auto\|ptp\|free` | auto | `auto` asks ptp4l whether the clock is locked |
| `--realtime`, `--cpu N` | off | Real-time priority and a dedicated core for detection (the service uses both) |

On the engine side nothing changes: `idps-engine live --port 5600 --calib … --site …` receives
the stations exactly as it receives the simulator.

## Timestamps

Each packet carries the **middle of the exposure** in the stations' shared time:

`t_mid = SensorTimestamp + ts_offset − exposure / 2`, moved from `CLOCK_MONOTONIC` to
`CLOCK_REALTIME` (which `phc2sys` keeps on PTP time) at the moment of each frame.

* libcamera's `SensorTimestamp` on the Pi is taken when the frame starts arriving from the
  sensor. For a global-shutter sensor that is just after the exposure ends, plus a small fixed
  delay. `--ts-offset-us` corrects that delay. It is **measured, not assumed**, in Phase 2 with
  an LED flashed at known times (VALIDATION.md T5); until then it is 0. Because every station has
  the same delay, a wrong value only adds latency; it does not move the stations apart.
* The bench checks the driver's clock: frames should arrive a few milliseconds after their
  timestamp. If they don't, the timestamp is in another clock and the bench says so.

## Tested so far, and what only the hardware can tell

**Tested here (tests/test_station.py), without a Pi:**
* **Detector:** the C++ and Python detectors give identical dots on 8- and 10-bit frames.
* **Accuracy:** dot centres within 0.05 px (median) of the true spot centres, with sensor noise.
* **Flags:** edge, clipped and merged dots are flagged; lamps (too large) are ignored.
* **Packets:** the C++ packets are byte for byte the Python ones.
* **The program:** `idps-station run` sends one packet per frame at 120 fps and HEALTH every
  second; `snapshot` and `bench` work, on synthetic and file frames.
* **The whole chain:** simulated cameras render images, the station detector finds the dots,
  and the engine identifies all three lab drones in 3 s at 1.7–2.2 mm median error, the same as
  with the simulator's ideal dots.
* **Speed:** about 0.05–0.3 ms per 1280×800 frame on a PC. The Pi 5 is slower; the bench will
  say by how much.

**Written but untested (needs the Pi and camera):**
* **The camera code** (`source_libcamera.cpp`) compiles against libcamera, but has never run
  on a camera.
* **Raw formats:** whether the Pi 5 offers the OV9281 as 8-bit raw (`R8`), and at which frame
  rates. The camera code falls back to 10-bit formats if not.
* **Timestamps:** what `SensorTimestamp` measures exactly, and the offset to the end of exposure.
* **Pi setup:** the setup script on a real Pi (camera overlay name, PTP services, chrony on the
  grandmaster).

## Next

* **The test marker:** firmware for a Raspberry Pi Pico that blinks an IR LED with a drone ID
  code (docs/PROTOCOL.md section 6.2), so detection and ID decoding can be tested with one
  camera on the bench.
* **The station agent** (control from the PC: exposure, thresholds, snapshots over the network;
  ARCHITECTURE.md section 4.1).
* **Lens calibration:** a tool for each camera's focal length and distortion, needed before
  multi-camera work.
