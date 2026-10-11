# IDPS Communication Protocol

Status: **proposal v0.1**. The wire format is versioned (`version = 1`). Golden test vectors
will live in `protocol/vectors/`, so that the C++ and Python implementations are tested
against the same bytes.

The protocol has three channels:

| Channel | Transport | Direction | Port (default) |
|---|---|---|---|
| Data plane: observations and health | UDP unicast (multicast optional) | station → engine | 47600 |
| Control plane: commands, configuration, snapshots | ZeroMQ REQ/REP over TCP, msgpack payloads | engine/UI → station | 47601 (on station) |
| Engine API: state and commands | ZeroMQ PUB/SUB + REQ/REP, msgpack payloads | engine ↔ UI and consumers | 47700 / 47701 |
| Time | PTP (IEEE 1588, linuxptp) or a hardware trigger | sync master → stations | 319/320 |

The example JSON in the project brief is fine for illustration. On the wire, observations are
**binary**:

* About 4× smaller than JSON.
* No parsing cost on the Pi.
* A fixed layout that is easy to fuzz and version.

---

## 1. Data plane: common header (16 bytes, little-endian)

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | `u32` | `magic` | `0x53504449` (`"IDPS"` in ASCII, little-endian) |
| 4 | `u8` | `version` | `1` |
| 5 | `u8` | `msg_type` | `1` = FRAME_OBS, `2` = HEALTH, `3` = BEACON_OBS (reserved) |
| 6 | `u16` | `station_id` | Unique, configured per station |
| 8 | `u32` | `seq` | Per-station packet counter (wraps). Gaps mean network loss |
| 12 | `u16` | `payload_len` | Bytes after the header |
| 14 | `u16` | `flags` | bit0 = `PART` (frame split across packets), bit1 = `LAST_PART`, bit2 = `SIMULATED`, bit3 = `UNCALIBRATED` (station pose not valid) |

Integrity relies on the UDP checksum, which IPv4 hosts generate and verify by default.
Receivers drop packets whose `magic`, `version` or `payload_len` do not match.

## 2. FRAME_OBS (`msg_type = 1`)

**One packet is sent for every captured frame, even when it holds zero markers.** A missing
packet therefore always means loss, never "nothing seen". This matters for tracking (negative
information) and for frame-drop statistics.

Fixed part (32 bytes):

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | `u64` | `frame_id` | Sensor/driver frame counter. Gaps mean frames dropped *on the station* |
| 8 | `i64` | `t_mid_ns` | **Mid-exposure** time in the system time base (PTP/TAI ns). Never packet-send time |
| 16 | `u32` | `exposure_ns` | Actual exposure duration |
| 20 | `u16` | `proc_us` | Detection time for this frame |
| 22 | `u8` | `sync_state` | 0 = free-run unsynced, 1 = PTP locked, 2 = HW-trigger locked, 3 = holdover |
| 23 | `u8` | `part_index` | 0-based index of this part, for split frames |
| 24 | `u16` | `n_markers` | Markers in *this* packet |
| 26 | `u16` | `n_markers_total` | Markers detected in the whole frame |
| 28 | `u16` | `threshold` | Detection threshold in use (raw DN) |
| 30 | `u16` | `drops_since_last` | Frames dropped by the station since the previous packet |

Marker record (16 bytes), repeated `n_markers` times:

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | `f32` | `x` | Subpixel centroid, **distorted** image coordinates, origin at the centre of the top-left pixel |
| 4 | `f32` | `y` | " |
| 8 | `u32` | `sum` | Background-subtracted integrated intensity (raw DN). Used for ID decoding |
| 12 | `u16` | `peak` | Peak pixel value (raw DN) |
| 14 | `u8` | `area` | Pixel count above `T_low`, saturating at 255 |
| 15 | `u8` | `mflags` | bit0 = saturated, bit1 = touches image edge, bit2 = elongated (possible merged blobs), bit3 = inside an ROI window |

**Payload limit:**

* A standard 1500-byte MTU leaves 1472 bytes of UDP payload.
* That fits `(1472 − 16 − 32) / 16 = 89` markers per packet.
* Frames with more markers are split. Each part repeats the fixed fields and sets
  `PART`/`LAST_PART`/`part_index`.

**Bandwidth example:** 20 visible markers per frame gives 368 B per packet. At 120 Hz that is
44 KB/s ≈ 0.35 Mbit/s per station, plus IP/UDP overhead.

**`confidence` in the brief:** the brief's example puts a `confidence` value in each marker
record. The station does not send one. Confidence depends on cross-camera consistency, so the
engine computes it. The station sends the raw evidence instead (`peak`, `area`, `mflags`).

## 3. HEALTH (`msg_type = 2`, 1 Hz)

The payload is a UTF-8 JSON object (v1; chosen over msgpack so the C++ station needs no extra
library). It is not on the hot path, so it is easier to extend.

```json
{
  "fw": "edge 0.3.1+g1a2b3c4",  "uptime_s": 9321,
  "fps_measured": 119.98, "frames_captured": 1118640, "frames_dropped": 3,
  "proc_us_p50": 1400, "proc_us_p99": 2900,
  "cpu_temp_c": 61.2, "throttled": 0, "cpu_load": [0.21, 0.95, 0.88, 0.40],
  "ptp_state": "SLAVE", "ptp_offset_ns": -1830, "ptp_path_delay_ns": 41200,
  "exposure_ns": 400000, "gain": 1.0, "threshold": 60,
  "gimbal": {"state": "locked", "pan_deg": 12.40, "tilt_deg": -21.10},
  "calib_version": "2026-10-11T08:00Z-3fa9c1",
  "net": {"iface": "eth0", "tx_errors": 0}
}
```

Health packets also serve as **discovery**: the engine registers any station it hears from.
Unknown `station_id`s appear in the UI as "unconfigured".

## 4. Control plane (station `idps-agent`, ZeroMQ REQ/REP)

Requests are msgpack maps of the form `{"cmd": <name>, "id": <u32>, "args": {...}}`.
Replies have the form `{"id": <u32>, "ok": bool, "result" | "error": ...}`.

| Command | Args | Effect |
|---|---|---|
| `get_info` | — | Model, sensor modes, lens ID, firmware, capabilities |
| `set_capture` | `mode`, `fps`, `exposure_ns`, `gain` | Reconfigure capture (brief interruption, logged) |
| `set_detect` | `threshold`, `threshold_low`, `min_area`, `max_area`, `roi_list` | Detector parameters |
| `start` / `stop` | — | Start or stop observation streaming |
| `snapshot` | `count`, `format` (`raw8`/`raw10`), `with_overlay` | Returns image(s) over the REP socket in chunks. Used for calibration and diagnostics |
| `gimbal_move` | `pan_deg`, `tilt_deg`, `speed` | Move; the station flags itself `UNCALIBRATED` |
| `gimbal_lock` / `gimbal_status` | — | Engage brake or hold; read encoder and status |
| `set_station_id`, `set_engine_addr` | … | Provisioning |
| `reboot`, `update` | … | Maintenance (requires the `--allow-maintenance` agent flag) |

Security: none in v1. The system runs on an isolated network. Message authentication (a
shared-key HMAC on control messages) is planned before any deployment on a shared network.

## 5. Time synchronisation

**Requirement:** each observation's mid-exposure timestamp must be known in a common time
base to σ ≤ 100 µs (target), equivalent to ≤ 0.4 mm of motion at 4 m/s.

| Mode | Mechanism | Expected accuracy | Notes |
|---|---|---|---|
| **S0 Free-run + NTP** | chrony over Wi-Fi | ~0.5–5 ms | Development only. The simulator shows this alone costs ~1–2 cm at 4 m/s (RISKS R2) |
| **S1 Free-run + PTP** | `ptp4l` on Ethernet, software timestamps (Pi 4B NIC hardware timestamping to be verified; CM4 and Pi 5 support it) | ~10–100 µs | Baseline for the wired system. Cameras are not phase-aligned; the engine uses per-camera timestamps (async EKF or interpolation) |
| **S2 Hardware trigger** | Sync master emits a 120 Hz pulse to each sensor's FSIN/trigger input over a wired line | < 10 µs, phase-aligned | Simplest maths. Needs sensor/driver trigger support (verify for OV9281 on Pi) and cabling |
| **S3 Radio sync** | Sync master broadcasts a timing beacon (e.g. a 2.4 GHz proprietary radio); a microcontroller per station regenerates the trigger | ~1–20 µs | Wireless and phase-aligned; **can also clock the drone LEDs** (§6, mode B) |

All modes still timestamp every frame. A trigger aligns capture, but timestamps verify the
alignment and catch missed triggers.

**Validation:** an LED strobed at known times (GPIO from the sync master) is observed by all
cameras. The recovered intensity transitions give per-camera timing offset and jitter
(VALIDATION.md T5).

## 6. Marker identification code (optical)

### 6.1 Constraints

* The camera samples LED brightness at the frame rate F (120 Hz), integrating over the
  exposure window.
* Brightness depends on range (1/r²), LED emission angle, exposure and occlusion. Absolute
  levels are useless; only **relative** changes within a track carry information.
* The marker must stay detectable in its "low" state at maximum range. Otherwise tracking,
  and with it the decoder's sample timing, breaks.
* The modulation must not bias the centroid. That holds as long as neither state saturates
  the sensor.

### 6.2 Mode A: free-running LEDs (Phase 1–4 default, no radio needed)

| Parameter | Value (initial; tuned in Phase 1) |
|---|---|
| Levels | HIGH = 100 %, LOW = 40 % LED current (amplitude ratio ≈ 2.5:1) |
| Line code | Manchester (DC-balanced, so mean brightness is constant and the self-clocking transitions allow clock recovery) |
| Chip rate | F / 4 = 30 chips/s, i.e. ≥ 3 full frames per chip despite unsynchronised clocks |
| Frame | `SYNC (6 chips: HHH LLL, a Manchester-illegal run)` + `ID (8 bits)` + `CRC-4 (4 bits)` |
| Frame duration | 6 + 2×12 = 30 chips = **1.0 s** |
| ID space | 256 IDs; the code set is pruned to a minimum Hamming distance ≥ 3 including CRC, leaving ≥ 64 usable IDs |
| Decoding | Engine-side, on the 3D track. Fuse per-camera normalised intensity, then: high-pass relative to the running mean → matched filter for SYNC → bit slicing with a clock PLL → CRC check. The ID is asserted after 2 consecutive matching frames and re-verified continuously |

Acquisition takes ~1–2 s. It is not needed every frame, because identity is carried by the
3D track between decodes. An ID swap is detected on the next decoded frame that disagrees.

### 6.3 Mode B: camera-synchronised LEDs (Phase 10 target)

* The drone's LED driver receives the same radio sync as the cameras (S3).
* It changes level only *between* exposures, giving 1 chip per frame.
* Frame: 4-chip SYNC + 8-bit ID + CRC-4, Manchester → 28 chips ≈ **0.23 s** at 120 Hz.
* It also enables **time-division ID**: each drone is assigned a slot in which it blinks
  differently, which helps resolve swaps after close passes.

### 6.4 Alternatives considered

* **On/off keying:** the marker disappears during "0" bits and track continuity breaks.
  Rejected.
* **Wavelength coding** (850 vs 940 nm): requires colour discrimination that a mono sensor
  lacks. Rejected.
* **Multi-marker geometric constellations** (OptiTrack-style rigid bodies): needs no
  modulation, but distinct patterns become hard to separate for 50 drones at 50–100 m. Kept
  as a **complementary** cue once multiple markers per drone are fitted.

## 7. Engine API (for UI and later consumers)

**PUB topic `state` (120 Hz, msgpack):**

```json
{"t_ns": 58322100000, "epoch": 10482,
 "tracks": [{"track_id": 17, "marker_id": 5, "id_conf": 0.999,
             "p": [12.4031, -3.2210, 4.8812], "v": [1.20, 0.01, -0.30],
             "cov": [/* 6 unique entries of the 3x3 */], "n_cams": 7,
             "status": "confirmed"}],
 "drones": [/* later: rigid bodies */]}
```

Other PUB topics: `health` (1 Hz), `events`, `calib`.

**REQ/REP:** `list_stations`, `station_cmd` (relayed to agents), `record_start/stop`,
`calib_run`, `calib_activate <version>`, `get_coverage`, `plan_load <plan.yaml>`,
`assign_station <station_id> <plan_camera_id>`, `aim_to_plan [coarse|fine]`,
`beacon_survey`, `as_built_report`, `continuous_calib on|off`, `calib_flight_pattern`,
`station_restore <station_id>`.

Consumers such as flight control and Blender read `state`. They never talk to stations
directly.
