# Test marker (marker/)

Firmware for an ESP32-C5 (bench part: Seeed XIAO ESP32-C5). It makes an IR LED blink a drone
ID code, the same way each drone's marker will, so the camera station and the engine can be tested
before there is a drone.

- **The code:** PROTOCOL.md section 6.2, Mode A. It is SYNC (H H H L L L), then the
  Manchester-coded 8-bit ID plus CRC-4. That is 30 chips at 30 chips/s, so one code takes 1 s. It
  runs freely, with no sync to the cameras; the engine finds the code's phase itself.
- **Two levels:** HIGH is 100 % and LOW is 40 % of the LED's brightness. The LED is never fully
  off, so the cameras never lose the dot. Both levels are PWM duty cycles at 200 kHz, so each
  0.5 ms exposure averages 100 PWM cycles and sees a steady brightness.
- **Preview:** the board's own LED (GPIO27 on the XIAO) blinks the same code, on for HIGH and off
  for LOW. You can check it by eye before the IR LED arrives. Slow it down with `rate 2`.
- **Check:** `tests/test_marker_firmware.py` compiles the firmware's code generator on the PC. It
  checks that all 256 IDs give exactly the chips the engine decodes.

## Build and flash (Mac)

You need ESP-IDF 5.5 or later; that's the first version that supports the C5. Install it once:

```sh
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.1 --recursive --shallow-submodules --depth 1 https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32c5
```

Then build and flash:

```sh
. ~/esp/esp-idf/export.sh           # once per terminal
cd ~/IPS/marker
idf.py set-target esp32c5           # once
idf.py build
idf.py -p /dev/cu.usbmodem* flash monitor
```

Plug the XIAO into the Mac over USB-C. `ls /dev/cu.usbmodem*` shows its port. If flashing doesn't
start, hold BOOT, tap RESET, then release BOOT. Leave the monitor with Ctrl-].

## Console

The console runs over the same USB port, in `idf.py monitor` or any serial terminal. Settings are
saved in flash and survive power-off.

| command    | does                                                        |
|------------|-------------------------------------------------------------|
| `id 7`     | blink ID 7 (0–255)                                          |
| `rate 2`   | 2 chips/s, slow enough to read the preview LED by eye       |
| `rate 30`  | back to the system rate                                     |
| `low 40`   | LOW level, % of HIGH                                        |
| `bright 50`| HIGH level, % of full; dims the LED if the dot saturates    |
| `status`   | settings and the code as H/L letters                        |

At `rate 2`, ID 1 shows as three blinks long on and three long off (SYNC), then the 24 data chips.

## Wiring the IR LED

5 mm 850 nm LED on the bench:

```
GPIO1 (pin D0) ──[100 Ω]──►|── GND
                      LED+   LED−  (long leg is +)
```

That gives about (3.3 V − 1.5 V) / 100 Ω ≈ 18 mA, inside both the pin's and the LED's limits.
The XIAO's D-pin to GPIO mapping is in Seeed's pinout. If your LED is on another pin, change it
with `idf.py menuconfig` → *IDPS marker* → *IR LED GPIO*, then rebuild.

For a 1–3 W LED, the pin can't drive it directly. The pin drives the gate of a logic-level
N-MOSFET (e.g. AO3400). The MOSFET switches the LED, with its current resistor or constant-current
driver, from the battery. Put a 10 kΩ pull-down from gate to GND. The firmware doesn't change.

## Testing with the camera station

1. Point a station camera at the LED and run `idps-station snapshot`. The dot should be there and
   not saturated (peak < 255). If it saturates, lower `bright` or shorten the exposure.
2. The engine reads IDs on its 3D tracks, so it needs two calibrated stations watching the LED.
   With them, `idps-engine live` should name the marker within a few seconds. With one camera, the
   snapshot is the check: the dot is there, and its brightness changes between frames taken a
   moment apart.
