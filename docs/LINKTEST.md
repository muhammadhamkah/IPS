# Radio link test (linktest/)

Two ESP32-C5 boards measure the radio link that will carry drone positions, before the drone is
designed around it.

- **Ground board:** stays plugged into the Mac. It sends a position-sized packet (64 bytes) 120
  times a second, as the PC will send each drone its position.
- **Drone board:** you carry it around on a USB power bank. It answers every packet with what it
  has received so far.
- **Every second, the ground board prints:**
  - how many positions the drone got and how many were lost;
  - the **longest gap**: the longest time the drone went without a new position. At 120/s, 8.3 ms
    is normal.
  - the round trip time;
  - the signal strength at both ends.

The link is Wi-Fi: the ground board is an access point on 5 GHz channel 36, and the drone board
joins it. Packets go one-to-one, so Wi-Fi resends lost ones, as the real link will. `channel 6`
runs the same test on 2.4 GHz for comparison.

Two LEDs show how it's going:
- The **drone board's LED** is on while positions arrive. It goes off when more than 100 ms pass
  without one, so you can see dropouts while walking around.
- The **ground board's LED** is on while the drone answers.

## Flash both boards

```sh
cd ~/IPS && git pull
. ~/esp/esp-idf/export.sh
cd linktest
idf.py set-target esp32c5            # once
idf.py build
idf.py -p /dev/cu.usbmodem1101 flash monitor
```

Flash one board, then the other. Each board's port name may differ, so check with
`ls /dev/cu.usbmodem*`. On the first start, give each board its role in the monitor:

```
role ground        (the board that stays at the Mac)
role drone         (the other one)
```

The role is saved, so after that the boards start in their role on their own.

## Run the test

1. Plug the drone board into a USB power bank or charger, so it runs on its own.
2. Keep the ground board in `idf.py monitor` on the Mac. You should see `drone connected`, then one
   line a second:
   ```
      12 s  sent 120  drone got 120  lost  0 | longest gap   9.1 ms | round trip  2.10 /  4.80 ms (median / max) | signal -38 dBm here, -40 dBm at the drone
   ```
3. Type `reset`, then walk the drone board around for a few minutes:
   - to the far end of the room;
   - behind people, and inside your closed hand;
   - next to the Wi-Fi router, and with phones streaming video nearby.
4. Type `stats` for the totals: positions lost, how many gaps were longer than 25, 50, 100 and
   250 ms, the longest gap, and the round trip percentiles.

### Other commands

| command | does |
|---|---|
| `channel 6` / `channel 36` / `channel 149` | 2.4 GHz vs 5 GHz, and another 5 GHz channel (ground board; restarts) |
| `rate 240` | twice the rate: room for more drones on one link |
| `size 300` | bigger packets (e.g. several drones in one packet) |
| `country US` | radio rules for a country (default `01`, world safe) |
| `watch off` | stop the line every second |

## What good looks like

The flight controller can carry on for a short gap on its own sensors, so what matters is the
**longest gap**, not the occasional lost packet.

| measure | good | worry |
|---|---|---|
| lost positions | < 0.1 % | > 1 % |
| gaps > 50 ms | 0 in 5 minutes | several a minute |
| longest gap | < 50 ms | > 100 ms |
| round trip, median | < 5 ms | > 20 ms |

Paste the `stats` output from a walk-around into the chat. If 5 GHz is clean, a C5 on each drone
is the plan. If not, the numbers show whether to change the channel or rate, or to rethink the
link.
