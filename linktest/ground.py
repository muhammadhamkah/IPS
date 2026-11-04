#!/usr/bin/env python3
"""Ground side of the radio link test, run on the Mac when there is only one ESP32-C5
(docs/LINKTEST.md, "One board"). The board runs `role solo`: it is the Wi-Fi access point and
answers like a drone. Join its Wi-Fi 'idps-linktest' (password 'idps-linktest'), then:

    python3 linktest/ground.py [--rate 120] [--size 64] [--minutes 0]

Every second: positions sent, how many the board got, the longest gap at the board, round trip,
signal. Ctrl-C (or --minutes) prints the totals. Standard library only.
"""
import argparse
import random
import select
import socket
import struct
import time

BOARD = ("192.168.4.1", 14600)
MAGIC = 0x4B4C4449
T_POS, T_ECHO = 1, 2
GAP_MS = (25, 50, 100, 250)
# magic, type, pad, size, seq, epoch, t_us, rx, late, gap_max_window_us, gap_max_us, gaps[4], rssi
PKT = struct.Struct("<IBBHIIqIIII4Ib3x")


def pct(sorted_vals, p):
    if not sorted_vals:
        return 0.0
    return sorted_vals[min(len(sorted_vals) - 1, int(len(sorted_vals) * p / 100))]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rate", type=int, default=120, help="positions per second (120)")
    ap.add_argument("--size", type=int, default=64, help=f"bytes per packet ({PKT.size}..1400; 64)")
    ap.add_argument("--minutes", type=float, default=0, help="stop after this long (0: until Ctrl-C)")
    ap.add_argument("--board", default=BOARD[0], help="the board's address (192.168.4.1)")
    a = ap.parse_args()
    size = max(PKT.size, min(1400, a.size))

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setblocking(False)
    board = (a.board, BOARD[1])
    epoch = random.getrandbits(32)          # a new epoch: the board restarts its counters
    t0 = time.monotonic()
    us = lambda: int((time.monotonic() - t0) * 1e6)

    seq = sent = errors = echoes = 0
    last = None                              # the newest answer's counters
    rtts, win = [], []
    sent_prev = rx_prev = 0
    last_echo = None
    period = 1.0 / a.rate
    next_send = next_report = time.monotonic()
    next_report += 1.0
    end = t0 + a.minutes * 60 if a.minutes else None
    pad = bytes(size - PKT.size)
    print(f"sending {a.rate} positions/s of {size} bytes to {board[0]}; Ctrl-C for the totals")
    try:
        while end is None or time.monotonic() < end:
            now = time.monotonic()
            if now >= next_send:
                seq += 1
                pkt = PKT.pack(MAGIC, T_POS, 0, size, seq, epoch, us(), 0, 0, 0, 0, 0, 0, 0, 0, 0) + pad
                try:
                    s.sendto(pkt, board)
                except OSError as e:
                    errors += 1
                    if errors == 1:
                        print(f"cannot send: {e}. Is the Mac on the Wi-Fi 'idps-linktest'?")
                sent += 1
                next_send += period
                if next_send < now - 0.1:    # fell far behind (Mac asleep?): don't burst to catch up
                    next_send = now + period
            while True:
                try:
                    data = s.recv(2048)
                except BlockingIOError:
                    break
                except OSError:
                    break
                if len(data) < PKT.size:
                    continue
                f = PKT.unpack_from(data)
                if f[0] != MAGIC or f[1] != T_ECHO or f[5] != epoch:
                    continue
                rtt = us() - f[6]
                echoes += 1
                rtts.append(rtt)
                win.append(rtt)
                if last is None or f[7] >= last[7]:
                    last = f
                last_echo = time.monotonic()
            if now >= next_report:
                next_report += 1.0
                secs = now - t0
                if last is None or now - last_echo > 1.0:
                    print(f"{secs:5.0f} s  NO ANSWER from the board"
                          + ("" if last is None else f" for {now - last_echo:.1f} s"))
                else:
                    got = last[7] - rx_prev
                    lost = max(0, (sent - sent_prev) - got)
                    win.sort()
                    print(f"{secs:5.0f} s  sent {sent - sent_prev:3d}  board got {got:3d}  lost {lost:2d} | "
                          f"longest gap {last[9] / 1000:5.1f} ms | round trip {pct(win, 50) / 1000:5.2f} / "
                          f"{(win[-1] if win else 0) / 1000:5.2f} ms (median / max) | signal {last[15]} dBm at the board")
                    rx_prev = last[7]
                sent_prev = sent
                win = []
            # wait for an answer or the next thing to do, whichever comes first
            select.select([s], [], [], max(0.0, min(next_send, next_report) - time.monotonic()))
    except KeyboardInterrupt:
        pass
    secs = time.monotonic() - t0
    print(f"\n=== {secs:.0f} s, {a.rate} positions/s of {size} bytes ===")
    if last is None:
        print("no answer from the board: is the Mac on the Wi-Fi 'idps-linktest', and the board in 'role solo'?")
        return
    rx = last[7]
    lost = max(0, sent - rx)
    print(f"positions: sent {sent}, board got {rx}, lost {lost} ({100 * lost / max(sent, 1):.3f} %)")
    if errors:
        print(f"  of which {errors} could not be sent at all")
    gaps = "   ".join(f">{g} ms: {n}" for g, n in zip(GAP_MS, last[11:15]))
    print(f"gaps at the board (time without a new position): {gaps}   longest {last[10] / 1000:.1f} ms")
    rtts.sort()
    print(f"round trip (Mac -> board -> Mac): median {pct(rtts, 50) / 1000:.2f} ms, 99% {pct(rtts, 99) / 1000:.2f} ms, "
          f"99.9% {pct(rtts, 99.9) / 1000:.2f} ms, max {rtts[-1] / 1000:.1f} ms")
    print(f"answers lost on the way back: {rx - echoes}; arrived out of order: {last[8]}")


if __name__ == "__main__":
    main()
