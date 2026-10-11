"""Regenerate the golden test vectors: python3 protocol/make_vectors.py"""
from pathlib import Path

from idps.protocol.packets import (FLAG_SIMULATED, MFLAG_ELONGATED, MFLAG_SATURATED, SYNC_PTP, FrameObs, Marker,
                                   encode_frame, encode_health)

OUT = Path(__file__).parent / "vectors"


def vectors():
    yield "frame_empty", encode_frame(FrameObs(7, 1000, 58322100000, 500000), 41)[0]
    yield "frame_two_markers", encode_frame(FrameObs(
        3, 10482, 58322100000, 400000,
        [Marker(621.42, 383.81, 1770, 177, 7, 0), Marker(12.5, 790.25, 65000, 1023, 31, MFLAG_SATURATED | MFLAG_ELONGATED)],
        proc_us=1400, sync_state=SYNC_PTP, threshold=60, flags=FLAG_SIMULATED), 5)[0]
    many = FrameObs(1, 2, 3, 4, [Marker(float(i), float(2 * i), i, i, i % 256, 0) for i in range(100)])
    for k, pkt in enumerate(encode_frame(many, 0)):
        yield f"frame_split_part{k}", pkt
    yield "health", encode_health(3, 9, {"fw": "sim", "fps_measured": 119.98, "ptp_offset_ns": -1830})


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    for name, data in vectors():
        (OUT / f"{name}.bin").write_bytes(data)
        print(name, len(data), "bytes")
