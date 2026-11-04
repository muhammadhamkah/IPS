/*
 * Time on the station (docs/NETWORK_TIMING.md). Frames are stamped by the camera driver in
 * CLOCK_MONOTONIC; packets carry CLOCK_REALTIME, which phc2sys keeps on PTP time, so every
 * station shares one time base. The conversion is measured for each frame, so a clock step
 * or slew by phc2sys is followed at once.
 */
#pragma once

#include <time.h>

#include <cstdint>

namespace idps {

inline int64_t now_ns(clockid_t c) {
    timespec ts;
    clock_gettime(c, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

inline int64_t mono_ns() { return now_ns(CLOCK_MONOTONIC); }
inline int64_t real_ns() { return now_ns(CLOCK_REALTIME); }

/* CLOCK_REALTIME - `from` (ns): the closest of a few back-to-back readings, so a preemption
 * between two clock reads does not bias it. */
inline int64_t realtime_minus(clockid_t from) {
    int64_t best_gap = INT64_MAX, best = 0;
    for (int i = 0; i < 3; i++) {
        const int64_t a = now_ns(from), r = now_ns(CLOCK_REALTIME), b = now_ns(from);
        if (b - a < best_gap) {
            best_gap = b - a;
            best = r - (a + (b - a) / 2);
        }
    }
    return best;
}

}  // namespace idps
