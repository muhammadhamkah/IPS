/*
 * MIPI CSI-2 packed RAW10 (libcamera's R10_CSI2P): every 4 pixels take 5 bytes, the high 8 bits
 * of each pixel then one byte with the 4 pairs of low bits (pixel 0's in bits 0-1). The station
 * uses 8-bit raw (R8) when the sensor offers it; this is the fallback.
 */
#pragma once

#include <cstdint>

namespace idps {

inline void unpack_raw10_csi2(const uint8_t *src, int width, int height, int stride, uint16_t *dst) {
    for (int y = 0; y < height; y++) {
        const uint8_t *s = src + (size_t)y * stride;
        uint16_t *d = dst + (size_t)y * width;
        int x = 0;
        for (; x + 4 <= width; x += 4, s += 5) {
            const uint8_t lo = s[4];
            d[x] = (uint16_t)((s[0] << 2) | (lo & 3));
            d[x + 1] = (uint16_t)((s[1] << 2) | ((lo >> 2) & 3));
            d[x + 2] = (uint16_t)((s[2] << 2) | ((lo >> 4) & 3));
            d[x + 3] = (uint16_t)((s[3] << 2) | ((lo >> 6) & 3));
        }
        for (int k = 0; x < width; x++, k++) d[x] = (uint16_t)((s[k] << 2) | ((s[4] >> (2 * k)) & 3));
    }
}

}  // namespace idps
