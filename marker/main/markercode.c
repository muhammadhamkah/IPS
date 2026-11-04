#include "markercode.h"

uint8_t mc_crc4(uint8_t id) {
    uint8_t reg = 0;
    for (int i = 7; i >= 0; i--) {
        const uint8_t bit = (id >> i) & 1u, top = (reg >> 3) & 1u;
        reg = (uint8_t)((reg << 1) & 0xFu);
        if (bit ^ top) reg ^= 0x3u;
    }
    return reg;
}

int mc_chips(int id, uint8_t out[MC_CHIPS]) {
    static const uint8_t sync[MC_SYNC_CHIPS] = {1, 1, 1, 0, 0, 0};
    if (id < 0 || id > MC_ID_MAX) return -1;
    int k = 0;
    for (int i = 0; i < MC_SYNC_CHIPS; i++) out[k++] = sync[i];
    const unsigned word = ((unsigned)id << 4) | mc_crc4((uint8_t)id);   /* 12 bits, MSB first */
    for (int i = 11; i >= 0; i--) {
        const unsigned b = (word >> i) & 1u;
        out[k++] = b ? 1 : 0;      /* Manchester: 1 -> H L, 0 -> L H */
        out[k++] = b ? 0 : 1;
    }
    return 0;
}
