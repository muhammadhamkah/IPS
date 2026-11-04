/*
 * The marker's optical ID code, Mode A (docs/PROTOCOL.md section 6.2), as idps/protocol/markercode.py
 * defines it: SYNC (H H H L L L) then Manchester(ID 8 bits + CRC-4), 30 chips = 1 s at 30 chips/s.
 * Plain C with no ESP-IDF dependencies, so the PC tests check it against the Python version.
 */
#pragma once

#include <stdint.h>

#define MC_CHIPS 30          /* chips per frame */
#define MC_SYNC_CHIPS 6
#define MC_ID_MAX 255

/* CRC-4 (x^4 + x + 1), MSB first, zero initial value, of an 8-bit ID */
uint8_t mc_crc4(uint8_t id);

/* One frame of chips for `id` (1 = HIGH, 0 = LOW) into out[MC_CHIPS]. Returns 0, or -1 if the
 * ID is out of range. */
int mc_chips(int id, uint8_t out[MC_CHIPS]);
