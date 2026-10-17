/*
 * IDPS wire protocol v1 (docs/PROTOCOL.md). Little-endian, packed.
 * Mirrors idps/protocol/packets.py; golden vectors in protocol/vectors/.
 * Used by the camera-station code (edge/) when it is written.
 */
#ifndef IDPS_PROTOCOL_H
#define IDPS_PROTOCOL_H

#include <stdint.h>

#define IDPS_MAGIC 0x53504449u /* "IDPS" */
#define IDPS_VERSION 1u

enum idps_msg_type { IDPS_MSG_FRAME_OBS = 1, IDPS_MSG_HEALTH = 2 };

enum idps_header_flags {
    IDPS_FLAG_PART = 1u << 0,
    IDPS_FLAG_LAST_PART = 1u << 1,
    IDPS_FLAG_SIMULATED = 1u << 2,
    IDPS_FLAG_UNCALIBRATED = 1u << 3,
};

enum idps_marker_flags {
    IDPS_MFLAG_SATURATED = 1u << 0,
    IDPS_MFLAG_EDGE = 1u << 1,
    IDPS_MFLAG_ELONGATED = 1u << 2,
    IDPS_MFLAG_ROI = 1u << 3,
};

enum idps_sync_state { IDPS_SYNC_FREE_RUN = 0, IDPS_SYNC_PTP = 1, IDPS_SYNC_HW_TRIGGER = 2, IDPS_SYNC_HOLDOVER = 3 };

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;       /* IDPS_MAGIC */
    uint8_t version;      /* IDPS_VERSION */
    uint8_t msg_type;     /* idps_msg_type */
    uint16_t station_id;
    uint32_t seq;         /* per-station packet counter; gaps = network loss */
    uint16_t payload_len; /* bytes after this header */
    uint16_t flags;       /* idps_header_flags */
} idps_header_t;          /* 16 bytes */

typedef struct {
    uint64_t frame_id;        /* sensor frame counter; gaps = frames dropped on the station */
    int64_t t_mid_ns;         /* mid-exposure time, system time base (PTP/TAI ns) */
    uint32_t exposure_ns;
    uint16_t proc_us;         /* detection time for this frame */
    uint8_t sync_state;       /* idps_sync_state */
    uint8_t part_index;       /* 0-based part of a split frame */
    uint16_t n_markers;       /* markers in this packet */
    uint16_t n_markers_total; /* markers in the whole frame */
    uint16_t threshold;       /* detection threshold, raw DN */
    uint16_t drops_since_last;
} idps_frame_obs_t;           /* 32 bytes */

typedef struct {
    float x, y;      /* subpixel centroid, distorted image coords, origin = centre of top-left pixel */
    uint32_t sum;    /* background-subtracted integrated intensity, raw DN */
    uint16_t peak;   /* peak pixel value, raw DN */
    uint8_t area;    /* pixels above the low threshold, saturating at 255 */
    uint8_t mflags;  /* idps_marker_flags */
} idps_marker_t;     /* 16 bytes */
#pragma pack(pop)

/* One FRAME_OBS packet per captured frame, even with zero markers. HEALTH
 * (1 Hz) carries a UTF-8 JSON object after the header. */
#define IDPS_MAX_UDP_PAYLOAD 1472
#define IDPS_MAX_MARKERS_PER_PACKET \
    ((IDPS_MAX_UDP_PAYLOAD - sizeof(idps_header_t) - sizeof(idps_frame_obs_t)) / sizeof(idps_marker_t)) /* 89 */

#ifdef __cplusplus
static_assert(sizeof(idps_header_t) == 16, "header size");
static_assert(sizeof(idps_frame_obs_t) == 32, "frame size");
static_assert(sizeof(idps_marker_t) == 16, "marker size");
#else
_Static_assert(sizeof(idps_header_t) == 16, "header size");
_Static_assert(sizeof(idps_frame_obs_t) == 32, "frame size");
_Static_assert(sizeof(idps_marker_t) == 16, "marker size");
#endif

#endif /* IDPS_PROTOCOL_H */
