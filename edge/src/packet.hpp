/*
 * FRAME_OBS and HEALTH packets (docs/PROTOCOL.md), byte for byte as idps/protocol/packets.py
 * encodes them; tests compare the two.
 */
#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "../../protocol/idps_protocol.h"
#include "detect.hpp"

namespace idps {

struct FrameInfo {
    uint16_t station_id = 0;
    uint64_t frame_id = 0;
    int64_t t_mid_ns = 0;         // mid-exposure, system time base (PTP-disciplined)
    uint32_t exposure_ns = 0;
    uint32_t proc_us = 0;
    uint8_t sync_state = IDPS_SYNC_FREE_RUN;
    uint16_t threshold = 0;
    uint32_t drops_since_last = 0;
    uint16_t flags = 0;           // header flags (IDPS_FLAG_SIMULATED, IDPS_FLAG_UNCALIBRATED)
};

/* One frame's packets: dots split 89 to a packet (docs/PROTOCOL.md section 2); a frame with no
 * dots is still one packet. The caller's next sequence number is seq + result.size(). */
inline std::vector<std::string> encode_frame(const FrameInfo &f, const std::vector<Dot> &dots, uint32_t seq) {
    const size_t per = IDPS_MAX_MARKERS_PER_PACKET;
    const size_t total = dots.size();
    const size_t parts = total ? (total + per - 1) / per : 1;
    std::vector<std::string> out;
    out.reserve(parts);
    for (size_t k = 0; k < parts; k++) {
        const size_t a = k * per, n = total ? std::min(per, total - a) : 0;
        idps_header_t h{};
        idps_frame_obs_t fo{};
        uint16_t flags = f.flags;
        if (parts > 1) flags |= IDPS_FLAG_PART | (k == parts - 1 ? IDPS_FLAG_LAST_PART : 0);
        fo.frame_id = f.frame_id;
        fo.t_mid_ns = f.t_mid_ns;
        fo.exposure_ns = f.exposure_ns;
        fo.proc_us = (uint16_t)std::min<uint32_t>(f.proc_us, 0xFFFF);
        fo.sync_state = f.sync_state;
        fo.part_index = (uint8_t)k;
        fo.n_markers = (uint16_t)n;
        fo.n_markers_total = (uint16_t)std::min<size_t>(total, 0xFFFF);
        fo.threshold = f.threshold;
        fo.drops_since_last = (uint16_t)std::min<uint32_t>(f.drops_since_last, 0xFFFF);
        h.magic = IDPS_MAGIC;
        h.version = IDPS_VERSION;
        h.msg_type = IDPS_MSG_FRAME_OBS;
        h.station_id = f.station_id;
        h.seq = seq + (uint32_t)k;
        h.payload_len = (uint16_t)(sizeof fo + n * sizeof(idps_marker_t));
        h.flags = flags;
        std::string pkt(sizeof h + h.payload_len, '\0');
        char *p = &pkt[0];
        std::memcpy(p, &h, sizeof h);
        std::memcpy(p + sizeof h, &fo, sizeof fo);
        p += sizeof h + sizeof fo;
        for (size_t i = 0; i < n; i++) {
            const Dot &d = dots[a + i];
            idps_marker_t m{};
            m.x = (float)d.x;
            m.y = (float)d.y;
            const double s = d.sum < 0 ? 0.0 : (d.sum > 4294967295.0 ? 4294967295.0 : d.sum);
            m.sum = (uint32_t)(int64_t)s;            // as Python's int(sum) & 0xFFFFFFFF
            m.peak = (uint16_t)d.peak;
            m.area = (uint8_t)std::min(d.area, 255);
            m.mflags = (uint8_t)d.flags;
            std::memcpy(p, &m, sizeof m);
            p += sizeof m;
        }
        out.push_back(std::move(pkt));
    }
    return out;
}

/* HEALTH: the header and a JSON object (docs/PROTOCOL.md section 3). Empty if too large. */
inline std::string encode_health(uint16_t station_id, uint32_t seq, const std::string &json, uint16_t flags = 0) {
    if (json.size() > IDPS_MAX_UDP_PAYLOAD - sizeof(idps_header_t)) return {};
    idps_header_t h{};
    h.magic = IDPS_MAGIC;
    h.version = IDPS_VERSION;
    h.msg_type = IDPS_MSG_HEALTH;
    h.station_id = station_id;
    h.seq = seq;
    h.payload_len = (uint16_t)json.size();
    h.flags = flags;
    std::string pkt(sizeof h, '\0');
    std::memcpy(&pkt[0], &h, sizeof h);
    return pkt + json;
}

}  // namespace idps
