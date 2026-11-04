/* Reads protocol/vectors/frame_two_markers.bin with the C structs and checks fields. */
#include <stdio.h>
#include <string.h>
#include "../../protocol/idps_protocol.h"

int main(int argc, char **argv) {
    unsigned char buf[2048];
    FILE *f = fopen(argv[1], "rb");
    if (!f) return 2;
    size_t n = fread(buf, 1, sizeof buf, f);
    fclose(f);
    idps_header_t h; idps_frame_obs_t fr; idps_marker_t m[2];
    memcpy(&h, buf, sizeof h);
    memcpy(&fr, buf + sizeof h, sizeof fr);
    memcpy(m, buf + sizeof h + sizeof fr, sizeof m);
    int ok = h.magic == IDPS_MAGIC && h.version == IDPS_VERSION && h.msg_type == IDPS_MSG_FRAME_OBS &&
             h.station_id == 3 && h.seq == 5 && h.payload_len == n - sizeof h && (h.flags & IDPS_FLAG_SIMULATED) &&
             fr.frame_id == 10482 && fr.t_mid_ns == 58322100000LL && fr.exposure_ns == 400000 &&
             fr.proc_us == 1400 && fr.sync_state == IDPS_SYNC_PTP && fr.n_markers == 2 && fr.threshold == 60 &&
             m[0].x > 621.41f && m[0].x < 621.43f && m[0].sum == 1770 && m[0].peak == 177 && m[0].area == 7 &&
             m[1].peak == 1023 && m[1].mflags == (IDPS_MFLAG_SATURATED | IDPS_MFLAG_ELONGATED) &&
             IDPS_MAX_MARKERS_PER_PACKET == 89;
    printf(ok ? "ok\n" : "MISMATCH\n");
    return ok ? 0 : 1;
}
