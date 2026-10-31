/*
 * The station's own state, for HEALTH packets and the benchmark: CPU temperature, throttling,
 * per-core load, PTP lock (linuxptp's `pmc`) and whether the network port has hardware
 * timestamps (`ethtool -T`). Every reading degrades to "unknown" off a Pi.
 */
#pragma once

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace idps {

inline std::string run_command(const std::string &cmd) {
    std::string out;
    FILE *p = popen((cmd + " 2>/dev/null").c_str(), "r");
    if (!p) return out;
    char buf[512];
    while (std::fgets(buf, sizeof buf, p)) out += buf;
    pclose(p);
    return out;
}

inline double cpu_temp_c() {
    std::ifstream f("/sys/class/thermal/thermal_zone0/temp");
    long milli = 0;
    if (!(f >> milli)) return -1;
    return milli / 1000.0;
}

/* vcgencmd get_throttled: bit 0 under-voltage now, 1 frequency capped, 2 throttled, 3 soft temp
 * limit; bits 16-19 the same "has happened since boot". -1 if unknown. */
inline long throttled_flags() {
    const std::string s = run_command("vcgencmd get_throttled");
    const size_t i = s.find("0x");
    if (i == std::string::npos) return -1;
    return std::strtol(s.c_str() + i, nullptr, 16);
}

/* Per-core busy fraction since the previous call. */
class CpuLoad {
  public:
    std::vector<double> sample() {
        std::ifstream f("/proc/stat");
        std::string line;
        std::vector<std::pair<long long, long long>> now;   // (busy, total)
        while (std::getline(f, line)) {
            if (line.rfind("cpu", 0) != 0 || line.size() < 4 || line[3] == ' ') continue;   // cpuN only
            std::istringstream ss(line.substr(line.find(' ')));
            long long v, total = 0, idle = 0;
            for (int k = 0; ss >> v; k++) {
                total += v;
                if (k == 3 || k == 4) idle += v;                                           // idle, iowait
            }
            now.push_back({total - idle, total});
        }
        std::vector<double> out;
        for (size_t i = 0; i < now.size() && i < prev_.size(); i++) {
            const long long dt = now[i].second - prev_[i].second;
            out.push_back(dt > 0 ? double(now[i].first - prev_[i].first) / dt : 0.0);
        }
        prev_ = now;
        return out;
    }

  private:
    std::vector<std::pair<long long, long long>> prev_;
};

struct PtpStatus {
    bool known = false;          // ptp4l answered
    std::string state = "NONE";  // ptp4l port state: SLAVE, MASTER, LISTENING, ...
    long long offset_ns = 0;     // offset from the master (slave)
    bool locked() const { return known && (state == "MASTER" || (state == "SLAVE" && std::llabs(offset_ns) < 100000)); }
};

/* Ask ptp4l (through linuxptp's pmc) for its port state and offset. */
inline PtpStatus ptp_status() {
    PtpStatus st;
    const std::string s = run_command("pmc -u -b 0 'GET PORT_DATA_SET' 'GET TIME_STATUS_NP'");
    auto field = [&](const char *name) -> std::string {
        const size_t i = s.find(name);
        if (i == std::string::npos) return {};
        std::istringstream ss(s.substr(i + std::char_traits<char>::length(name)));
        std::string v;
        ss >> v;
        return v;
    };
    const std::string port = field("portState");
    if (port.empty()) return st;
    st.known = true;
    st.state = port;
    const std::string off = field("master_offset");
    if (!off.empty()) st.offset_ns = std::atoll(off.c_str());
    return st;
}

/* Does the network port timestamp PTP packets in hardware (docs/NETWORK_TIMING.md)? */
inline bool hardware_timestamps(const std::string &iface, std::string *detail = nullptr) {
    const std::string s = run_command("ethtool -T " + iface);
    if (detail) *detail = s;
    const size_t c = s.find("PTP Hardware Clock:");
    const bool clock = c != std::string::npos && s.compare(c + 19, 5, " none") != 0;
    return clock && s.find("hardware-transmit") != std::string::npos && s.find("hardware-receive") != std::string::npos;
}

}  // namespace idps
