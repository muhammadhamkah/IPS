/*
 * idps-station: the camera station program (docs/STATION.md).
 *
 *   idps-station run --engine 192.168.1.10 --id 3     capture, detect, timestamp, send (the job)
 *   idps-station bench [--seconds 60]                 the first-station go/no-go numbers
 *   idps-station snapshot --out frame.pgm             one raw image (aiming, checking the filter)
 *   idps-station focus                                dot size and brightness, live (focus, exposure)
 *
 * One thread per stage: the camera's (capture), detection (dots, timestamp, packet, send), and
 * the main thread (HEALTH once a second). Frames wait in a 4-deep queue that drops its oldest
 * when detection falls behind (counted, and reported in every packet).
 */
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "clock.hpp"
#include "detect.hpp"
#include "frame.hpp"
#include "packet.hpp"
#include "source_libcamera.hpp"
#include "source_test.hpp"
#include "sysinfo.hpp"

using namespace idps;

static const char *VERSION_STR = "idps-station 0.1";
static std::atomic<bool> g_stop{false};
static void on_signal(int) { g_stop = true; }

// ------------------------------------------------------------------ options
struct Options {
    std::string cmd;
    std::string source = "camera";      // camera | synthetic | file:PATH
    SourceConfig src;
    DetectParams det;
    bool det_set_thr = false, det_set_lo = false;
    std::string engine;                 // host[:port]
    int station_id = 0;
    double ts_offset_us = 0;            // SensorTimestamp -> end of exposure (measured per sensor mode, Phase 2)
    std::string sync = "auto";          // auto (ask ptp4l) | ptp | free
    std::string iface = "eth0";
    double seconds = 60;
    std::string out = "frame.pgm";
    int skip = 30;
    std::vector<int> bench_dots = {0, 1, 10, 50, 200};
    bool realtime = false;
    int cpu = -1;
    bool simulated = false;
};

static void usage() {
    std::printf(
        "%s: the IDPS camera station (docs/STATION.md)\n\n"
        "  idps-station run --engine HOST[:PORT] --id N [options]   capture, detect and send to the engine\n"
        "  idps-station bench [--seconds S] [options]               go/no-go numbers for this Pi and camera\n"
        "  idps-station snapshot [--out frame.pgm] [options]        save one raw image\n"
        "  idps-station focus [options]                             live dot size and brightness\n"
        "  idps-station version\n\n"
        "Source:    --source camera|synthetic|file:PATH (camera)   --camera N (0)\n"
        "           --width 1280 --height 800 --fps 120 --exposure-us 500 --gain 1.0 --bits 8|10 (8)\n"
        "           --dots N (synthetic: 20)\n"
        "Detection: --threshold DN --threshold-low DN (8-bit: 60/30; 10-bit: 240/120) --seed-step 2\n"
        "           --min-area 1 --max-area 400\n"
        "Run:       --engine HOST[:PORT] (port 5600)  --id N  --ts-offset-us X (0)  --sync auto|ptp|free\n"
        "           --iface eth0  --realtime (SCHED_FIFO)  --cpu N (pin detection)  --simulated\n"
        "Bench:     --seconds S (60; 1800 for the 30-minute soak)  --bench-dots 0,1,10,50,200\n",
        VERSION_STR);
}

static bool parse(int argc, char **argv, Options &o) {
    if (argc < 2) return false;
    o.cmd = argv[1];
    for (int i = 2; i < argc; i++) {
        const std::string a = argv[i];
        auto val = [&]() -> std::string {
            if (i + 1 >= argc) { std::fprintf(stderr, "%s needs a value\n", a.c_str()); std::exit(2); }
            return argv[++i];
        };
        if (a == "--source") o.source = val();
        else if (a == "--camera") o.src.camera_index = std::atoi(val().c_str());
        else if (a == "--width") o.src.width = std::atoi(val().c_str());
        else if (a == "--height") o.src.height = std::atoi(val().c_str());
        else if (a == "--fps") o.src.fps = std::atof(val().c_str());
        else if (a == "--exposure-us") o.src.exposure_us = std::atoi(val().c_str());
        else if (a == "--gain") o.src.gain = std::atof(val().c_str());
        else if (a == "--bits") o.src.bits = std::atoi(val().c_str());
        else if (a == "--dots") o.src.synthetic_dots = std::atoi(val().c_str());
        else if (a == "--threshold") { o.det.threshold = std::atoi(val().c_str()); o.det_set_thr = true; }
        else if (a == "--threshold-low") { o.det.threshold_low = std::atoi(val().c_str()); o.det_set_lo = true; }
        else if (a == "--seed-step") o.det.seed_step = std::atoi(val().c_str());
        else if (a == "--min-area") o.det.min_area = std::atoi(val().c_str());
        else if (a == "--max-area") o.det.max_area = std::atoi(val().c_str());
        else if (a == "--engine") o.engine = val();
        else if (a == "--id") o.station_id = std::atoi(val().c_str());
        else if (a == "--ts-offset-us") o.ts_offset_us = std::atof(val().c_str());
        else if (a == "--sync") o.sync = val();
        else if (a == "--iface") o.iface = val();
        else if (a == "--seconds") o.seconds = std::atof(val().c_str());
        else if (a == "--out") o.out = val();
        else if (a == "--skip") o.skip = std::atoi(val().c_str());
        else if (a == "--realtime") o.realtime = true;
        else if (a == "--cpu") o.cpu = std::atoi(val().c_str());
        else if (a == "--simulated") o.simulated = true;
        else if (a == "--bench-dots") {
            o.bench_dots.clear();
            std::string s = val();
            for (size_t p = 0; p <= s.size();) {
                size_t q = s.find(',', p);
                if (q == std::string::npos) q = s.size();
                if (q > p) o.bench_dots.push_back(std::atoi(s.substr(p, q - p).c_str()));
                p = q + 1;
            }
        } else if (a == "-h" || a == "--help") { usage(); std::exit(0); }
        else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return false; }
    }
    // thresholds and clipping level follow the pixel depth unless given
    const bool ten = o.src.bits > 8;
    if (!o.det_set_thr) o.det.threshold = ten ? 240 : 60;
    if (!o.det_set_lo) o.det.threshold_low = ten ? 120 : 30;
    o.det.saturation = ten ? 1023 : 255;
    if (o.source != "camera") o.simulated = true;
    return true;
}

static std::unique_ptr<Source> make_source(const Options &o, std::string &err) {
    SourceConfig c = o.src;
    if (o.source == "camera") return make_libcamera_source(c);
    if (o.source == "synthetic") return make_synthetic_source(c);
    if (o.source.rfind("file:", 0) == 0) {
        c.file = o.source.substr(5);
        return make_file_source(c, err);
    }
    err = "unknown source " + o.source + " (camera, synthetic or file:PATH)";
    return nullptr;
}

// ------------------------------------------------------------------ statistics
/* The last n samples, for percentiles. */
class Window {
  public:
    explicit Window(size_t n = 1200) : v_(n), n_(n) {}
    void add(double x) {
        std::lock_guard<std::mutex> lk(m_);
        v_[i_++ % n_] = x;
        if (count_ < n_) count_++;
    }
    double pct(double p) const {
        std::lock_guard<std::mutex> lk(m_);
        if (!count_) return 0;
        std::vector<double> s(v_.begin(), v_.begin() + count_);
        std::sort(s.begin(), s.end());
        return s[std::min(count_ - 1, (size_t)std::floor(p / 100.0 * (count_ - 1) + 0.5))];
    }
    void clear() { std::lock_guard<std::mutex> lk(m_); count_ = 0; i_ = 0; }

  private:
    std::vector<double> v_;
    size_t n_, i_ = 0, count_ = 0;
    mutable std::mutex m_;
};

// ------------------------------------------------------------------ the pipeline
class Station {
  public:
    Station(const Options &o) : o_(o), det_(o.det), ring_(4) {}

    bool open_engine() {
        if (o_.engine.empty()) return true;
        std::string host = o_.engine, port = "5600";
        const size_t c = host.rfind(':');
        if (c != std::string::npos) { port = host.substr(c + 1); host = host.substr(0, c); }
        addrinfo hints{}, *res = nullptr;
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        if (getaddrinfo(host.c_str(), port.c_str(), &hints, &res) != 0 || !res) {
            std::fprintf(stderr, "cannot resolve engine address %s\n", o_.engine.c_str());
            return false;
        }
        sock_ = socket(res->ai_family, res->ai_socktype, 0);
        const bool ok = sock_ >= 0 && connect(sock_, res->ai_addr, res->ai_addrlen) == 0;
        freeaddrinfo(res);
        if (!ok) { std::fprintf(stderr, "cannot open a UDP socket to %s\n", o_.engine.c_str()); return false; }
        return true;
    }

    bool start(Source &src) {
        th_ = std::thread([this] { detect_loop(); });
        if (!src.start([this](Frame &&f) { ring_.push(std::move(f)); })) {
            ring_.close();
            th_.join();
            return false;
        }
        t_start_mono_ = mono_ns();
        return true;
    }

    void stop(Source &src) {
        src.stop();
        ring_.close();
        if (th_.joinable()) th_.join();
        ring_.drain();
    }

    // one HEALTH packet (docs/PROTOCOL.md section 3); also refreshes the sync state
    void send_health(double fps, double temp, long thr, const std::vector<double> &load, const PtpStatus &ptp) {
        update_sync(ptp);
        char b[1024];
        std::string loads;
        for (size_t i = 0; i < load.size(); i++) {
            char x[16];
            std::snprintf(x, sizeof x, "%s%.2f", i ? "," : "", load[i]);
            loads += x;
        }
        std::snprintf(b, sizeof b,
                      "{\"fw\":\"%s\",\"uptime_s\":%.1f,\"fps_measured\":%.2f,\"frames_captured\":%llu,"
                      "\"frames_dropped\":%llu,\"proc_us_p50\":%.0f,\"proc_us_p99\":%.0f,\"latency_us_p99\":%.0f,"
                      "\"cpu_temp_c\":%.1f,\"throttled\":%ld,\"cpu_load\":[%s],\"ptp_state\":\"%s\","
                      "\"ptp_offset_ns\":%lld,\"exposure_ns\":%d,\"gain\":%.2f,\"threshold\":%d,\"sync_state\":%d,"
                      "\"send_errors\":%llu}",
                      VERSION_STR, (mono_ns() - t_start_mono_) * 1e-9, fps, (unsigned long long)frames_.load(),
                      (unsigned long long)dropped_total(), proc_.pct(50), proc_.pct(99), latency_.pct(99), temp, thr,
                      loads.c_str(), ptp.state.c_str(), ptp.offset_ns, o_.src.exposure_us * 1000, o_.src.gain,
                      o_.det.threshold, (int)sync_state_.load(), (unsigned long long)send_errors_.load());
        send(encode_health((uint16_t)o_.station_id, seq_++, b, flags()));
    }

    uint64_t frames() const { return frames_; }
    uint64_t sensor_drops() const { return sensor_drops_; }
    uint64_t ring_drops() const { return ring_.drops(); }
    uint64_t dropped_total() const { return sensor_drops_ + ring_.drops(); }
    Window &proc() { return proc_; }
    Window &latency() { return latency_; }
    Window &sensor_age() { return sensor_age_; }
    Window &dots() { return dots_; }
    Window &areas() { return areas_; }
    Window &peaks() { return peaks_; }
    uint64_t saturated() const { return saturated_; }
    int64_t started() const { return t_start_mono_; }

  private:
    void update_sync(const PtpStatus &ptp) {
        if (o_.sync == "ptp") sync_state_ = IDPS_SYNC_PTP;
        else if (o_.sync == "free") sync_state_ = IDPS_SYNC_FREE_RUN;
        else if (ptp.locked()) { sync_state_ = IDPS_SYNC_PTP; was_locked_ = true; }
        else sync_state_ = was_locked_ ? IDPS_SYNC_HOLDOVER : IDPS_SYNC_FREE_RUN;
    }

    uint16_t flags() const { return o_.simulated ? IDPS_FLAG_SIMULATED : 0; }

    // never blocks: a packet the network cannot take right now is dropped and counted (HEALTH
    // send_errors); waiting would delay every later frame
    void send(const std::string &pkt) {
        if (sock_ >= 0 && !pkt.empty() && ::send(sock_, pkt.data(), pkt.size(), MSG_DONTWAIT) < 0) send_errors_++;
    }

    void detect_loop() {
        if (o_.realtime) {
            sched_param sp{};
            sp.sched_priority = 50;
            if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
                std::fprintf(stderr, "note: --realtime needs root or CAP_SYS_NICE; running at normal priority\n");
        }
        if (o_.cpu >= 0) {
            cpu_set_t set;
            CPU_ZERO(&set);
            CPU_SET(o_.cpu, &set);
            pthread_setaffinity_np(pthread_self(), sizeof set, &set);
        }
        Frame f;
        uint64_t reported_ring_drops = 0;
        while (ring_.pop(f)) {
            const int64_t t0 = mono_ns();
            const std::vector<Dot> &dots = f.bytes_per_pixel == 1
                ? det_.detect(f.data, f.width, f.height, f.stride)
                : det_.detect(reinterpret_cast<const uint16_t *>(f.data), f.width, f.height, f.stride);
            const int64_t t1 = mono_ns();
            // mid-exposure in the shared time base: the driver's timestamp (taken as the end of
            // exposure plus a fixed offset per sensor mode, measured in Phase 2) minus half the
            // exposure, moved from CLOCK_MONOTONIC to CLOCK_REALTIME (PTP time, via phc2sys)
            const int64_t t_mid = f.sensor_ns + (int64_t)std::llround(o_.ts_offset_us * 1000) - f.exposure_ns / 2 +
                                  realtime_minus(CLOCK_MONOTONIC);
            uint64_t gap = 0;
            if (have_seq_ && f.sequence > last_seq_ + 1) gap = f.sequence - last_seq_ - 1;
            have_seq_ = true;
            last_seq_ = f.sequence;
            sensor_drops_ += gap;
            const uint64_t rd = ring_.drops();
            FrameInfo fi;
            fi.station_id = (uint16_t)o_.station_id;
            fi.frame_id = f.sequence;
            fi.t_mid_ns = t_mid;
            fi.exposure_ns = (uint32_t)f.exposure_ns;
            fi.proc_us = (uint32_t)((t1 - t0) / 1000);
            fi.sync_state = sync_state_;
            fi.threshold = (uint16_t)o_.det.threshold;
            fi.drops_since_last = (uint32_t)(gap + rd - reported_ring_drops);
            fi.flags = flags();
            reported_ring_drops = rd;
            // statistics, then hand the buffer back before sending
            int n_sat = 0;
            for (const Dot &d : dots) {
                areas_.add(d.area);
                peaks_.add(d.peak);
                if (d.flags & DOT_SATURATED) n_sat++;
            }
            saturated_ += n_sat;
            dots_.add((double)dots.size());
            const int64_t sensor_ns = f.sensor_ns, arrival = f.arrival_mono_ns;
            std::vector<std::string> pkts = encode_frame(fi, dots, seq_);
            seq_ += (uint32_t)pkts.size();
            f.release();
            f = Frame{};
            for (const auto &p : pkts) send(p);
            const int64_t t2 = mono_ns();
            proc_.add((t1 - t0) / 1000.0);
            latency_.add((t2 - sensor_ns) / 1000.0);
            sensor_age_.add((arrival - sensor_ns) / 1000.0);
            frames_++;
        }
    }

    const Options &o_;
    Detector det_;
    FrameRing ring_;
    std::thread th_;
    int sock_ = -1;
    std::atomic<uint32_t> seq_{0};
    std::atomic<uint64_t> frames_{0}, sensor_drops_{0}, saturated_{0}, send_errors_{0};
    std::atomic<uint8_t> sync_state_{IDPS_SYNC_FREE_RUN};
    bool was_locked_ = false;
    bool have_seq_ = false;
    uint64_t last_seq_ = 0;
    int64_t t_start_mono_ = 0;
    Window proc_, latency_, sensor_age_, dots_, areas_{4000}, peaks_{4000};
};

static std::string temp_str(double c) {
    if (c < 0) return "unknown";
    char b[32];
    std::snprintf(b, sizeof b, "%.1f C", c);
    return b;
}

static void sleep_s(double s) { std::this_thread::sleep_for(std::chrono::microseconds((int64_t)(s * 1e6))); }

// ------------------------------------------------------------------ commands
static int cmd_run(const Options &o) {
    if (o.engine.empty()) { std::fprintf(stderr, "run needs --engine HOST[:PORT]\n"); return 2; }
    std::string err;
    auto src = make_source(o, err);
    if (!src) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }
    Station st(o);
    if (!st.open_engine()) return 1;
    if (!st.start(*src)) { std::fprintf(stderr, "%s\n", src->error().c_str()); return 1; }
    std::printf("%s: station %d -> %s; %s\n", VERSION_STR, o.station_id, o.engine.c_str(), src->describe().c_str());
    std::fflush(stdout);
    CpuLoad load;
    load.sample();
    uint64_t last_frames = 0;
    int64_t last = mono_ns();
    PtpStatus ptp = o.sync == "auto" ? ptp_status() : PtpStatus{};
    for (int tick = 1; !g_stop; tick++) {
        sleep_s(1.0);
        const int64_t now = mono_ns();
        const double fps = (st.frames() - last_frames) / ((now - last) * 1e-9);
        last_frames = st.frames();
        last = now;
        if (o.sync == "auto" && tick % 5 == 0) ptp = ptp_status();
        const double temp = cpu_temp_c();
        const long thr = throttled_flags();
        st.send_health(fps, temp, thr, load.sample(), ptp);
        if (tick % 10 == 0) {
            std::printf("%.0f fps, dots/frame %.0f, detect p99 %.0f us, exposure->sent p99 %.0f us, dropped %llu, "
                        "%.1f C, ptp %s %lld ns\n", fps, st.dots().pct(50), st.proc().pct(99), st.latency().pct(99),
                        (unsigned long long)st.dropped_total(), temp, ptp.state.c_str(), ptp.offset_ns);
            std::fflush(stdout);
        }
    }
    st.stop(*src);
    return 0;
}

static int cmd_snapshot(const Options &o) {
    std::string err;
    auto src = make_source(o, err);
    if (!src) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }
    std::mutex m;
    std::vector<uint8_t> px;
    int W = 0, H = 0, bpp = 1;
    std::atomic<int> seen{0};
    std::atomic<bool> got{false};
    if (!src->start([&](Frame &&f) {
            if (seen++ == o.skip && !got) {        // after the exposure has settled
                std::lock_guard<std::mutex> lk(m);
                W = f.width; H = f.height; bpp = f.bytes_per_pixel;
                px.resize((size_t)W * H * bpp);
                for (int y = 0; y < H; y++)
                    std::memcpy(&px[(size_t)y * W * bpp], f.data + (size_t)y * f.stride * bpp, (size_t)W * bpp);
                got = true;
            }
            f.release();
        })) {
        std::fprintf(stderr, "%s\n", src->error().c_str());
        return 1;
    }
    for (int i = 0; i < 100 && !got; i++) sleep_s(0.05);
    src->stop();
    if (!got) { std::fprintf(stderr, "no frame arrived\n"); return 1; }
    FILE *f = std::fopen(o.out.c_str(), "wb");
    if (!f) { std::fprintf(stderr, "cannot write %s\n", o.out.c_str()); return 1; }
    std::fprintf(f, "P5\n%d %d\n%d\n", W, H, bpp == 1 ? 255 : 1023);
    if (bpp == 1) std::fwrite(px.data(), 1, px.size(), f);
    else
        for (size_t i = 0; i < px.size(); i += 2) {      // PGM stores 16-bit pixels big-endian
            const uint8_t be[2] = {px[i + 1], px[i]};
            std::fwrite(be, 1, 2, f);
        }
    std::fclose(f);
    Detector det(o.det);
    const auto &dots = bpp == 1 ? det.detect(px.data(), W, H, W)
                                : det.detect(reinterpret_cast<const uint16_t *>(px.data()), W, H, W);
    std::printf("wrote %s (%dx%d, %d-bit), %zu dots:\n", o.out.c_str(), W, H, bpp == 1 ? 8 : 10, dots.size());
    for (size_t i = 0; i < dots.size() && i < 20; i++)
        std::printf("  x %8.2f  y %8.2f  peak %5d  area %3d%s\n", dots[i].x, dots[i].y, dots[i].peak, dots[i].area,
                    dots[i].flags & DOT_SATURATED ? "  saturated" : "");
    return 0;
}

static int cmd_focus(const Options &o) {
    std::string err;
    auto src = make_source(o, err);
    if (!src) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }
    Station st(o);
    if (!st.start(*src)) { std::fprintf(stderr, "%s\n", src->error().c_str()); return 1; }
    std::printf("%s\nAim for dots 3-5 px across (turn the lens) and peaks below %d (exposure, gain). Ctrl+C stops.\n",
                src->describe().c_str(), o.det.saturation);
    uint64_t last_sat = 0;
    while (!g_stop) {
        sleep_s(1.0);
        const double area = st.areas().pct(50);
        const uint64_t sat = st.saturated() - last_sat;
        last_sat = st.saturated();
        std::printf("dots %3.0f   size ~%.1f px (area %4.0f)   peak %5.0f (max %5.0f)   saturated %llu/s\n",
                    st.dots().pct(50), 2 * std::sqrt(area / M_PI), area, st.peaks().pct(50), st.peaks().pct(100),
                    (unsigned long long)sat);
        std::fflush(stdout);
        st.areas().clear();
        st.peaks().clear();
        st.dots().clear();
    }
    st.stop(*src);
    return 0;
}

static int cmd_bench(Options o) {
    bool go = true;
    auto verdict = [&](bool ok) { go &= ok; return ok ? "PASS" : "FAIL"; };
    std::printf("%s benchmark (docs/HARDWARE.md, Phase 2 go/no-go)\n\n", VERSION_STR);

    // 1. detection time against the number of dots, on frames in memory
    std::printf("1. Dot detection, %dx%d %d-bit frames in memory (target: p99 <= 4000 us up to 50 dots)\n",
                o.src.width, o.src.height, o.src.bits);
    std::printf("   %6s %10s %10s %10s\n", "dots", "p50 us", "p99 us", "max us");
    for (int n : o.bench_dots) {
        auto frames = synthetic_frames(o.src.width, o.src.height, o.src.bits, n, 30);
        Detector det(o.det);
        std::vector<double> t;
        const int64_t until = mono_ns() + 3000000000LL;
        for (size_t k = 0; mono_ns() < until || t.size() < 300; k++) {
            const auto &fr = frames[k % frames.size()];
            const int64_t a = mono_ns();
            if (o.src.bits > 8) det.detect(reinterpret_cast<const uint16_t *>(fr.data()), o.src.width, o.src.height, o.src.width);
            else det.detect(fr.data(), o.src.width, o.src.height, o.src.width);
            t.push_back((mono_ns() - a) / 1000.0);
            if (g_stop) return 1;
        }
        std::sort(t.begin(), t.end());
        const double p99 = t[(size_t)(0.99 * (t.size() - 1))];
        std::printf("   %6d %10.0f %10.0f %10.0f   %s\n", n, t[t.size() / 2], p99, t.back(),
                    n <= 50 ? verdict(p99 <= 4000) : "(for information)");
    }

    // 2. the whole pipeline from the camera for a while
    std::string err;
    if (o.source == "camera" && !libcamera_available()) {
        std::printf("\n   (built without libcamera: part 2 runs on synthetic frames instead of the camera)\n");
        o.source = "synthetic";
    }
    auto src = make_source(o, err);
    if (!src) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }
    Station st(o);
    if (!st.start(*src)) { std::fprintf(stderr, "%s\n", src->error().c_str()); return 1; }
    std::printf("\n2. Capture + detection for %.0f s: %s\n", o.seconds, src->describe().c_str());
    double tmax = -1, tstart = cpu_temp_c();
    long thr_or = 0;
    bool thr_known = false;
    const int64_t t0 = mono_ns();
    sleep_s(1.0);                                  // settle
    const uint64_t f0 = st.frames(), d0 = st.dropped_total();
    const int64_t m0 = mono_ns();
    int ticks = 1;
    while (!g_stop && (mono_ns() - t0) * 1e-9 < o.seconds) {
        sleep_s(1.0);
        tmax = std::max(tmax, cpu_temp_c());
        const long th = throttled_flags();
        if (th >= 0) { thr_or |= th; thr_known = true; }
        if (++ticks % 10 == 0) {
            std::printf("   %4d s: %llu frames, %s\n", ticks, (unsigned long long)st.frames(), temp_str(cpu_temp_c()).c_str());
            std::fflush(stdout);
        }
    }
    const double secs = (mono_ns() - m0) * 1e-9;
    const uint64_t frames = st.frames() - f0, drops = st.dropped_total() - d0;
    st.stop(*src);
    const double fps = frames / secs;
    const double drop_pct = frames + drops ? 100.0 * drops / double(frames + drops) : 0;
    std::printf("   frame rate            %8.2f fps (target %.0f)          %s\n", fps, o.src.fps, verdict(fps >= 0.99 * o.src.fps));
    std::printf("   frames dropped        %8llu (%.4f %%, target < 0.01 %%)   %s\n", (unsigned long long)drops, drop_pct,
                verdict(drop_pct < 0.01));
    std::printf("   detection p50 / p99   %6.0f / %.0f us\n", st.proc().pct(50), st.proc().pct(99));
    std::printf("   timestamp -> sent p99 %8.0f us (target <= 6000)       %s\n", st.latency().pct(99),
                verdict(st.latency().pct(99) <= 6000));
    const double age = st.sensor_age().pct(50);
    if (o.source == "camera") {
        const bool sane = age > 0 && age < 100000;
        std::printf("   frames arrive %.0f us after their timestamp: %s\n", age,
                    sane ? "the driver stamps in CLOCK_MONOTONIC, as assumed"
                         : "the timestamp is NOT CLOCK_MONOTONIC: tell the developers (timestamps would be wrong)");
        go &= sane;
    }
    std::printf("   CPU temperature       %s at start, %s max\n", temp_str(tstart).c_str(), temp_str(tmax).c_str());
    if (thr_known)
        std::printf("   throttling            0x%lx %s   %s\n", thr_or, thr_or ? "(power or heat: check the supply and cooler)" : "(none)",
                    verdict((thr_or & 0x7) == 0));
    else
        std::printf("   throttling            unknown (no vcgencmd: not a Raspberry Pi?)\n");

    // 3. clock synchronisation
    std::string detail;
    const bool hw = hardware_timestamps(o.iface, &detail);
    std::printf("\n3. PTP clock sync on %s\n   hardware timestamps   %s   %s\n", o.iface.c_str(), hw ? "yes" : "no",
                hw ? "PASS" : "(software PTP: ~12 us instead of ~1 us, still fine; docs/NETWORK_TIMING.md)");
    const PtpStatus ptp = ptp_status();
    std::printf("   ptp4l                 %s\n", ptp.known ? (ptp.state + (ptp.state == "SLAVE" ? ", offset " +
                std::to_string(ptp.offset_ns) + " ns" : "")).c_str() : "not running (set it up: docs/STATION.md)");

    std::printf("\n%s\n", go ? "GO: this Pi and camera meet the station targets." : "NO-GO: see the FAIL lines above.");
    return go ? 0 : 3;
}

int main(int argc, char **argv) {
    Options o;
    if (!parse(argc, argv, o)) { usage(); return 2; }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);
    if (o.cmd == "run") return cmd_run(o);
    if (o.cmd == "bench") return cmd_bench(o);
    if (o.cmd == "snapshot") return cmd_snapshot(o);
    if (o.cmd == "focus") return cmd_focus(o);
    if (o.cmd == "version") { std::printf("%s (camera support: %s)\n", VERSION_STR, libcamera_available() ? "yes" : "no"); return 0; }
    usage();
    return 2;
}
