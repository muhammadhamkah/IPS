/*
 * Frame sources without a camera: synthetic dots (benchmarks, network tests) and frame files
 * (idps/station/render.py writes them: `--source file:frames.idpsfrm`). Both deliver frames at
 * the configured rate, stamped like the camera would (CLOCK_MONOTONIC at the end of exposure).
 */
#pragma once

#include <time.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <thread>
#include <vector>

#include "clock.hpp"
#include "frame.hpp"

namespace idps {

/* `count` 8- or 16-bit frames with `dots` Gaussian spots each (blur sigma 1.2 px, peaks 80-240
 * of 255, scaled for 10-bit), moving a little from frame to frame, on a noisy dark background. */
inline std::vector<std::vector<uint8_t>> synthetic_frames(int W, int H, int bits, int dots, int count,
                                                          unsigned seed = 1) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> ux(8, W - 9), uy(8, H - 9), upk(80, 240), uph(0, 6.283);
    std::normal_distribution<double> noise(0.0, 1.0);
    const int bpp = bits > 8 ? 2 : 1;
    const double scale = bits > 8 ? 4.0 : 1.0, full = bits > 8 ? 1023 : 255, sigma = 1.2;
    struct D { double x, y, peak, ph; };
    std::vector<D> ds(dots);
    for (auto &d : ds) d = {ux(rng), uy(rng), upk(rng) * scale, uph(rng)};
    std::vector<std::vector<uint8_t>> out(count, std::vector<uint8_t>((size_t)W * H * bpp));
    std::vector<double> img((size_t)W * H);
    for (int k = 0; k < count; k++) {
        for (auto &v : img) v = 4 * scale + noise(rng) * scale;
        for (auto &d : ds) {
            const double cx = d.x + 3 * std::cos(d.ph + 0.05 * k), cy = d.y + 3 * std::sin(d.ph + 0.05 * k);
            for (int y = (int)cy - 5; y <= (int)cy + 6; y++)
                for (int x = (int)cx - 5; x <= (int)cx + 6; x++) {
                    if (x < 0 || y < 0 || x >= W || y >= H) continue;
                    const double r2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
                    img[(size_t)y * W + x] += d.peak * std::exp(-r2 / (2 * sigma * sigma));
                }
        }
        for (size_t i = 0; i < img.size(); i++) {
            const double v = std::max(0.0, std::min(full, std::round(img[i])));
            if (bpp == 1) out[k][i] = (uint8_t)v;
            else { const uint16_t u = (uint16_t)v; std::memcpy(&out[k][2 * i], &u, 2); }
        }
    }
    return out;
}

/* Plays a set of frames in memory at the configured rate, round and round. */
class PlaybackSource : public Source {
  public:
    PlaybackSource(const SourceConfig &c, std::vector<std::vector<uint8_t>> frames, int W, int H, int bpp,
                   std::string what)
        : c_(c), frames_(std::move(frames)), W_(W), H_(H), bpp_(bpp), what_(std::move(what)) {}
    ~PlaybackSource() override { stop(); }

    bool start(std::function<void(Frame &&)> on_frame) override {
        if (frames_.empty()) { error_ = "no frames"; return false; }
        run_ = true;
        th_ = std::thread([this, on_frame] {
            const int64_t period = (int64_t)std::llround(1e9 / c_.fps);
            int64_t next = mono_ns() + period;
            uint64_t seq = 0;
            while (run_) {
                timespec ts{(time_t)(next / 1000000000LL), (long)(next % 1000000000LL)};
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
                Frame f;
                const auto &px = frames_[seq % frames_.size()];
                f.data = px.data();
                f.width = W_; f.height = H_; f.stride = W_; f.bytes_per_pixel = bpp_;
                f.sequence = seq++;
                f.sensor_ns = next;                          // "end of exposure"
                f.exposure_ns = (int64_t)c_.exposure_us * 1000;
                f.arrival_mono_ns = mono_ns();
                f.release = [] {};
                on_frame(std::move(f));
                next += period;
                const int64_t now = mono_ns();
                if (now > next + 10 * period) next = now + period;   // fell far behind: do not burst
            }
        });
        return true;
    }

    void stop() override {
        run_ = false;
        if (th_.joinable()) th_.join();
    }

    std::string describe() const override {
        char b[160];
        std::snprintf(b, sizeof b, "%s, %dx%d %d-bit, %.1f fps", what_.c_str(), W_, H_, bpp_ == 1 ? 8 : 10, c_.fps);
        return b;
    }

  private:
    SourceConfig c_;
    std::vector<std::vector<uint8_t>> frames_;
    int W_, H_, bpp_;
    std::string what_;
    std::atomic<bool> run_{false};
    std::thread th_;
};

inline std::unique_ptr<Source> make_synthetic_source(const SourceConfig &c) {
    auto frames = synthetic_frames(c.width, c.height, c.bits, c.synthetic_dots, 60);
    char what[64];
    std::snprintf(what, sizeof what, "synthetic (%d dots)", c.synthetic_dots);
    return std::make_unique<PlaybackSource>(c, std::move(frames), c.width, c.height, c.bits > 8 ? 2 : 1, what);
}

/* A frame file (idps/station/render.py write_frames). nullptr and `err` set on failure. */
inline std::unique_ptr<Source> make_file_source(const SourceConfig &c, std::string &err) {
    FILE *f = std::fopen(c.file.c_str(), "rb");
    if (!f) { err = "cannot open " + c.file; return nullptr; }
    char magic[8];
    uint32_t hdr[4];
    if (std::fread(magic, 1, 8, f) != 8 || std::memcmp(magic, "IDPSFRM1", 8) != 0 ||
        std::fread(hdr, 4, 4, f) != 4) {
        std::fclose(f);
        err = c.file + ": not an IDPS frame file";
        return nullptr;
    }
    const int W = (int)hdr[0], H = (int)hdr[1], bpp = hdr[2] == 16 ? 2 : 1;
    std::vector<std::vector<uint8_t>> frames(hdr[3], std::vector<uint8_t>((size_t)W * H * bpp));
    for (auto &fr : frames)
        if (std::fread(fr.data(), 1, fr.size(), f) != fr.size()) {
            std::fclose(f);
            err = c.file + ": file is cut short";
            return nullptr;
        }
    std::fclose(f);
    return std::make_unique<PlaybackSource>(c, std::move(frames), W, H, bpp, "file " + c.file);
}

}  // namespace idps
