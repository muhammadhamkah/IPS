/*
 * A captured frame and the frame sources (camera, synthetic, file). A source calls the
 * station's handler for every frame from its own thread; the handler must call release()
 * once it is done with the pixels (the camera gets the buffer back).
 */
#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>

namespace idps {

struct Frame {
    const uint8_t *data = nullptr;   // first pixel
    int width = 0, height = 0;
    int stride = 0;                  // in pixels
    int bytes_per_pixel = 1;         // 1: 8-bit, 2: 16-bit (10-bit values)
    uint64_t sequence = 0;           // sensor frame counter (gaps = frames the camera dropped)
    int64_t sensor_ns = 0;           // the driver's timestamp, in `sensor_clock`
    int64_t exposure_ns = 0;         // the exposure actually used
    int64_t arrival_mono_ns = 0;     // when the station got it (CLOCK_MONOTONIC)
    std::function<void()> release;   // give the buffer back (call exactly once)
};

struct SourceConfig {
    int width = 1280, height = 800;
    double fps = 120.0;
    int exposure_us = 500;
    double gain = 1.0;
    int bits = 8;                    // 8 (RAW8) or 10
    int camera_index = 0;
    std::string file;                // --source file:PATH
    int synthetic_dots = 20;         // --source synthetic
};

class Source {
  public:
    virtual ~Source() = default;
    /* Start capturing; returns false (and sets error()) on failure. */
    virtual bool start(std::function<void(Frame &&)> on_frame) = 0;
    virtual void stop() = 0;
    virtual std::string describe() const = 0;   // what was actually configured
    const std::string &error() const { return error_; }

  protected:
    std::string error_;
};

/* Bounded queue that drops its oldest frame when full (docs/ARCHITECTURE.md principle 6: late
 * data is worth less than no data). Dropped frames are released and counted. */
class FrameRing {
  public:
    explicit FrameRing(size_t cap) : cap_(cap) {}

    void push(Frame &&f) {
        Frame old;
        bool dropped = false;
        {
            std::lock_guard<std::mutex> lk(m_);
            if (q_.size() >= cap_) {
                old = std::move(q_.front());
                q_.pop_front();
                dropped = true;
                drops_++;
            }
            q_.push_back(std::move(f));
        }
        cv_.notify_one();
        if (dropped && old.release) old.release();
    }

    /* Wait for a frame; false once closed and empty. */
    bool pop(Frame &out) {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&] { return !q_.empty() || closed_; });
        if (q_.empty()) return false;
        out = std::move(q_.front());
        q_.pop_front();
        return true;
    }

    void close() {
        {
            std::lock_guard<std::mutex> lk(m_);
            closed_ = true;
        }
        cv_.notify_all();
    }

    /* Release everything still queued (after the consumer stopped). */
    void drain() {
        std::deque<Frame> rest;
        {
            std::lock_guard<std::mutex> lk(m_);
            rest.swap(q_);
        }
        for (auto &f : rest)
            if (f.release) f.release();
    }

    uint64_t drops() const {
        std::lock_guard<std::mutex> lk(m_);
        return drops_;
    }

  private:
    size_t cap_;
    mutable std::mutex m_;
    std::condition_variable cv_;
    std::deque<Frame> q_;
    bool closed_ = false;
    uint64_t drops_ = 0;
};

}  // namespace idps
