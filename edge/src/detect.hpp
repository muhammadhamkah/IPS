/*
 * LED dot detector for one raw monochrome frame (docs/ARCHITECTURE.md section 4.2).
 *
 *   1. Seed scan: every seed_step-th row, pixels >= threshold not yet part of a dot.
 *   2. Region grow: the 8-connected pixels >= threshold_low around each seed (hysteresis).
 *   3. Measure: background = mean of the pixels below threshold_low on the rectangle 2 px
 *      outside the dot's bounding box; centroid = (value - background)-weighted mean over the
 *      box grown by 1 px (the dot's pixels, plus the dim pixels around it, so the faint edge of
 *      the spot is not cut off), sum, peak, area, second moments (two dots merged into one look
 *      elongated) and flags.
 *
 * Coordinates: pixel (0, 0) is the centre of the top-left pixel, x right, y down (as the
 * engine expects: distorted image coordinates). idps/station/detect.py is the same algorithm
 * in Python, step for step; tests compare the two.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace idps {

struct DetectParams {
    int threshold = 60;          // seed level (raw DN)
    int threshold_low = 30;      // a dot's pixels (raw DN)
    int seed_step = 2;           // scan every n-th row for seeds (a dot spans >= this many rows)
    int min_area = 1;            // pixels >= threshold_low
    int max_area = 400;          // larger regions (a lamp, sunlight) are not markers
    int saturation = 255;        // a pixel at or above this is clipped (255 for 8-bit, 1023 for 10-bit)
    double elongation = 1.5;     // sqrt(major / minor second moment) above this: flagged elongated
    int elongation_min_area = 6; // ... only for dots at least this big (small ones are noisy)
    int max_dots = 1000;
};

struct Dot {
    double x, y;      // sub-pixel centroid
    double sum;       // background-subtracted integrated intensity
    int peak;         // brightest pixel (raw DN)
    int area;         // pixels >= threshold_low
    int flags;        // IDPS_MFLAG_* (idps_protocol.h): 1 saturated, 2 edge, 4 elongated
};

enum { DOT_SATURATED = 1, DOT_EDGE = 2, DOT_ELONGATED = 4 };

class Detector {
  public:
    explicit Detector(DetectParams p = {}) : p_(p) {}
    const DetectParams &params() const { return p_; }
    void set_params(const DetectParams &p) { p_ = p; }

    /* Detect the dots of one frame. img: rows of `stride` pixels (stride >= width). */
    template <typename T>
    const std::vector<Dot> &detect(const T *img, int width, int height, int stride) {
        dots_.clear();
        if (width <= 0 || height <= 0) return dots_;
        prepare(width, height);
        const int thr = p_.threshold, lo = p_.threshold_low;
        const int step = std::max(1, p_.seed_step);
        for (int y = 0; y < height; y += step) {
            const T *row = img + (size_t)y * stride;
            int x = 0;
            while (x < width) {
                // skip dark runs fast: the max of a block of 32 is a vectorisable reduction
                if (x + 32 <= width) {
                    T m = 0;
                    for (int k = 0; k < 32; k++) m = std::max(m, row[x + k]);
                    if ((int)m < thr) { x += 32; continue; }
                }
                const int end = std::min(width, x + 32);
                for (; x < end; x++) {
                    if ((int)row[x] >= thr && label_[(size_t)y * width + x] < first_) {
                        grow(img, width, height, stride, x, y, lo);
                        if ((int)dots_.size() >= p_.max_dots) return dots_;
                    }
                }
            }
        }
        return dots_;
    }

  private:
    template <typename T>
    void grow(const T *img, int W, int H, int stride, int sx, int sy, int lo) {
        // flood fill (8-connected) of the pixels >= lo; every dot gets its own number in label_,
        // and numbers below first_ are earlier frames' (so label_ is never cleared)
        const uint32_t id = next_++;
        region_.clear();
        stack_.clear();
        stack_.push_back((uint32_t)sy * W + sx);
        label_[(size_t)sy * W + sx] = id;
        int x0 = sx, x1 = sx, y0 = sy, y1 = sy, peak = 0;
        while (!stack_.empty()) {
            const uint32_t i = stack_.back();
            stack_.pop_back();
            region_.push_back(i);
            const int x = (int)(i % W), y = (int)(i / W);
            x0 = std::min(x0, x); x1 = std::max(x1, x);
            y0 = std::min(y0, y); y1 = std::max(y1, y);
            peak = std::max(peak, (int)img[(size_t)y * stride + x]);
            for (int dy = -1; dy <= 1; dy++) {
                const int yy = y + dy;
                if (yy < 0 || yy >= H) continue;
                for (int dx = -1; dx <= 1; dx++) {
                    const int xx = x + dx;
                    if (xx < 0 || xx >= W || (dx == 0 && dy == 0)) continue;
                    const size_t j = (size_t)yy * W + xx;
                    if (label_[j] < first_ && (int)img[(size_t)yy * stride + xx] >= lo) {
                        label_[j] = id;
                        stack_.push_back((uint32_t)j);
                    }
                }
            }
        }
        const int area = (int)region_.size();
        if (area < p_.min_area || area > p_.max_area) return;

        // background: the pixels below lo on the rectangle 2 px outside the bounding box
        double bsum = 0;
        int bn = 0;
        const int bx0 = x0 - 2, bx1 = x1 + 2, by0 = y0 - 2, by1 = y1 + 2;
        auto ring = [&](int x, int y) {
            if (x < 0 || y < 0 || x >= W || y >= H) return;
            const int v = img[(size_t)y * stride + x];
            if (v < lo) { bsum += v; bn++; }
        };
        for (int x = bx0; x <= bx1; x++) { ring(x, by0); ring(x, by1); }
        for (int y = by0 + 1; y <= by1 - 1; y++) { ring(bx0, y); ring(bx1, y); }
        const double bg = bn ? bsum / bn : 0.0;

        // centroid over the box grown by 1 px: this dot's pixels and the dim pixels around it
        // (a bright pixel there that is not this dot's belongs to a neighbouring dot)
        const int wx0 = std::max(0, x0 - 1), wx1 = std::min(W - 1, x1 + 1);
        const int wy0 = std::max(0, y0 - 1), wy1 = std::min(H - 1, y1 + 1);
        double s = 0, sx_ = 0, sy_ = 0;
        for (int y = wy0; y <= wy1; y++) {
            const T *row = img + (size_t)y * stride;
            for (int x = wx0; x <= wx1; x++) {
                const int v = row[x];
                if (v >= lo && label_[(size_t)y * W + x] != id) continue;
                const double w = v - bg;
                if (w <= 0) continue;
                s += w; sx_ += w * x; sy_ += w * y;
            }
        }
        if (s <= 0) return;
        const double cx = sx_ / s, cy = sy_ / s;

        // second moments of the dot's own pixels: two dots merged look elongated
        int flags = 0;
        if (area >= p_.elongation_min_area) {
            double mxx = 0, myy = 0, mxy = 0, mw = 0;
            for (uint32_t i : region_) {
                const int x = (int)(i % W), y = (int)(i / W);
                const double w = img[(size_t)y * stride + x] - bg;
                if (w <= 0) continue;
                const double dx = x - cx, dy = y - cy;
                mxx += w * dx * dx; myy += w * dy * dy; mxy += w * dx * dy; mw += w;
            }
            if (mw > 0) {
                mxx /= mw; myy /= mw; mxy /= mw;
                const double tr = mxx + myy, d = std::sqrt(std::max(0.0, (mxx - myy) * (mxx - myy) / 4 + mxy * mxy));
                const double l1 = tr / 2 + d, l2 = std::max(tr / 2 - d, 1e-6);
                if (std::sqrt(l1 / l2) > p_.elongation) flags |= DOT_ELONGATED;
            }
        }
        if (peak >= p_.saturation) flags |= DOT_SATURATED;
        if (x0 == 0 || y0 == 0 || x1 == W - 1 || y1 == H - 1) flags |= DOT_EDGE;
        dots_.push_back({cx, cy, s, peak, area, flags});
    }

    void prepare(int W, int H) {
        const size_t n = (size_t)W * H;
        if (label_.size() != n || next_ > 0xF0000000u) {   // new size, or the numbers ran out: clear
            label_.assign(n, 0);
            next_ = 1;
        }
        first_ = next_;
    }

    DetectParams p_;
    std::vector<uint32_t> label_;
    uint32_t next_ = 1, first_ = 1;
    std::vector<uint32_t> region_, stack_;
    std::vector<Dot> dots_;
};

}  // namespace idps
