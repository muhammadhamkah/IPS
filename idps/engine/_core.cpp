// IDPS positioning engine core in C++ (idps.engine._core).
//
// A faithful port of the per-epoch work in idps/engine/engine.py (the Python engine stays
// as the reference and the fallback): association, triangulation, Kalman filtering,
// fixed-reflection masks, starting tracks, life cycle, ID hand-over, duplicate removal and
// blink-code decoding. Python keeps packet ingest, epoch grouping and output formatting.
//
// Frames: world = planner drawing frame (metres, Z up). Camera: R maps world -> camera
// (rows x right, y down, z forward), Xc = R (X - pos).
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace idps {

using Vec3 = std::array<double, 3>;
static const double INF = std::numeric_limits<double>::infinity();

// ------------------------------------------------------------------ small linear algebra
static inline Vec3 sub(const Vec3 &a, const Vec3 &b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
static inline double dot(const Vec3 &a, const Vec3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
static inline double norm(const Vec3 &a) { return std::sqrt(dot(a, a)); }

// 3x3 inverse (row-major); false if singular
static bool inv3(const double *A, double *out) {
    double c00 = A[4] * A[8] - A[5] * A[7], c01 = A[5] * A[6] - A[3] * A[8], c02 = A[3] * A[7] - A[4] * A[6];
    double det = A[0] * c00 + A[1] * c01 + A[2] * c02;
    if (!(std::fabs(det) > 1e-300) || !std::isfinite(det)) return false;
    double id = 1.0 / det;
    out[0] = c00 * id;
    out[1] = (A[2] * A[7] - A[1] * A[8]) * id;
    out[2] = (A[1] * A[5] - A[2] * A[4]) * id;
    out[3] = c01 * id;
    out[4] = (A[0] * A[8] - A[2] * A[6]) * id;
    out[5] = (A[2] * A[3] - A[0] * A[5]) * id;
    out[6] = c02 * id;
    out[7] = (A[1] * A[6] - A[0] * A[7]) * id;
    out[8] = (A[0] * A[4] - A[1] * A[3]) * id;
    return true;
}

// eigenvalues of a symmetric 3x3 (analytic); returns condition number max/min (inf if min <= 0)
static double cond_sym3(const double *A) {
    double p1 = A[1] * A[1] + A[2] * A[2] + A[5] * A[5];
    double q = (A[0] + A[4] + A[8]) / 3.0;
    double e1, e2, e3;
    if (p1 < 1e-300) {
        e1 = A[0]; e2 = A[4]; e3 = A[8];
    } else {
        double p2 = (A[0] - q) * (A[0] - q) + (A[4] - q) * (A[4] - q) + (A[8] - q) * (A[8] - q) + 2 * p1;
        double p = std::sqrt(p2 / 6.0);
        double B[9];
        for (int i = 0; i < 9; i++) B[i] = (A[i] - (i % 4 == 0 ? q : 0.0)) / p;
        double r = (B[0] * (B[4] * B[8] - B[5] * B[7]) - B[1] * (B[3] * B[8] - B[5] * B[6]) +
                    B[2] * (B[3] * B[7] - B[4] * B[6])) / 2.0;
        r = std::max(-1.0, std::min(1.0, r));
        double phi = std::acos(r) / 3.0;
        e1 = q + 2 * p * std::cos(phi);
        e3 = q + 2 * p * std::cos(phi + 2.0 * M_PI / 3.0);
        e2 = 3 * q - e1 - e3;
    }
    double mx = std::max({std::fabs(e1), std::fabs(e2), std::fabs(e3)});
    double mn = std::min({e1, e2, e3});
    return mn > 0 ? mx / mn : INF;
}

// ------------------------------------------------------------------ cameras
struct Cam {
    int station;
    Vec3 pos;
    double R[9];
    double f, cx, cy, k[5];
    int w, h;
    double sigma_ang;

    Vec3 ray(double x, double y) const {
        const double k1 = k[0], k2 = k[1], p1 = k[2], p2 = k[3], k3 = k[4];
        double xd = (x - cx) / f, yd = (y - cy) / f, xn = xd, yn = yd;
        for (int it = 0; it < 8; it++) {
            double r2 = xn * xn + yn * yn;
            double radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
            double dx = 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn);
            double dy = p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn;
            xn = (xd - dx) / radial;
            yn = (yd - dy) / radial;
        }
        Vec3 d{xn * R[0] + yn * R[3] + R[6], xn * R[1] + yn * R[4] + R[7], xn * R[2] + yn * R[5] + R[8]};
        double n = norm(d);
        return {d[0] / n, d[1] / n, d[2] / n};
    }

    // world point -> distorted pixel (u, v) and depth z
    void project(const Vec3 &X, double &u, double &v, double &z) const {
        Vec3 d = sub(X, pos);
        double xc = R[0] * d[0] + R[1] * d[1] + R[2] * d[2];
        double yc = R[3] * d[0] + R[4] * d[1] + R[5] * d[2];
        z = R[6] * d[0] + R[7] * d[1] + R[8] * d[2];
        double zs = z > 1e-9 ? z : 1e-9;
        double xn = xc / zs, yn = yc / zs;
        const double k1 = k[0], k2 = k[1], p1 = k[2], p2 = k[3], k3 = k[4];
        double r2 = xn * xn + yn * yn;
        double radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        u = cx + f * (xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn));
        v = cy + f * (yn * radial + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn);
    }
};

// ------------------------------------------------------------------ fixed reflections
struct StaticMask {
    double hold = 2.0, r = 0.5;
    std::vector<double> x, y, first, last;

    int nearest(double px, double py, double &dist) const {
        int j = -1;
        dist = INF;
        for (size_t i = 0; i < x.size(); i++) {
            double d = std::max(std::fabs(px - x[i]), std::fabs(py - y[i]));
            if (d < dist) { dist = d; j = (int)i; }
        }
        return j;
    }
    bool known(double px, double py, double t) const {
        double d;
        int j = nearest(px, py, d);
        return j >= 0 && d < r && t - first[j] > hold;
    }
    // classify a frame's leftover dots (updates the spots)
    std::vector<char> classify(const std::vector<std::array<double, 2>> &xy, double t) {
        size_t n = xy.size();
        std::vector<char> st(n, 0), isnew(n, 1);
        std::vector<int> hit(n, -1);
        if (!x.empty()) {
            for (size_t i = 0; i < n; i++) {
                double d;
                int j = nearest(xy[i][0], xy[i][1], d);
                if (j >= 0 && d < r) { hit[i] = j; isnew[i] = 0; }
            }
            // update (like the vectorised Python: all hits see the spot's old 'last')
            std::vector<double> old_last = last;
            for (size_t i = 0; i < n; i++) {
                int j = hit[i];
                if (j < 0) continue;
                if (t - old_last[j] > 0.1) first[j] = t;
                last[j] = t;
                st[i] = t - first[j] > hold;
            }
        }
        for (size_t i = 0; i < n; i++)
            if (isnew[i]) { x.push_back(xy[i][0]); y.push_back(xy[i][1]); first.push_back(t); last.push_back(t); }
        size_t w = 0;
        for (size_t i = 0; i < x.size(); i++)
            if (t - last[i] <= 0.1) { x[w] = x[i]; y[w] = y[i]; first[w] = first[i]; last[w] = last[i]; w++; }
        x.resize(w); y.resize(w); first.resize(w); last.resize(w);
        return st;
    }
};

// ------------------------------------------------------------------ blink-code decoder
static const int CHIPS = 30;
static int crc4(int value, int nbits = 8) {
    int reg = 0;
    for (int i = nbits - 1; i >= 0; i--) {
        int bit = (value >> i) & 1, top = (reg >> 3) & 1;
        reg = (reg << 1) & 0xF;
        if (bit ^ top) reg ^= 0x3;
    }
    return reg;
}
static int decode_chips(const std::vector<int> &c, size_t s) {
    static const int SYNC[6] = {1, 1, 1, 0, 0, 0};
    for (int i = 0; i < 6; i++)
        if (c[s + i] != SYNC[i]) return -1;
    int word = 0;
    for (int b = 0; b < 12; b++) {
        int a = c[s + 6 + 2 * b], bb = c[s + 7 + 2 * b];
        int bit;
        if (a == 1 && bb == 0) bit = 1;
        else if (a == 0 && bb == 1) bit = 0;
        else return -1;
        word = (word << 1) | bit;
    }
    int mid = word >> 4, crc = word & 0xF;
    return crc4(mid) == crc ? mid : -1;
}
// returns (marker id or -1, valid codes)
static std::pair<int, int> decode_blink(const std::vector<double> &t, const std::vector<double> &lv,
                                        double chip_rate = 30.0, int phases = 24, double edge = 0.15) {
    size_t n = t.size();
    if (n < 30) return {-1, 0};
    double T = 1.0 / chip_rate;
    double best_c = -1;
    std::vector<double> best_mean;
    std::vector<char> best_have;
    std::vector<long> idx(n);
    std::vector<char> keep(n);
    for (int k = 0; k < phases; k++) {
        double ph = k * T / phases;
        long kept = 0, mx = -1;
        for (size_t i = 0; i < n; i++) {
            double x = (t[i] - t[0] + T - ph) / T;
            long ix = (long)std::floor(x);
            double fr = x - ix;
            idx[i] = ix;
            keep[i] = fr > edge && fr < 1 - edge;
            if (keep[i]) { kept++; mx = std::max(mx, ix); }
        }
        if (kept < 20) continue;
        std::vector<double> s(mx + 1, 0.0);
        std::vector<long> c(mx + 1, 0);
        for (size_t i = 0; i < n; i++)
            if (keep[i]) { s[idx[i]] += lv[i]; c[idx[i]]++; }
        std::vector<double> mean(mx + 1, 0.0);
        std::vector<char> have(mx + 1, 0);
        double acc = 0;
        long nh = 0;
        for (long j = 0; j <= mx; j++)
            if (c[j] > 0) { have[j] = 1; mean[j] = s[j] / c[j]; acc += std::fabs(mean[j]); nh++; }
        double contrast = nh ? acc / nh : 0.0;
        if (contrast > best_c) { best_c = contrast; best_mean = mean; best_have = have; }
    }
    if (best_c < 0) return {-1, 0};
    std::vector<int> chips(best_mean.size());
    for (size_t j = 0; j < chips.size(); j++) chips[j] = best_mean[j] > 0 ? 1 : 0;
    std::vector<std::pair<int, int>> votes;   // insertion order, like the Python dict
    if (chips.size() >= (size_t)CHIPS) {
        for (size_t s0 = 0; s0 + CHIPS <= chips.size(); s0++) {
            bool all = true;
            for (int j = 0; j < CHIPS; j++)
                if (!best_have[s0 + j]) { all = false; break; }
            if (!all) continue;
            int mid = decode_chips(chips, s0);
            if (mid < 0) continue;
            bool found = false;
            for (auto &v : votes)
                if (v.first == mid) { v.second++; found = true; break; }
            if (!found) votes.push_back({mid, 1});
        }
    }
    if (votes.empty()) return {-1, 0};
    auto best = votes[0];
    for (auto &v : votes)
        if (v.second > best.second) best = v;
    return best;
}

// ------------------------------------------------------------------ tracks
struct Votes {
    std::vector<std::pair<int, int>> v;    // insertion-ordered (id, votes)
    int get(int id) const {
        for (auto &e : v)
            if (e.first == id) return e.second;
        return 0;
    }
    void add(int id, int n) {
        for (auto &e : v)
            if (e.first == id) { e.second += n; return; }
        v.push_back({id, n});
    }
    int best() const {
        if (v.empty()) return -1;
        auto b = v[0];
        for (auto &e : v)
            if (e.second > b.second) b = e;
        return b.first;
    }
};

struct Sample { double t; int station; double lv; };

struct Track {
    int tid;
    double x[6];
    double P[36];
    double t;
    bool confirmed = false;
    int hits = 1, epochs = 1, n_cams = 0, rejects = 0;
    long updated_epoch = -1;
    double born = 0, last_update = 0, next_decode = 0;
    int marker_id = -1;
    Votes votes;
    std::deque<Sample> samples;

    Vec3 pos() const { return {x[0], x[1], x[2]}; }
    Vec3 vel() const { return {x[3], x[4], x[5]}; }
    void add_brightness(double tt, int station, double value) {
        samples.push_back({tt, station, std::log(std::max(value, 1.0))});
        if (samples.size() > 6000) samples.pop_front();
    }
    void predict(double tnew, double q) {
        double dt = std::max(tnew - t, 0.0);
        for (int i = 0; i < 3; i++) x[i] += x[3 + i] * dt;
        // P = F P F^T + Q, F = [[I, dt I], [0, I]]
        double Pn[36];
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++) {
                double a = P[i * 6 + j];
                if (i < 3) a += dt * P[(i + 3) * 6 + j];
                Pn[i * 6 + j] = a;
            }
        double Pm[36];
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++) {
                double a = Pn[i * 6 + j];
                if (j < 3) a += dt * Pn[i * 6 + j + 3];
                Pm[i * 6 + j] = a;
            }
        for (int i = 0; i < 3; i++) {
            Pm[i * 6 + i] += q * dt * dt * dt / 3;
            Pm[i * 6 + i + 3] += q * dt * dt / 2;
            Pm[(i + 3) * 6 + i] += q * dt * dt / 2;
            Pm[(i + 3) * 6 + i + 3] += q * dt;
        }
        std::copy(Pm, Pm + 36, P);
        t = std::max(tnew, t);
    }
    // levels with each station's 1 s centred moving mean removed, sorted by time
    void levels(double since, std::vector<double> &tout, std::vector<double> &lout) const {
        // group the window's samples by station in one pass (stations in sorted order, as numpy.unique)
        std::vector<int> stations;
        std::vector<std::vector<std::pair<double, double>>> per;
        for (auto &s : samples) {
            if (s.t < since - 0.5) continue;
            size_t k = std::lower_bound(stations.begin(), stations.end(), s.station) - stations.begin();
            if (k == stations.size() || stations[k] != s.station) {
                stations.insert(stations.begin() + k, s.station);
                per.insert(per.begin() + k, std::vector<std::pair<double, double>>());
            }
            per[k].push_back({s.t, s.lv});
        }
        std::vector<std::pair<double, double>> out;
        std::vector<double> tt, c;
        for (auto &g : per) {
            size_t m = g.size();
            tt.resize(m);
            c.assign(m + 1, 0.0);
            for (size_t i = 0; i < m; i++) { tt[i] = g[i].first; c[i + 1] = c[i] + g[i].second; }
            for (size_t i = 0; i < m; i++) {
                if (!(tt[i] - 0.5 >= tt[0] && tt[i] + 0.5 <= tt[m - 1])) continue;
                if (tt[i] < since) continue;
                size_t j0 = std::lower_bound(tt.begin(), tt.end(), tt[i] - 0.5) - tt.begin();
                size_t j1 = std::upper_bound(tt.begin(), tt.end(), tt[i] + 0.5) - tt.begin();
                double mean = (c[j1] - c[j0]) / std::max<double>(j1 - j0, 1);
                out.push_back({tt[i], g[i].second - mean});
            }
        }
        std::stable_sort(out.begin(), out.end(),
                         [](const std::pair<double, double> &x, const std::pair<double, double> &y) { return x.first < y.first; });
        tout.clear(); lout.clear();
        for (auto &q : out) { tout.push_back(q.first); lout.push_back(q.second); }
    }
};

struct Lost { double t; int mid; Vec3 pos, vel; Votes votes; };

struct Config {
    double reorder_s = 0.008, sigma_px = 0.15, gate_px = 4.0, max_resid_px = 3.0, init_gap_m = 0.03,
           init_support_px = 2.5, coast_s = 0.5, q_accel = 400.0, innov_gate = 25.0, inherit_radius_m = 0.5,
           inherit_within_s = 1.0, decode_every_s = 0.5, decode_every_named_s = 2.0, decode_window_s = 2.5;
    int min_cams_init = 3, min_cams_update = 2, confirm_hits = 8, confirm_window = 16, innov_max_rejects = 3,
        id_votes_needed = 2, decodes_per_epoch = 2;
};

// ------------------------------------------------------------------ triangulation
struct Obs { int frame, dot; };

static bool solve_rays(const std::vector<Vec3> &O, const std::vector<Vec3> &D, const std::vector<double> &w,
                       const std::vector<char> &use, Vec3 &X, double *A) {
    double b[3] = {0, 0, 0};
    std::fill(A, A + 9, 0.0);
    for (size_t k = 0; k < O.size(); k++) {
        if (!use[k]) continue;
        const Vec3 &d = D[k];
        double Pm[9];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) Pm[i * 3 + j] = (i == j ? 1.0 : 0.0) - d[i] * d[j];
        for (int i = 0; i < 9; i++) A[i] += w[k] * Pm[i];
        for (int i = 0; i < 3; i++) b[i] += w[k] * (Pm[i * 3] * O[k][0] + Pm[i * 3 + 1] * O[k][1] + Pm[i * 3 + 2] * O[k][2]);
    }
    double Ai[9];
    if (!inv3(A, Ai)) return false;
    for (int i = 0; i < 3; i++) X[i] = Ai[i * 3] * b[0] + Ai[i * 3 + 1] * b[1] + Ai[i * 3 + 2] * b[2];
    return true;
}

static void residuals(const std::vector<Vec3> &O, const std::vector<Vec3> &D, const std::vector<double> &F,
                      const Vec3 &X, std::vector<double> &res) {
    res.resize(O.size());
    for (size_t k = 0; k < O.size(); k++) {
        Vec3 v = sub(X, O[k]);
        double r = norm(v);
        double c = std::max(-1.0, std::min(1.0, dot(v, D[k]) / std::max(r, 1e-9)));
        res[k] = c > 0 ? std::acos(c) * F[k] : INF;
    }
}

// O are already motion-shifted origins. With X0: range weights from X0 and one solve (tracked
// markers); without: unweighted solve then two re-weightings and a conditioning check.
static bool triangulate(const std::vector<Vec3> &O, const std::vector<Vec3> &D, const std::vector<double> &F,
                        double sigma, double max_resid, int min_cams, const Vec3 *X0, int max_drops, Vec3 &X,
                        double *cov, std::vector<char> &use) {
    size_t k = O.size();
    use.assign(k, 1);
    std::vector<double> w(k, 1.0), res;
    double A[9];
    for (int iter = 0;; iter++) {
        int cnt = 0;
        for (char u : use) cnt += u;
        if (cnt < min_cams) return false;
        int rounds;
        if (X0) {
            X = *X0;
            rounds = 1;
        } else {
            std::fill(w.begin(), w.end(), 1.0);
            if (!solve_rays(O, D, w, use, X, A)) return false;
            rounds = 2;
        }
        for (int r = 0; r < rounds; r++) {
            for (size_t i = 0; i < k; i++) w[i] = 1.0 / std::pow(std::max(norm(sub(X, O[i])), 0.1), 2);
            if (!solve_rays(O, D, w, use, X, A)) return false;
        }
        residuals(O, D, F, X, res);
        int worst = -1;
        double wv = -1;
        for (size_t i = 0; i < k; i++)
            if (use[i] && res[i] > wv) { wv = res[i]; worst = (int)i; }
        if (wv > max_resid) {
            if (cnt > min_cams && iter < max_drops) { use[worst] = 0; continue; }
            return false;
        }
        if (!X0 && cond_sym3(A) > 1e8) return false;
        double Ai[9];
        if (!inv3(A, Ai)) return false;
        for (int i = 0; i < 9; i++) cov[i] = sigma * sigma * Ai[i];
        return true;
    }
}


// Solve S Y = B for symmetric positive-definite S (m x m) and B (m x c), in place in B.
static bool chol_solve(std::vector<double> &S, int m, std::vector<double> &B, int c) {
    for (int j = 0; j < m; j++) {
        double d = S[j * m + j];
        for (int k = 0; k < j; k++) d -= S[j * m + k] * S[j * m + k];
        if (!(d > 0)) return false;
        d = std::sqrt(d);
        S[j * m + j] = d;
        for (int i = j + 1; i < m; i++) {
            double a = S[i * m + j];
            for (int k = 0; k < j; k++) a -= S[i * m + k] * S[j * m + k];
            S[i * m + j] = a / d;
        }
    }
    for (int col = 0; col < c; col++) {
        for (int i = 0; i < m; i++) {                    // L y = b
            double a = B[i * c + col];
            for (int k = 0; k < i; k++) a -= S[i * m + k] * B[k * c + col];
            B[i * c + col] = a / S[i * m + i];
        }
        for (int i = m - 1; i >= 0; i--) {               // L^T x = y
            double a = B[i * c + col];
            for (int k = i + 1; k < m; k++) a -= S[k * m + i] * B[k * c + col];
            B[i * c + col] = a / S[i * m + i];
        }
    }
    return true;
}

// Kalman update of [p, v] at tr.t from lines of sight: camera k at O[k] saw the marker along
// D[k] at tr.t + dt[k]; the components of (p + v dt - O) across the line are zero.
static void bearing_update(Track &tr, const std::vector<Vec3> &O, const std::vector<Vec3> &D,
                           const std::vector<double> &dt, double sigma_ang) {
    const int m = 2 * (int)O.size();
    if (m == 0) return;
    std::vector<double> H((size_t)m * 6), z(m), r2(m);
    for (size_t k = 0; k < O.size(); k++) {
        const Vec3 &d = D[k];
        Vec3 a = std::fabs(d[0]) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
        double ad = dot(a, d);
        Vec3 u{a[0] - ad * d[0], a[1] - ad * d[1], a[2] - ad * d[2]};
        double un = norm(u);
        u = {u[0] / un, u[1] / un, u[2] / un};
        Vec3 w{d[1] * u[2] - d[2] * u[1], d[2] * u[0] - d[0] * u[2], d[0] * u[1] - d[1] * u[0]};
        Vec3 q{tr.x[0] + tr.x[3] * dt[k] - O[k][0], tr.x[1] + tr.x[4] * dt[k] - O[k][1], tr.x[2] + tr.x[5] * dt[k] - O[k][2]};
        double rng = std::max(norm(q), 0.1);
        const Vec3 *e[2] = {&u, &w};
        for (int r = 0; r < 2; r++) {
            int row = 2 * (int)k + r;
            for (int j = 0; j < 3; j++) {
                H[row * 6 + j] = (*e[r])[j];
                H[row * 6 + 3 + j] = (*e[r])[j] * dt[k];
            }
            z[row] = dot(*e[r], O[k]);
            r2[row] = (sigma_ang * rng) * (sigma_ang * rng);
        }
    }
    // HP = H P (m x 6); S = HP H^T + R
    std::vector<double> HP((size_t)m * 6, 0.0), S((size_t)m * m, 0.0);
    for (int i = 0; i < m; i++)
        for (int j = 0; j < 6; j++) {
            double a = 0;
            for (int k = 0; k < 6; k++) a += H[i * 6 + k] * tr.P[k * 6 + j];
            HP[i * 6 + j] = a;
        }
    for (int i = 0; i < m; i++)
        for (int j = 0; j < m; j++) {
            double a = 0;
            for (int k = 0; k < 6; k++) a += HP[i * 6 + k] * H[j * 6 + k];
            S[i * m + j] = a + (i == j ? r2[i] : 0.0);
        }
    std::vector<double> Y = HP;                          // Y = S^-1 H P  (m x 6), K = Y^T
    if (!chol_solve(S, m, Y, 6)) return;
    std::vector<double> nu(m);
    for (int i = 0; i < m; i++) {
        double a = z[i];
        for (int k = 0; k < 6; k++) a -= H[i * 6 + k] * tr.x[k];
        nu[i] = a;
    }
    for (int j = 0; j < 6; j++) {
        double a = 0;
        for (int i = 0; i < m; i++) a += Y[i * 6 + j] * nu[i];
        tr.x[j] += a;
    }
    double Pn[36];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            double a = 0;
            for (int k = 0; k < m; k++) a += Y[k * 6 + i] * HP[k * 6 + j];
            Pn[i * 6 + j] = tr.P[i * 6 + j] - a;
        }
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) tr.P[i * 6 + j] = 0.5 * (Pn[i * 6 + j] + Pn[j * 6 + i]);
}

// ------------------------------------------------------------------ the engine core
class Core {
  public:
    Config cfg;
    std::vector<Cam> cams;
    std::vector<StaticMask> masks;
    Vec3 lo, hi;
    std::vector<Track> tracks;
    std::vector<Lost> lost;
    int n_tracks = 0, tracks_started = 0;
    long epochs = 0;
    double max_sigma = 0;

    Core(py::array_t<int> station, py::array_t<double> pos, py::array_t<double> R, py::array_t<double> f,
         py::array_t<double> cx, py::array_t<double> cy, py::array_t<double> dist, py::array_t<int> wh,
         py::array_t<double> bounds_lo, py::array_t<double> bounds_hi, py::dict conf) {
        auto st = station.unchecked<1>();
        auto P = pos.unchecked<2>();
        auto Rr = R.unchecked<3>();
        auto ff = f.unchecked<1>();
        auto cxx = cx.unchecked<1>();
        auto cyy = cy.unchecked<1>();
        auto dd = dist.unchecked<2>();
        auto w = wh.unchecked<2>();
        for (auto item : conf) {
            std::string key = py::str(item.first);
            py::handle v = item.second;
#define SETD(n) if (key == #n) cfg.n = v.cast<double>();
#define SETI(n) if (key == #n) cfg.n = v.cast<int>();
            SETD(reorder_s) SETD(sigma_px) SETD(gate_px) SETD(max_resid_px) SETD(init_gap_m) SETD(init_support_px)
            SETD(coast_s) SETD(q_accel) SETD(innov_gate) SETD(inherit_radius_m) SETD(inherit_within_s)
            SETD(decode_every_s) SETD(decode_every_named_s) SETD(decode_window_s)
            SETI(min_cams_init) SETI(min_cams_update) SETI(confirm_hits) SETI(confirm_window) SETI(innov_max_rejects)
            SETI(id_votes_needed) SETI(decodes_per_epoch)
#undef SETD
#undef SETI
        }
        for (py::ssize_t i = 0; i < st.shape(0); i++) {
            Cam c;
            c.station = st(i);
            c.pos = {P(i, 0), P(i, 1), P(i, 2)};
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 3; b++) c.R[a * 3 + b] = Rr(i, a, b);
            c.f = ff(i); c.cx = cxx(i); c.cy = cyy(i);
            for (int j = 0; j < 5; j++) c.k[j] = dd(i, j);
            c.w = w(i, 0); c.h = w(i, 1);
            c.sigma_ang = cfg.sigma_px / c.f;
            max_sigma = std::max(max_sigma, c.sigma_ang);
            cams.push_back(c);
        }
        masks.resize(cams.size());
        auto l = bounds_lo.unchecked<1>();
        auto h = bounds_hi.unchecked<1>();
        lo = {l(0), l(1), l(2)};
        hi = {h(0), h(1), h(2)};
    }

    // one epoch: frames (camera index, time s) and their dots
    py::dict process(py::array_t<int> fcam_a, py::array_t<double> ft_a, py::array_t<int> dframe_a,
                     py::array_t<double> xy_a, py::array_t<double> sum_a, py::array_t<int> flags_a) {
        auto fcam = fcam_a.unchecked<1>();
        auto ftm = ft_a.unchecked<1>();
        auto dfr = dframe_a.unchecked<1>();
        auto xy = xy_a.unchecked<2>();
        auto sm = sum_a.unchecked<1>();
        auto fl = flags_a.unchecked<1>();
        const int F = (int)fcam.shape(0), N = (int)dfr.shape(0);
        epochs++;
        std::vector<double> times(F);
        for (int i = 0; i < F; i++) times[i] = ftm(i);
        std::vector<double> srt = times;
        std::sort(srt.begin(), srt.end());
        const double T = F % 2 ? srt[F / 2] : 0.5 * (srt[F / 2 - 1] + srt[F / 2]);
        // dots per frame, and lines of sight
        std::vector<std::vector<int>> fdots(F);
        for (int i = 0; i < N; i++) fdots[dfr(i)].push_back(i);
        std::vector<Vec3> rays(N);
        for (int i = 0; i < N; i++) rays[i] = cams[fcam(dfr(i))].ray(xy(i, 0), xy(i, 1));
        for (auto &tr : tracks) tr.epochs++;

        std::vector<char> used(N, 0);
        // ---------------- association
        const int NT = (int)tracks.size();
        std::vector<std::vector<Obs>> assigned(NT);
        if (NT) {
            std::vector<Vec3> pos(NT), vel(NT);
            std::vector<double> tk(NT), sig(NT);
            std::vector<char> moving(NT);
            for (int ti = 0; ti < NT; ti++) {
                const Track &tr = tracks[ti];
                pos[ti] = tr.pos(); vel[ti] = tr.vel(); tk[ti] = tr.t;
                double s = std::sqrt((tr.P[0] + tr.P[7] + tr.P[14]) / 3.0);
                double g = std::max(T - tr.t, 0.0);
                sig[ti] = std::hypot(s, std::sqrt(cfg.q_accel * g * g * g / 3.0));
                moving[ti] = norm(vel[ti]) > 0.05;
            }
            struct Cand { double d; int ti, fi, di; };
            std::vector<Cand> cand;
            for (int fi = 0; fi < F; fi++) {
                if (fdots[fi].empty()) continue;
                const Cam &cam = cams[fcam(fi)];
                const StaticMask &mask = masks[fcam(fi)];
                std::vector<char> refl(fdots[fi].size());
                for (size_t a = 0; a < fdots[fi].size(); a++) {
                    int di = fdots[fi][a];
                    refl[a] = mask.known(xy(di, 0), xy(di, 1), times[fi]);
                }
                const double m = cfg.gate_px;
                for (int ti = 0; ti < NT; ti++) {
                    double dtt = times[fi] - tk[ti];
                    Vec3 Xp{pos[ti][0] + vel[ti][0] * dtt, pos[ti][1] + vel[ti][1] * dtt, pos[ti][2] + vel[ti][2] * dtt};
                    double u, v, z;
                    cam.project(Xp, u, v, z);
                    if (!(z > 0.3 && u >= -m && u <= cam.w - 1 + m && v >= -m && v <= cam.h - 1 + m)) continue;
                    double gate = cfg.gate_px + 3 * sig[ti] * cam.f / std::max(z, 0.3);
                    for (size_t a = 0; a < fdots[fi].size(); a++) {
                        int di = fdots[fi][a];
                        double d = std::hypot(xy(di, 0) - u, xy(di, 1) - v);
                        if (d < gate && !(moving[ti] && refl[a])) cand.push_back({d, ti, fi, (int)a});
                    }
                }
            }
            std::sort(cand.begin(), cand.end(), [](const Cand &a, const Cand &b) {
                return std::tie(a.d, a.ti, a.fi, a.di) < std::tie(b.d, b.ti, b.fi, b.di);
            });
            std::vector<char> taken_tf((size_t)NT * F, 0);
            for (auto &c : cand) {
                int dot_i = fdots[c.fi][c.di];
                if (used[dot_i] || taken_tf[(size_t)c.ti * F + c.fi]) continue;
                used[dot_i] = 1;
                taken_tf[(size_t)c.ti * F + c.fi] = 1;
                assigned[c.ti].push_back({c.fi, dot_i});
            }
        }

        // ---------------- update tracks
        for (int ti = 0; ti < NT; ti++) {
            auto &obs = assigned[ti];
            if (obs.empty()) continue;
            std::vector<Obs> clean;
            for (auto &o : obs)
                if (!(fl(o.dot) & 4)) clean.push_back(o);          // MFLAG_ELONGATED: two markers merged
            if (clean.size() >= 3 && clean.size() < obs.size()) {
                for (auto &o : obs)
                    if (fl(o.dot) & 4) used[o.dot] = 0;
                obs = clean;
            }
            if ((int)obs.size() < cfg.min_cams_update) {
                for (auto &o : obs) used[o.dot] = 0;
                obs.clear();
            }
        }
        for (int ti = 0; ti < NT; ti++) {
            auto &obs = assigned[ti];
            if (obs.empty()) continue;
            Track &tr = tracks[ti];
            double Tm = 0;
            for (auto &o : obs) Tm += times[o.frame];
            Tm /= obs.size();
            tr.predict(Tm, cfg.q_accel);
            Vec3 X0 = tr.pos(), V = tr.vel();
            std::vector<Vec3> O(obs.size()), D(obs.size());
            std::vector<double> Fp(obs.size());
            for (size_t k = 0; k < obs.size(); k++) {
                const Cam &c = cams[fcam(obs[k].frame)];
                double dt = times[obs[k].frame] - Tm;
                O[k] = {c.pos[0] - V[0] * dt, c.pos[1] - V[1] * dt, c.pos[2] - V[2] * dt};
                D[k] = rays[obs[k].dot];
                Fp[k] = c.f;
            }
            Vec3 X;
            double cov[9];
            std::vector<char> keep;
            bool ok = triangulate(O, D, Fp, max_sigma, cfg.max_resid_px, cfg.min_cams_update, &X0, 3, X, cov, keep);
            if (!ok) {
                for (auto &o : obs) used[o.dot] = 0;
                continue;
            }
            // Kalman update with outlier gating
            double S[9], Si[9];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) S[i * 3 + j] = tr.P[i * 6 + j] + cov[i * 3 + j] + (i == j ? 1e-8 : 0.0);
            if (!inv3(S, Si)) {
                for (auto &o : obs) used[o.dot] = 0;
                continue;
            }
            double y[3] = {X[0] - tr.x[0], X[1] - tr.x[1], X[2] - tr.x[2]};
            double d2 = 0;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) d2 += y[i] * Si[i * 3 + j] * y[j];
            if (tr.confirmed && d2 > cfg.innov_gate && tr.rejects < cfg.innov_max_rejects) {
                tr.rejects++;                       // an outlier: coast this epoch but keep the dots
                continue;
            }
            tr.rejects = 0;
            // The triangulated point only screened the epoch; the filter is updated with each
            // camera's line of sight at that camera's own exposure time (see engine.py
            // _bearing_update): no velocity guess is folded into a measurement.
            {
                std::vector<Vec3> Ok, Dk;
                std::vector<double> dk;
                for (size_t k = 0; k < obs.size(); k++)
                    if (keep[k]) {
                        Ok.push_back(cams[fcam(obs[k].frame)].pos);
                        Dk.push_back(D[k]);
                        dk.push_back(times[obs[k].frame] - Tm);
                    }
                bearing_update(tr, Ok, Dk, dk, max_sigma);
            }
            tr.hits++;
            tr.last_update = T;
            tr.updated_epoch = epochs;
            int nc = 0;
            for (char kk : keep) nc += kk;
            tr.n_cams = nc;
            for (size_t k = 0; k < obs.size(); k++)
                if (keep[k]) tr.add_brightness(times[obs[k].frame], cams[fcam(obs[k].frame)].station, sm(obs[k].dot));
        }

        // ---------------- fixed reflections among the dots left over
        std::vector<char> stat(N, 0);
        for (int fi = 0; fi < F; fi++) {
            std::vector<std::array<double, 2>> pts;
            std::vector<int> ids;
            for (int di : fdots[fi])
                if (!used[di]) { pts.push_back({xy(di, 0), xy(di, 1)}); ids.push_back(di); }
            if (pts.empty()) continue;
            auto s = masks[fcam(fi)].classify(pts, times[fi]);
            for (size_t a = 0; a < ids.size(); a++) stat[ids[a]] = s[a];
        }

        start_tracks(F, fcam, times, fdots, rays, used, stat, xy, sm, T);
        life_cycle(T);

        // ---------------- IDs and output
        std::vector<int> o_tid, o_mid, o_n;
        std::vector<double> o_t, o_pos, o_vel, o_cov;
        {   // blink codes: at most decodes_per_epoch per slot, the longest-waiting first
            std::vector<std::pair<std::pair<double, int>, size_t>> due;
            for (size_t i = 0; i < tracks.size(); i++)
                if (tracks[i].confirmed && T >= tracks[i].next_decode) due.push_back({{tracks[i].next_decode, tracks[i].tid}, i});
            std::sort(due.begin(), due.end());
            for (size_t k = 0; k < due.size() && (int)k < cfg.decodes_per_epoch; k++) decode(due[k].second, T);
        }
        for (size_t i = 0; i < tracks.size(); i++) {
            Track &tr = tracks[i];
            if (tr.confirmed && tr.marker_id >= 0 && tr.updated_epoch == epochs) {
                o_tid.push_back(tr.tid); o_mid.push_back(tr.marker_id); o_n.push_back(tr.n_cams);
                o_t.push_back(tr.t);
                for (int k = 0; k < 3; k++) { o_pos.push_back(tr.x[k]); o_vel.push_back(tr.x[3 + k]); }
                for (int a = 0; a < 3; a++)
                    for (int b = 0; b < 3; b++) o_cov.push_back(tr.P[a * 6 + b]);
            }
        }
        py::dict out;
        size_t no = o_tid.size();
        out["track"] = py::array_t<int>(no, o_tid.data());
        out["marker_id"] = py::array_t<int>(no, o_mid.data());
        out["n_cams"] = py::array_t<int>(no, o_n.data());
        out["t"] = py::array_t<double>(no, o_t.data());
        out["pos"] = py::array_t<double>({no, (size_t)3}, o_pos.data());
        out["vel"] = py::array_t<double>({no, (size_t)3}, o_vel.data());
        out["cov"] = py::array_t<double>({no, (size_t)3, (size_t)3}, o_cov.data());
        return out;
    }

  private:
    template <class FC, class XY, class SM>
    void start_tracks(int F, const FC &fcam, const std::vector<double> &times, const std::vector<std::vector<int>> &fdots,
                      const std::vector<Vec3> &rays, std::vector<char> &used, const std::vector<char> &stat,
                      const XY &xy, const SM &sm, double T) {
        std::vector<std::vector<int>> freed(F);
        for (int fi = 0; fi < F; fi++)
            for (int di : fdots[fi])
                if (!used[di] && !stat[di]) freed[fi].push_back(di);
        struct Cnd { Vec3 X; Obs a, b; double gap; };
        std::vector<Cnd> cands;
        for (int a = 0; a < F; a++)
            for (int b = a + 1; b < F; b++) {
                if (freed[a].empty() || freed[b].empty() || cams[fcam(a)].station == cams[fcam(b)].station) continue;
                const Vec3 &oa = cams[fcam(a)].pos, &ob = cams[fcam(b)].pos;
                for (int i : freed[a])
                    for (int j : freed[b]) {
                        const Vec3 &d1 = rays[i], &d2 = rays[j];
                        Vec3 w0 = sub(oa, ob);
                        double bb = dot(d1, d2), dd = dot(d1, w0), e = dot(d2, w0);
                        double den = std::max(1 - bb * bb, 1e-12);
                        double s = (bb * e - dd) / den, t = (e - bb * dd) / den;
                        Vec3 p1{oa[0] + s * d1[0], oa[1] + s * d1[1], oa[2] + s * d1[2]};
                        Vec3 p2{ob[0] + t * d2[0], ob[1] + t * d2[1], ob[2] + t * d2[2]};
                        Vec3 X{(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2, (p1[2] + p2[2]) / 2};
                        double gap = norm(sub(p1, p2));
                        if (!(gap < cfg.init_gap_m + 0.003 * std::max(s, t) && s > 0.3 && t > 0.3)) continue;
                        if (X[0] < lo[0] || X[1] < lo[1] || X[2] < lo[2] || X[0] > hi[0] || X[1] > hi[1] || X[2] > hi[2])
                            continue;
                        cands.push_back({X, {a, i}, {b, j}, gap});
                    }
            }
        if (cands.empty()) return;
        struct Scored { int neg; double gap; Vec3 X; std::vector<Obs> support; };
        std::vector<Scored> scored;
        for (auto &c : cands) {
            std::vector<Obs> support{c.a, c.b};
            for (int fi = 0; fi < F; fi++) {
                if (fi == c.a.frame || fi == c.b.frame || freed[fi].empty()) continue;
                double u, v, z;
                cams[fcam(fi)].project(c.X, u, v, z);
                if (z <= 0.3) continue;
                double best = INF;
                int bj = -1;
                for (int di : freed[fi]) {
                    double d = std::hypot(xy(di, 0) - u, xy(di, 1) - v);
                    if (d < best) { best = d; bj = di; }
                }
                if (best < cfg.init_support_px) support.push_back({fi, bj});
            }
            std::vector<int> sts;
            for (auto &o : support) {
                int s = cams[fcam(o.frame)].station;
                if (std::find(sts.begin(), sts.end(), s) == sts.end()) sts.push_back(s);
            }
            if ((int)sts.size() >= cfg.min_cams_init) scored.push_back({-(int)support.size(), c.gap, c.X, support});
        }
        std::stable_sort(scored.begin(), scored.end(), [](const Scored &a, const Scored &b) {
            return std::tie(a.neg, a.gap) < std::tie(b.neg, b.gap);
        });
        std::vector<char> taken(used.size(), 0);
        for (auto &s : scored) {
            bool clash = false;
            for (auto &o : s.support)
                if (taken[o.dot]) { clash = true; break; }
            if (clash) continue;
            std::vector<Vec3> O, D;
            std::vector<double> Fp;
            double sig = 0;
            for (auto &o : s.support) {
                const Cam &c = cams[fcam(o.frame)];
                O.push_back(c.pos); D.push_back(rays[o.dot]); Fp.push_back(c.f);
                sig = std::max(sig, c.sigma_ang);
            }
            Vec3 X;
            double cov[9];
            std::vector<char> keep;
            if (!triangulate(O, D, Fp, sig, cfg.max_resid_px * 2, cfg.min_cams_init, nullptr, 1 << 20, X, cov, keep))
                continue;
            for (auto &o : s.support) taken[o.dot] = 1;
            Track tr;
            tr.tid = ++n_tracks;
            for (int i = 0; i < 3; i++) { tr.x[i] = X[i]; tr.x[3 + i] = 0; }
            std::fill(tr.P, tr.P + 36, 0.0);
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 3; b++) tr.P[a * 6 + b] = cov[a * 3 + b] + (a == b ? 1e-6 : 0.0);
            for (int a = 3; a < 6; a++) tr.P[a * 6 + a] = 9.0;
            tr.t = T; tr.born = T; tr.last_update = T; tr.next_decode = T + 1.0;
            int nc = 0;
            for (char k : keep) nc += k;
            tr.n_cams = nc;
            int gone = inherit(tr, T);
            if (gone >= 0) tracks.erase(tracks.begin() + gone);
            for (size_t k = 0; k < s.support.size(); k++) {
                used[s.support[k].dot] = 1;
                if (keep[k])
                    tr.add_brightness(times[s.support[k].frame], cams[fcam(s.support[k].frame)].station, sm(s.support[k].dot));
            }
            tracks.push_back(std::move(tr));
            tracks_started++;
        }
    }

    // a new track where an identified marker was just lost takes over its ID
    // returns the index (in tracks) of a coasting track that handed its ID over, to be removed by the caller
    int inherit(Track &tr, double T) {
        double best_d = INF;
        int best_mid = -1, best_track = -1;
        const Votes *best_votes = nullptr;
        Vec3 p = tr.pos();
        for (size_t i = 0; i < tracks.size(); i++) {
            const Track &o = tracks[i];
            if (&o == &tr || o.marker_id < 0 || !(o.updated_epoch < epochs - 2)) continue;
            double dt = T - o.t;
            Vec3 q{o.x[0] + o.x[3] * dt - p[0], o.x[1] + o.x[4] * dt - p[1], o.x[2] + o.x[5] * dt - p[2]};
            double d = norm(q);
            if (d < cfg.inherit_radius_m + 4.0 * dt && d < best_d) {
                best_d = d; best_mid = o.marker_id; best_track = (int)i; best_votes = &o.votes;
            }
        }
        int best_lost = -1;
        for (size_t i = 0; i < lost.size(); i++) {
            const Lost &l = lost[i];
            double dt = T - l.t;
            Vec3 q{l.pos[0] + l.vel[0] * dt - p[0], l.pos[1] + l.vel[1] * dt - p[1], l.pos[2] + l.vel[2] * dt - p[2]};
            double d = norm(q);
            if (d < cfg.inherit_radius_m + 4.0 * dt && d < best_d) {
                best_d = d; best_mid = l.mid; best_track = -1; best_votes = &l.votes; best_lost = (int)i;
            }
        }
        if (best_mid < 0) return -1;
        for (size_t i = 0; i < tracks.size(); i++)
            if (&tracks[i] != &tr && tracks[i].marker_id == best_mid && (int)i != best_track) return -1;
        Votes votes = *best_votes;
        (void)best_lost;
        lost.erase(std::remove_if(lost.begin(), lost.end(), [&](const Lost &l) { return l.mid == best_mid; }), lost.end());
        tr.marker_id = best_mid;
        tr.votes = votes;
        tr.hits = std::max(tr.hits, cfg.confirm_hits - 2);
        return best_track;
    }

    void life_cycle(double T) {
        std::vector<Track> alive;
        alive.reserve(tracks.size());
        for (auto &tr : tracks) {
            if (!tr.confirmed) {
                if (tr.hits >= cfg.confirm_hits) tr.confirmed = true;
                else if (tr.epochs > cfg.confirm_window) continue;
            }
            if (T - tr.last_update > cfg.coast_s) {
                if (tr.marker_id >= 0) lost.push_back({tr.last_update, tr.marker_id, tr.pos(), tr.vel(), tr.votes});
                continue;
            }
            alive.push_back(std::move(tr));
        }
        tracks = std::move(alive);
        lost.erase(std::remove_if(lost.begin(), lost.end(), [&](const Lost &l) { return !(T - l.t < cfg.inherit_within_s); }),
                   lost.end());
        // unnamed tracks updated now, where a named one was just lost (by track id: inherit may erase)
        std::vector<int> ids;
        for (auto &tr : tracks)
            if (tr.marker_id < 0 && tr.updated_epoch == epochs) ids.push_back(tr.tid);
        for (int id : ids) {
            for (size_t i = 0; i < tracks.size(); i++)
                if (tracks[i].tid == id) {
                    if (tracks[i].marker_id < 0) {
                        int gone = inherit(tracks[i], T);
                        if (gone >= 0) tracks.erase(tracks.begin() + gone);
                    }
                    break;
                }
        }
        // duplicates on one marker: keep the older track
        std::vector<int> drop;
        for (size_t i = 0; i < tracks.size(); i++)
            for (size_t j = i + 1; j < tracks.size(); j++)
                if (norm(sub(tracks[i].pos(), tracks[j].pos())) < 0.02) drop.push_back(std::max(tracks[i].tid, tracks[j].tid));
        if (!drop.empty())
            tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
                                        [&](const Track &t) { return std::find(drop.begin(), drop.end(), t.tid) != drop.end(); }),
                         tracks.end());
    }

    void decode(size_t i, double T) {
        Track &tr = tracks[i];
        tr.next_decode = T + (tr.marker_id >= 0 ? cfg.decode_every_named_s : cfg.decode_every_s);
        std::vector<double> t, lv;
        tr.levels(T - cfg.decode_window_s, t, lv);
        auto r = decode_blink(t, lv);
        if (r.first < 0) return;
        tr.votes.add(r.first, r.second);
        int best = tr.votes.best();
        if (tr.votes.get(best) >= cfg.id_votes_needed && best != tr.marker_id) {
            for (auto &o : tracks) {
                if (&o != &tr && o.marker_id == best) {
                    if (o.votes.get(best) >= tr.votes.get(best)) return;
                    o.marker_id = -1;
                }
            }
            tr.marker_id = best;
        }
    }

  public:
    py::list track_states() const {
        py::list out;
        for (auto &tr : tracks)
            out.append(py::make_tuple(tr.tid, tr.marker_id, tr.confirmed, py::make_tuple(tr.x[0], tr.x[1], tr.x[2]),
                                      tr.n_cams));
        return out;
    }
    size_t n_static_spots() const {
        size_t n = 0;
        for (auto &m : masks) n += m.x.size();
        return n;
    }
};

}  // namespace idps

PYBIND11_MODULE(_core, m) {
    m.doc() = "IDPS positioning engine core (C++ port of idps/engine/engine.py)";
    py::class_<idps::Core>(m, "Core")
        .def(py::init<py::array_t<int>, py::array_t<double>, py::array_t<double>, py::array_t<double>,
                      py::array_t<double>, py::array_t<double>, py::array_t<double>, py::array_t<int>,
                      py::array_t<double>, py::array_t<double>, py::dict>())
        .def("process", &idps::Core::process)
        .def("track_states", &idps::Core::track_states)
        .def("n_static_spots", &idps::Core::n_static_spots)
        .def_readonly("tracks_started", &idps::Core::tracks_started)
        .def_readonly("epochs", &idps::Core::epochs);
    m.def("decode_blink", [](std::vector<double> t, std::vector<double> lv) { return idps::decode_blink(t, lv); });
}
