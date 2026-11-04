/*
 * The camera through libcamera (docs/ARCHITECTURE.md section 4.2, Capture):
 *  * a Raw stream: the sensor's pixels as read out, no ISP processing;
 *  * auto exposure off; fixed exposure, gain and frame duration;
 *  * 8-bit raw (R8) by default; 10-bit raw is unpacked to 16-bit pixels.
 * Each frame carries the sensor frame counter and libcamera's SensorTimestamp (CLOCK_MONOTONIC,
 * ns; the bench checks that against the clock). Buffers go back to the camera when the station
 * has detected the frame's dots.
 */
#include "source_libcamera.hpp"

#ifdef IDPS_HAVE_LIBCAMERA

#include <sys/mman.h>

#include <atomic>
#include <cstdio>
#include <map>
#include <mutex>
#include <vector>

#include <libcamera/libcamera.h>

#include "clock.hpp"
#include "unpack.hpp"

namespace idps {

using namespace libcamera;

namespace {

// 16-bit raw (the sensor's bits at the top), by its DRM fourcc: older libcamera has no name for it
const PixelFormat kR16('R' | ('1' << 8) | ('6' << 16) | ((uint32_t)' ' << 24));

class LibcameraSource : public Source {
  public:
    explicit LibcameraSource(const SourceConfig &c) : c_(c) {}
    ~LibcameraSource() override { stop(); }

    bool start(std::function<void(Frame &&)> on_frame) override {
        on_frame_ = std::move(on_frame);
        cm_ = std::make_unique<CameraManager>();
        if (cm_->start()) return fail("libcamera: camera manager did not start");
        auto cams = cm_->cameras();
        if (cams.empty()) return fail("no camera found (is the ribbon cable seated, and dtoverlay=ov9281 in /boot/firmware/config.txt?)");
        if (c_.camera_index >= (int)cams.size()) return fail("camera index out of range");
        cam_ = cams[c_.camera_index];
        if (cam_->acquire()) return fail("camera is in use by another program");
        acquired_ = true;
        model_ = cam_->id();

        cfg_ = cam_->generateConfiguration({StreamRole::Raw});
        if (!cfg_ || cfg_->empty()) return fail("camera offers no raw stream");
        StreamConfiguration &sc = cfg_->at(0);
        sc.size = Size(c_.width, c_.height);
        sc.bufferCount = 8;
        // prefer 8-bit raw (half the bytes, no unpacking); else 10-bit, packed or in 16 bits
        const std::vector<PixelFormat> want = c_.bits > 8
            ? std::vector<PixelFormat>{formats::R10_CSI2P, formats::R10, kR16, formats::R8}
            : std::vector<PixelFormat>{formats::R8, formats::R10_CSI2P, formats::R10, kR16};
        bool ok = false;
        for (const PixelFormat &pf : want) {
            sc.pixelFormat = pf;
            sc.size = Size(c_.width, c_.height);
            if (cfg_->validate() != CameraConfiguration::Invalid && sc.pixelFormat == pf) { ok = true; break; }
        }
        if (!ok) return fail("camera offers none of R8, R10, R16: got " + sc.pixelFormat.toString());
        if (cam_->configure(cfg_.get())) return fail("camera configuration was refused");
        stream_ = sc.stream();
        fmt_ = sc.pixelFormat;
        width_ = (int)sc.size.width;
        height_ = (int)sc.size.height;
        stride_bytes_ = (int)sc.stride;

        alloc_ = std::make_unique<FrameBufferAllocator>(cam_);
        if (alloc_->allocate(stream_) < 0) return fail("could not allocate camera buffers");
        for (const std::unique_ptr<FrameBuffer> &buf : alloc_->buffers(stream_)) {
            const FrameBuffer::Plane &pl = buf->planes()[0];
            const size_t len = pl.offset + pl.length;
            void *m = mmap(nullptr, len, PROT_READ, MAP_SHARED, pl.fd.get(), 0);
            if (m == MAP_FAILED) return fail("could not map a camera buffer");
            maps_[buf.get()] = {static_cast<uint8_t *>(m), len, pl.offset};
            std::unique_ptr<Request> req = cam_->createRequest();
            if (!req || req->addBuffer(stream_, buf.get())) return fail("could not create a capture request");
            scratch_[req.get()].resize(fmt_ == formats::R8 ? 0 : (size_t)width_ * height_);
            requests_.push_back(std::move(req));
        }

        ControlList ctrls(cam_->controls());
        const int64_t frame_us = (int64_t)(1e6 / c_.fps + 0.5);
        ctrls.set(controls::FrameDurationLimits, Span<const int64_t, 2>({frame_us, frame_us}));
        ctrls.set(controls::ExposureTime, (int32_t)c_.exposure_us);
        ctrls.set(controls::AnalogueGain, (float)c_.gain);
        // auto exposure off; the control's name differs between libcamera versions
        for (const auto &kv : cam_->controls()) {
            const std::string &n = kv.first->name();
            if (n == "AeEnable") ctrls.set(kv.first->id(), ControlValue(false));
            else if (n == "ExposureTimeMode" || n == "AnalogueGainMode") ctrls.set(kv.first->id(), ControlValue((int32_t)1));
        }

        cam_->requestCompleted.connect(this, &LibcameraSource::completed);
        if (cam_->start(&ctrls)) return fail("camera did not start");
        running_ = true;
        for (auto &r : requests_)
            if (cam_->queueRequest(r.get()) < 0) return fail("could not queue a capture request");
        return true;
    }

    void stop() override {
        if (running_) {
            running_ = false;
            cam_->stop();
            cam_->requestCompleted.disconnect(this);
        }
        requests_.clear();
        for (auto &kv : maps_) munmap(kv.second.base, kv.second.len);
        maps_.clear();
        if (alloc_ && stream_) alloc_->free(stream_);
        alloc_.reset();
        if (acquired_) { cam_->release(); acquired_ = false; }
        cam_.reset();
        if (cm_) { cm_->stop(); cm_.reset(); }
    }

    std::string describe() const override {
        char b[256];
        std::snprintf(b, sizeof b, "camera %s, %dx%d %s, %.1f fps, exposure %d us, gain %.2f", model_.c_str(), width_,
                      height_, fmt_.toString().c_str(), c_.fps, c_.exposure_us, c_.gain);
        return b;
    }

  private:
    bool fail(const std::string &why) {
        error_ = why;
        stop();
        return false;
    }

    void completed(Request *r) {
        if (r->status() == Request::RequestCancelled || !running_) return;
        FrameBuffer *buf = r->buffers().begin()->second;
        const FrameMetadata &md = buf->metadata();
        if (md.status != FrameMetadata::FrameSuccess) {
            requeue(r);
            return;
        }
        const auto &mp = maps_.at(buf);
        const uint8_t *px = mp.base + mp.offset;
        Frame f;
        f.width = width_;
        f.height = height_;
        f.sequence = md.sequence;
        f.arrival_mono_ns = mono_ns();
        auto ts = r->metadata().get(controls::SensorTimestamp);
        f.sensor_ns = ts ? *ts : (int64_t)md.timestamp;
        auto ex = r->metadata().get(controls::ExposureTime);
        f.exposure_ns = (int64_t)(ex ? *ex : c_.exposure_us) * 1000;
        if (fmt_ == formats::R8) {
            f.data = px;
            f.stride = stride_bytes_;
            f.bytes_per_pixel = 1;
        } else {
            std::vector<uint16_t> &out = scratch_[r];
            if (fmt_ == formats::R10_CSI2P) {
                unpack_raw10_csi2(px, width_, height_, stride_bytes_, out.data());
            } else {    // R10: 10 bits in 16; R16: the bits at the top
                const int shift = fmt_ == kR16 ? 6 : 0;
                for (int y = 0; y < height_; y++) {
                    const uint16_t *s = reinterpret_cast<const uint16_t *>(px + (size_t)y * stride_bytes_);
                    uint16_t *d = out.data() + (size_t)y * width_;
                    for (int x = 0; x < width_; x++) d[x] = (uint16_t)(s[x] >> shift);
                }
            }
            f.data = reinterpret_cast<const uint8_t *>(out.data());
            f.stride = width_;
            f.bytes_per_pixel = 2;
        }
        f.release = [this, r] { requeue(r); };
        on_frame_(std::move(f));
    }

    void requeue(Request *r) {
        if (!running_) return;
        r->reuse(Request::ReuseBuffers);
        cam_->queueRequest(r);
    }

    struct Map { uint8_t *base; size_t len; size_t offset; };

    SourceConfig c_;
    std::function<void(Frame &&)> on_frame_;
    std::unique_ptr<CameraManager> cm_;
    std::shared_ptr<Camera> cam_;
    std::unique_ptr<CameraConfiguration> cfg_;
    std::unique_ptr<FrameBufferAllocator> alloc_;
    Stream *stream_ = nullptr;
    PixelFormat fmt_;
    std::vector<std::unique_ptr<Request>> requests_;
    std::map<const FrameBuffer *, Map> maps_;
    std::map<const Request *, std::vector<uint16_t>> scratch_;
    std::string model_;
    int width_ = 0, height_ = 0, stride_bytes_ = 0;
    bool acquired_ = false;
    std::atomic<bool> running_{false};
};

}  // namespace

bool libcamera_available() { return true; }

std::unique_ptr<Source> make_libcamera_source(const SourceConfig &c) { return std::make_unique<LibcameraSource>(c); }

}  // namespace idps

#else   // built without libcamera

namespace idps {

namespace {
class NoCamera : public Source {
  public:
    bool start(std::function<void(Frame &&)>) override {
        error_ = "this idps-station was built without libcamera (install libcamera-dev and rebuild)";
        return false;
    }
    void stop() override {}
    std::string describe() const override { return "no camera support"; }
};
}  // namespace

bool libcamera_available() { return false; }
std::unique_ptr<Source> make_libcamera_source(const SourceConfig &) { return std::make_unique<NoCamera>(); }

}  // namespace idps

#endif
