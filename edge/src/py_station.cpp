/*
 * idps.station._station: the camera station's C++ detector and packet encoder, callable from
 * Python for tests and for the end-to-end check (images -> dots -> packets -> engine).
 */
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "detect.hpp"
#include "packet.hpp"
#include "unpack.hpp"

namespace py = pybind11;
using namespace idps;

static DetectParams params_from(const py::dict &d) {
    DetectParams p;
    for (auto item : d) {
        const std::string k = py::str(item.first);
        py::handle v = item.second;
        if (k == "threshold") p.threshold = v.cast<int>();
        else if (k == "threshold_low") p.threshold_low = v.cast<int>();
        else if (k == "seed_step") p.seed_step = v.cast<int>();
        else if (k == "min_area") p.min_area = v.cast<int>();
        else if (k == "max_area") p.max_area = v.cast<int>();
        else if (k == "saturation") p.saturation = v.cast<int>();
        else if (k == "elongation") p.elongation = v.cast<double>();
        else if (k == "elongation_min_area") p.elongation_min_area = v.cast<int>();
        else if (k == "max_dots") p.max_dots = v.cast<int>();
        else throw py::key_error("unknown detector parameter: " + k);
    }
    return p;
}

static py::list to_list(const std::vector<Dot> &dots) {
    py::list out;
    for (const Dot &d : dots) out.append(py::make_tuple(d.x, d.y, d.sum, d.peak, d.area, d.flags));
    return out;
}

template <typename T>
static py::list detect_t(Detector &det, py::array_t<T, py::array::c_style> img) {
    if (img.ndim() != 2) throw std::invalid_argument("image must be 2-D (height, width)");
    const int H = (int)img.shape(0), W = (int)img.shape(1);
    py::gil_scoped_release nogil;
    const auto &dots = det.detect(img.data(), W, H, W);
    py::gil_scoped_acquire gil;
    return to_list(dots);
}

static std::vector<Dot> dots_from(const py::list &l) {
    std::vector<Dot> dots;
    for (auto o : l) {
        auto t = o.cast<py::tuple>();
        dots.push_back({t[0].cast<double>(), t[1].cast<double>(), t[2].cast<double>(), t[3].cast<int>(),
                        t[4].cast<int>(), t[5].cast<int>()});
    }
    return dots;
}

PYBIND11_MODULE(_station, m) {
    m.doc() = "Camera station C++ code (edge/src): dot detector and packet encoder";
    py::class_<Detector>(m, "Detector")
        .def(py::init([](py::dict p) { return Detector(params_from(p)); }), py::arg("params") = py::dict())
        .def("set_params", [](Detector &d, py::dict p) { d.set_params(params_from(p)); })
        .def("detect", &detect_t<uint8_t>, "dots of an 8-bit frame: [(x, y, sum, peak, area, flags)]")
        .def("detect", &detect_t<uint16_t>, "dots of a 16-bit frame");
    m.def("encode_frame",
          [](int station_id, uint64_t frame_id, int64_t t_mid_ns, uint32_t exposure_ns, py::list dots, uint32_t seq,
             uint32_t proc_us, int sync_state, int threshold, uint32_t drops, int flags) {
              FrameInfo f;
              f.station_id = (uint16_t)station_id;
              f.frame_id = frame_id;
              f.t_mid_ns = t_mid_ns;
              f.exposure_ns = exposure_ns;
              f.proc_us = proc_us;
              f.sync_state = (uint8_t)sync_state;
              f.threshold = (uint16_t)threshold;
              f.drops_since_last = drops;
              f.flags = (uint16_t)flags;
              py::list out;
              for (auto &p : encode_frame(f, dots_from(dots), seq)) out.append(py::bytes(p));
              return out;
          },
          py::arg("station_id"), py::arg("frame_id"), py::arg("t_mid_ns"), py::arg("exposure_ns"), py::arg("dots"),
          py::arg("seq"), py::arg("proc_us") = 0, py::arg("sync_state") = 0, py::arg("threshold") = 0,
          py::arg("drops") = 0, py::arg("flags") = 0);
    m.def("encode_health", [](int station_id, uint32_t seq, const std::string &json, int flags) {
        return py::bytes(encode_health((uint16_t)station_id, seq, json, (uint16_t)flags));
    }, py::arg("station_id"), py::arg("seq"), py::arg("json"), py::arg("flags") = 0);
    m.def("unpack_raw10", [](py::bytes packed, int width, int height, int stride) {
        std::string s = packed;
        py::array_t<uint16_t> out({height, width});
        unpack_raw10_csi2((const uint8_t *)s.data(), width, height, stride, out.mutable_data());
        return out;
    }, "MIPI CSI-2 packed RAW10 (4 pixels in 5 bytes) to 16-bit pixels");
}
