/*
 * The camera: libcamera raw stream (no ISP processing), fixed exposure, gain and frame rate.
 * Built only when libcamera is found (IDPS_HAVE_LIBCAMERA).
 */
#pragma once

#include <memory>

#include "frame.hpp"

namespace idps {

std::unique_ptr<Source> make_libcamera_source(const SourceConfig &c);
bool libcamera_available();

}  // namespace idps
