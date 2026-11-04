# idps-station: the camera station program (docs/STATION.md). On the Pi:
#   cmake -S edge -B edge/build -DCMAKE_BUILD_TYPE=Release && cmake --build edge/build -j4
# libcamera (libcamera-dev) is optional: without it the station runs on synthetic or file frames only.
cmake_minimum_required(VERSION 3.16)
project(idps_station CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  # the Pi 5's Cortex-A76: lets the compiler vectorise the seed scan with NEON
  add_compile_options(-mcpu=cortex-a76)
endif()
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(Threads REQUIRED)
find_package(PkgConfig)
if(PkgConfig_FOUND)
  pkg_check_modules(LIBCAMERA IMPORTED_TARGET libcamera)
endif()

add_executable(idps-station src/station.cpp src/source_libcamera.cpp)
target_link_libraries(idps-station PRIVATE Threads::Threads)
if(LIBCAMERA_FOUND)
  target_compile_definitions(idps-station PRIVATE IDPS_HAVE_LIBCAMERA)
  target_link_libraries(idps-station PRIVATE PkgConfig::LIBCAMERA)
  message(STATUS "idps-station: camera support (libcamera ${LIBCAMERA_VERSION})")
else()
  message(STATUS "idps-station: no libcamera found, synthetic and file sources only")
endif()

install(TARGETS idps-station RUNTIME DESTINATION bin)
