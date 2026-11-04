# IDPS test marker for the ESP32-C5 (docs/MARKER.md). Build with ESP-IDF 5.5 or later:
#   idf.py set-target esp32c5 && idf.py build flash monitor
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(idps_marker)
