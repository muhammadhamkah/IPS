"""Builds the optional C++ parts: the engine core (idps.engine._core) and the camera station's
detector and packet encoder for Python (idps.station._station, from edge/src). Everything else
is plain Python (pyproject.toml). If no C++ compiler is available the build skips them: the
engine runs its Python implementation, and the station tests that need the C++ code are skipped."""
from setuptools import setup

try:
    from pybind11.setup_helpers import Pybind11Extension, build_ext

    core = Pybind11Extension("idps.engine._core", ["idps/engine/_core.cpp"], cxx_std=17,
                             extra_compile_args=["-O3"])
    core.optional = True
    station = Pybind11Extension("idps.station._station", ["edge/src/py_station.cpp"], cxx_std=17,
                                extra_compile_args=["-O3"])
    station.optional = True
    setup(ext_modules=[core, station], cmdclass={"build_ext": build_ext})
except ImportError:                       # pybind11 missing: Python engine only
    setup()
