"""Builds the optional C++ engine core (idps.engine._core). Everything else is plain Python
(pyproject.toml). If no C++ compiler is available the build skips the core and the engine
runs its Python implementation."""
from setuptools import setup

try:
    from pybind11.setup_helpers import Pybind11Extension, build_ext

    core = Pybind11Extension("idps.engine._core", ["idps/engine/_core.cpp"], cxx_std=17,
                             extra_compile_args=["-O3"])
    core.optional = True
    setup(ext_modules=[core], cmdclass={"build_ext": build_ext})
except ImportError:                       # pybind11 missing: Python engine only
    setup()
