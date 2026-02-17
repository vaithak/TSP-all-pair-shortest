"""
Setup script for TSP Path Planner package.
"""

from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext
import sys
import os
import subprocess

# Check if pybind11 is available
try:
    from pybind11.setup_helpers import Pybind11Extension, build_ext as pybind11_build_ext
    HAS_PYBIND11 = True
except ImportError:
    HAS_PYBIND11 = False
    print("WARNING: pybind11 not found. C++ TSP solver will not be built.")
    print("Install pybind11 with: pip install pybind11")


# Read version from __init__.py
def get_version():
    with open('tsp_path_planner/__init__.py', 'r') as f:
        for line in f:
            if line.startswith('__version__'):
                return line.split('=')[1].strip().strip('"').strip("'")
    return "1.0.0"


# Read long description from README
def get_long_description():
    if os.path.exists('README.md'):
        with open('README.md', 'r', encoding='utf-8') as f:
            return f.read()
    return ""


# OR-Tools configuration
ORTOOLS_BASE = "tsp_path_planner/or-tools"
ORTOOLS_INCLUDE = os.path.join(ORTOOLS_BASE, "include")
ORTOOLS_LIB = os.path.join(ORTOOLS_BASE, "lib")


# Build C++ extension if pybind11 is available
ext_modules = []

if HAS_PYBIND11 and os.path.exists(ORTOOLS_INCLUDE):
    print("Building C++ TSP solver extension...")
    
    # Determine platform-specific settings
    # Use -isystem so bundled OR-Tools/protobuf headers take priority
    # over any system-installed protobuf headers.
    extra_compile_args = [
        "-std=c++17", "-O3", "-fwrapv",
        f"-isystem{os.path.abspath(ORTOOLS_INCLUDE)}",
        # Definitions required by OR-Tools (from ortoolsTargets.cmake)
        "-DOR_PROTO_DLL=",
        "-DUSE_LP_PARSER",
        "-DUSE_MATH_OPT",
        "-DUSE_BOP",
        "-DUSE_CBC",
        "-DUSE_CLP",
        "-DUSE_GLOP",
        "-DUSE_PDLP",
        "-DUSE_SCIP",
    ]
    extra_link_args = []
    libraries = ["ortools"]
    
    if sys.platform == "darwin":  # macOS
        extra_link_args.append(f"-Wl,-rpath,{os.path.abspath(ORTOOLS_LIB)}")
    elif sys.platform.startswith("linux"):  # Linux
        extra_link_args.append(f"-Wl,-rpath,{os.path.abspath(ORTOOLS_LIB)}")
    
    ext_modules = [
        Pybind11Extension(
            "tsp_path_planner._tsp_solver_cpp",
            sources=[
                "tsp_path_planner/cpp/pybind_wrapper.cpp",
                "tsp_path_planner/cpp/tsp_solver.cpp",
            ],
            include_dirs=[ORTOOLS_INCLUDE],
            library_dirs=[ORTOOLS_LIB],
            libraries=libraries,
            extra_compile_args=extra_compile_args,
            extra_link_args=extra_link_args,
            language="c++",
        ),
    ]
    
    # Use pybind11 build_ext
    cmdclass = {"build_ext": pybind11_build_ext}
else:
    print("Skipping C++ extension build (missing pybind11 or OR-Tools)")
    print("Python fallback TSP solver will be used.")
    cmdclass = {}


# Package setup
setup(
    name="tsp_path_planner",
    version=get_version(),
    author="Your Name",
    author_email="your.email@example.com",
    description="TSP-based path planning with A* shortest paths",
    long_description=get_long_description(),
    long_description_content_type="text/markdown",
    url="https://github.com/vaithak/TSP-all-pair-shortest",
    packages=find_packages(),
    ext_modules=ext_modules,
    cmdclass=cmdclass,
    install_requires=[
        "numpy>=1.20.0",
        "matplotlib>=3.3.0",
        "scipy>=1.7.0",
    ],
    extras_require={
        "dev": [
            "pytest>=6.0",
            "pytest-cov>=2.0",
            "pybind11>=2.10.0",
        ],
    },
    python_requires=">=3.8",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: C++",
    ],
    keywords="tsp path-planning astar robotics navigation optimization",
    project_urls={
        "Bug Reports": "https://github.com/vaithak/TSP-all-pair-shortest/issues",
        "Source": "https://github.com/vaithak/TSP-all-pair-shortest",
    },
)
