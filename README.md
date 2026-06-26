# MapCreator

## Overview
[![Project Status: Active – The project has reached a usable state and is being actively developed.](http://www.repostatus.org/badges/latest/active.svg)](http://www.repostatus.org/#active)

A visual SLAM experiment toolset for **offline RGB-D map creation**, and the
reference implementation of the ViEW2014 paper below.

It reads saved Asus Xtion frames (with or without calibration data) and builds a 3D
map offline. The pipeline:

1. Match 2D keypoints between two frames and look them up in the 3D point clouds to
   form 3D correspondence sets.
2. Estimate the rigid transformation between the two 3D keypoint sets (SVD,
   refined with Levenberg–Marquardt).
3. Stitch the point clouds together with that transformation.

Repeating this over consecutive frames accumulates the map.

> **Status.** The codebase was recently refactored (`SLAM/` → `Core/`, an immutable
> `KeyFrame`, a `CoordinateConverter` hierarchy, and a modern target-based CMake
> build). The core SLAM library and data handler build and are unit-tested; the Qt
> GUI compiles but is not yet runtime-verified, and a few paths (the calibrated
> converter and polymorphic tracker parameters) are stubbed — see the inline code
> comments / TODOs.

## Reference

This repository implements:

> Kun Lin and Yutaka Satoh, **"A Study on Automatic Selection of Key-frames of
> RGB-D SLAM,"** *ViEW2014 — Vision Engineering Workshop*, 2014.

The paper reduces accumulated error in RGB-D reconstruction by measuring the
distribution of the 3D points backing the matched 2D image features and using the
**first principal-component ratio** to decide when to insert a new keyframe — with
significant gains in reconstruction accuracy and stability over all-frame and
fixed-interval baselines. In this codebase that criterion is the `KeyFrameOnly`
tracking mode (see `ValidateInliersDistribution` and `KeyFrameOnlyTrackerParameters`
in `Core`); `Consecutive` (all frames) and `FixedNumber` (fixed interval) are the
baselines it is compared against.

See <https://kunlin.vision/publications/>.

## Dependencies

- [OpenCV](https://opencv.org) 4.x — contrib `xfeatures2d` only needed for SIFT/SURF/FREAK
- [Eigen3](https://eigen.tuxfamily.org)
- [GLM](https://glm.g-truc.net)
- [Qt 5](https://www.qt.io) (Core, Widgets, OpenGL, Concurrent) + OpenGL 4.x
- [Boost](https://www.boost.org) (system, filesystem, serialization, iostreams, program_options)
- [Apache log4cxx](https://logging.apache.org/log4cxx/)
- [GoogleTest](https://github.com/google/googletest) — fetched automatically for the tests
- CMake ≥ 3.16 and a C++17 compiler

### Debian / Ubuntu (22.04+)
```bash
sudo apt update
sudo apt install -y \
  cmake build-essential pkg-config \
  libeigen3-dev libopencv-dev libglm-dev liblog4cxx-dev libboost-all-dev \
  qtbase5-dev libqt5opengl5-dev
```
The default build links the system OpenCV and uses ORB features. Set
`-DENABLE_OPENCV_CONTRIB=ON` only if you have an OpenCV built with the non-free
`xfeatures2d` module (SIFT/SURF/FREAK).

### macOS (MacPorts)
```bash
sudo port install cmake pkgconfig glm boost qt5 opencv eigen3 log4cxx
```

## Build
```bash
cmake -S . -B build
cmake --build build -j
```
Executables are written to `build/bin`, libraries to `build/lib`.

### Options
| Option | Default | Description |
| --- | --- | --- |
| `MAPCREATOR_BUILD_APP` | `ON` | Build the Qt GUI (`MapCreatorGui`) and executables |
| `MAPCREATOR_BUILD_TOOLS` | `ON` | Build the tool executables |
| `MAPCREATOR_BUILD_TESTS` | `ON` | Build the GoogleTest unit tests |
| `ENABLE_OPENCV_CONTRIB` | `OFF` | Use OpenCV contrib `xfeatures2d` (SIFT/SURF/FREAK) |

Library-only / headless build (no Qt GUI):
```bash
cmake -S . -B build -DMAPCREATOR_BUILD_APP=OFF
cmake --build build -j
```

## Tests
```bash
ctest --test-dir build --output-on-failure
```

## Project layout
| Path | Target | Purpose |
| --- | --- | --- |
| `src/Core`, `include/Core` | `MapCreatorCore` | SLAM core: features, matching, point clouds, tracking, optimization |
| `src/Streamer`, `include/Handler` | `MapCreatorHandler` | Reads Xtion frames and converts them to keyframes |
| `src/Applications`, `include/BasicViewer`, `include/Engine` | `MapCreatorGui` | Qt OpenGL viewers and dialogs |
| `src/Tools` | `MapCreator`, `FrameViewer`, `dataset_converter`, `Viewer` | Executables |
| `tests` | — | GoogleTest unit tests |

## Development

This repository uses [pre-commit](https://pre-commit.com) for hygiene checks and
[Conventional Commits](https://www.conventionalcommits.org) (with a **required
scope**, e.g. `fix(core): ...`):
```bash
pre-commit install && pre-commit install --hook-type commit-msg
```
