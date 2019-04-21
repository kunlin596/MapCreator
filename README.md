# MapCreator

[![Build Status](https://travis-ci.org/kunlin596/MapCreator.svg?branch=master)](https://travis-ci.org/kunlin596/MapCreator)

This is my personal RGBD-SLAM related experimental toolset.

It can read a saved Xtion frames (with or without calibration infos) and run map creation using them.

## Dependencies

* OpenCV https://opencv.org
* GLM (OpenGL Mathematics) https://glm.g-truc.net/0.9.9/index.html
* Qt 5 https://www.qt.io

## Build

```
mkdir build && cd build
cmake ..
make install
```
