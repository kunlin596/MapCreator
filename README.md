# MapCreator

## Overview
[![Project Status: Active – The project has reached an usable state and is being actively developed.](http://www.repostatus.org/badges/latest/active.svg)](http://www.repostatus.org/#active)
[![Build Status](https://travis-ci.org/kunlin596/MapCreator.svg?branch=master)](https://travis-ci.org/kunlin596/MapCreator)

VisualSLAM experiment toolset.

Now it can only read saved Xtion frames (with or without calibration infos) and run off-line map creation.

The pipeline works like this. At first, it matches the keypoints in two frames, and find matched keypoints in the 3d point clouds such that we can have the 3d matching keypoints sets. Secondly, we compute the trasformation matrix between the two 3d keypoint sets. Lastly, it stitches the 3d point clouds together using this tranformation matrix. And we perform the this computation every two frames build up a 3d point cloud map.

## Dependencies

* [OpenCV](https://opencv.org)
* [GLM](https://glm.g-truc.net/0.9.9/index.html)
* [Qt 5](https://www.qt.io)
* [Boost](https://www.boost.org)
* [Engen3](https://eigen.tuxfamily.org/dox)
* OpenGL 4.x

### macOS
```
port selfupdate
port upgrade outdated
port install pkgconfig glm boost qt59 opencv log4cxx
```

## Installation

Please make sure that Qt5's `moc` and `rcc` are in `PATH`, otherwise, the Qt objects won't compile.

```
mkdir build && cd build
cmake ..
make install
```
