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
```bash
sudo port selfupdate
sudo port upgrade outdated
sudo port install pkgconfig glm boost qt59 opencv log4cxx
```
### Debian/Ubuntu
```bash
udo apt update
sudo apt install -y pkg-config libboost-all-dev qt5-default qtbase5-dev qtdeclarative5-dev libqt5opengl5-dev libeigen3-dev liblog4cxx-dev

# libglm-dev doesn't provide cmake config, so we need to compile it from source
git clone https://github.com/g-truc/glm.git
cd glm
mkdir build && cd build
cmake ..
sudo make -j4 install
cd ../..

# clone opencv_contrib for building non-free modules
git clone https://github.com/opencv/opencv_contrib.git

# build and install opencv
git clone https://github.com/opencv/opencv.git
cd opencv 
git checkout 4.1.0
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DBUILD_EXAMPLES=OFF -DBUILD_opencv_apps=OFF -DBUILD_DOCS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
sudo make -j8 install
cd ../..
```

## Installation

Please make sure that Qt5's `moc` and `rcc` are in `PATH`, otherwise, the Qt objects won't compile.

```
mkdir build && cd build
cmake ..
make install
```
