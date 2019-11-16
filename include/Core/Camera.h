#pragma once

#include <opencv2/opencv.hpp>

#include "Frame.h"
#include "Image.h"

namespace MapCreator {

/**
 * @brief      Camera base
 */
class CameraBase : public FrameBase {
public:
  static size_t width;
  static size_t height;
  cv::Matx<float, 3, 4> matrix;
  float horizontalFov;
  float verticalFov;
  CameraBase() {}
  virtual ~CameraBase() {}
};

/**
 * @brief      Captured gray image frame
 */
class GrayFrame : public CameraBase {
public:
  GrayScaleImage<uint8_t> image;
  GrayFrame(const GrayScaleImage<uint8_t>& image) {
    this->image = image;
  }
  virtual ~GrayFrame() {}
};

/**
 * @brief      Captured depth image frame
 */
using DepthFrame = GrayFrame;

/**
 * @brief      Captured RGB image frame
 */
class RGBFrame: public CameraBase {
public:
  ColorImage image;
  RGBFrame(const ColorImage& image) {
    this->image = image;
  }
  virtual ~RGBFrame() {}
};

/**
 * @brief      Captured point image frame
 */
class PointFrame: public CameraBase {
public:
  PointImage image;
  PointFrame(const ColorImage& image) {
    this->image = image;
  }
  virtual ~PointFrame() {}
};

/**
 * @brief      Captured RGB and depth images frame
 */
class RGBDFrame : public CameraBase {
public:
  ColorImage colorimage;
  PointImage pointimage;

  RGBDFrame(const RGBFrame& color_frame, const PointFrame& depth_frame) {
    colorimage = color_frame.image;
    pointimage = depth_frame.image;
  }
  virtual ~RGBDFrame() {}
};

}  // namespace MapCreator
