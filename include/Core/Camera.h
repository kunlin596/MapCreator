#pragma once

#include <opencv2/opencv.hpp>

#include "Frame.h"
#include "Image.h"

namespace MapCreator {

/**
 * @brief      Camera base
 */
struct CameraBase : public FrameBase<CameraBase> {
  static size_t width;
  static size_t height;
  cv::Matx<float, 3, 4> matrix;
  float horizontalFov;
  float verticalFov;
}

/**
 * @brief      Captured gray image frame
 */
struct GrayFrame : public CameraBase {
  GrayScaleImage<uint8_t> image;
  explicit RGBDFrame(const GrayScaleImage<uint8_t>& image) {
    this->image = image;
  }
};

/**
 * @brief      Captured depth image frame
 */
using DepthFrame = GrayFrame;

/**
 * @brief      Captured RGB image frame
 */
struct RGBFrame: public CameraBase {
  ColorImage image;
  explicit RGBDFrame(const ColorImage& image) {
    this->image = image;
  }
};

/**
 * @brief      Captured RGB and depth images frame
 */
struct RGBDFrame : public CameraBase {
  ColorImage colorimage;
  DepthImage depthimage;

  explicit RGBDFrame(const RGBFrame& color_frame, const DepthFrame& depth_frame) {
    colorimage = color_frame.image;
    dpethimage = dpeth_frame.image;
  }
};

}  // namespace MapCreator
