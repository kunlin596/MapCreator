#pragma once

#include "Core/Pose.h"

namespace MapCreator {

template <typename T>
struct Frame_ {
  Pose_<T> pose;
  float width;
  float height;
  float horizontal_fov;
  float vertical_fov;

  Frame_::Frame_()
      : width(640),
        height(480),
        horizontal_fov(1.0225999f),
        vertical_fov(0.79661566f) {}
};

using Frame = Frame_<float>;

template <typename T>
struct RGBDFrame_ : Frame_<T> {
  ColorImage colorimage;
  DepthImage depthimage;

  RGBDFrame_::RGBDFrame_() : Frame_() {}
};

using RGBDFrame = RGBDFrame_<float>;
}  // namespace MapCreator