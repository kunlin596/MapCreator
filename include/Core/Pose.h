#pragma once
#include "MyMath.h"
#include "Types.h"

namespace MapCreator {

struct Pose {
  Quaternion_<float> quaternion;
  cv::Vec<float, 3> translation;

  cv::Matx33f GetRotationMatrix() {
    return CreateRotationMatrix(quaternion);
  }
};

}  // namespace MapCreator