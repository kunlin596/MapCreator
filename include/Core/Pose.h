#pragma once
#include "MyMath.h"
#include "Types.h"

namespace MapCreator {

template <typename T>
struct Pose_ {
  Quaternion_<T> quaternion;
  cv::Vec<T, 3> translation;

  cv::Matx33f GetRotationMatrix() {
    return CreateRotationMatrix(quaternion);
  }
};

using Pose = Pose_<float>;

}  // namespace MapCreator