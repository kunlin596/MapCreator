#pragma once

#include <memory>
#include <opencv2/core/matx.hpp>
#include <opencv2/opencv.hpp>

namespace MapCreator {

/**
 * @brief Quaternion (t; x, y, z)
 */
template <typename T>
using Quaternion_ = cv::Vec<T, 4>;
using Quaternion = Quaternion_<float>;

}  // namespace MapCreator
