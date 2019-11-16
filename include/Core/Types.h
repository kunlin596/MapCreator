#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>
#include <memory>

namespace MapCreator {

/**
 * @brief Quaternion (t; x, y, z)
 */
template <typename T = float, std::enable_if_t<std::is_floating_point<T>::value, int> = 0>
using Quaternion_ = typename cv::Vec<T, 4>;
using Quaternion = Quaternion_<float>;

}  // namespace MapCreator
