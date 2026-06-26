//
// Created by LinKun on 9/12/15.
//

#ifndef MAPCREATOR_UTILITY_H
#define MAPCREATOR_UTILITY_H

#include <algorithm>
#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>
#include <string>

namespace MapCreator {

using namespace std;

template <typename T>
inline bool OutOfRange(T value, T range_min, T range_max) {
  return ((value < range_min) || (range_max < value));
}

template <typename T>
inline T LimitRange(T value, T range_min, T range_max) {
  value = (std::max)(value, range_min);
  value = (std::min)(value, range_max);
  return value;
}

std::string GetString(const char* c_str, size_t len);

template <typename From, typename To>
To ConvertMatrix(const From& m);

// Convert an OpenCV 4x4 (row-vector convention, p' = p * M) to a glm matrix
// (column-vector convention, p' = M * p): glm = transpose(cv).
inline glm::mat4 ConvertCVMatx44fToGLMmat4(const cv::Matx44f& m) {
  glm::mat4 result(1.0f);
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) result[i][j] = m(i, j);
  return result;
}

}  // namespace MapCreator

#endif  // MAPCREATOR_UTILITY_H
