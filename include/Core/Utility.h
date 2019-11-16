//
// Created by LinKun on 9/12/15.
//

#ifndef MAPCREATOR_UTILITY_H
#define MAPCREATOR_UTILITY_H

#include <algorithm>
#include <glm/ext.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/type_ptr.hpp>
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

template <>
cv::Matx44f ConvertMatrix<glm::mat4, cv::Matx44f>(const glm::mat4& m) {
  return cv::Matx44f(glm::value_ptr(m));
}

template <>
glm::mat4 ConvertMatrix<cv::Matx44f, glm::mat4>(const cv::Matx44f& m) {
  return glm::make_mat4(m.val);
}

inline std::ostream& operator<<(std::ostream& os, const glm::vec4& v) {
  os << v.x << " " << v.y << " " << v.z << " " << v.w << std::endl;
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const glm::vec3& v) {
  os << v.x << " " << v.y << " " << v.z << std::endl;
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const glm::mat4& m) {
  os << glm::row(m, 0);
  os << glm::row(m, 1);
  os << glm::row(m, 2);
  os << glm::row(m, 3);
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const glm::mat3& m) {
  os << glm::row(m, 0);
  os << glm::row(m, 1);
  os << glm::row(m, 2);
  return os;
}

}  // namespace MapCreator

#endif  // MAPCREATOR_UTILITY_H
