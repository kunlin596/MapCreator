//
// Created by LinKun on 9/12/15.
//

#ifndef MAPCREATOR_UTILITY_H
#define MAPCREATOR_UTILITY_H

#include <algorithm>
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

}  // namespace MapCreator

#endif  // MAPCREATOR_UTILITY_H
