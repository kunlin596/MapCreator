//
// Created by LinKun on 9/12/15.
//

#ifndef MAPCREATOR_MATH_H
#define MAPCREATOR_MATH_H

#if !defined(_USE_MATH_DEFINES)
#define _USE_MATH_DEFINES
#endif

#include <cmath>
#include <opencv2/opencv.hpp>

#include "Types.h"

namespace MapCreator {

inline float PI(float x) { return static_cast<float>(M_PI) * x; }

inline double PI(double x) { return M_PI * x; }

inline double Degree(double radian) { return radian * 180 / M_PI; }

inline float Degree(float radian) {
  return radian * 180.0f / static_cast<float>(M_PI);
}

inline double Radian(double degree) { return degree * M_PI / 180; }

inline float Radian(float degree) {
  return degree * static_cast<float>(M_PI) / 180.0f;
}

/**
 * @brief      Normalize vector
 *
 * @param[in]  v     { parameter_description }
 *
 * @tparam     T     { description }
 * @tparam     N     { description }
 *
 * @return     { description_of_the_return_value }
 */
template <typename T, int N>
cv::Vec<T, N> Normalize(const cv::Vec<T, N>& v) {
  cv::Vec<T, N> tmp;
  cv::normalize(v, tmp);
  return tmp;
}

/**
 * @brief      Interpolate the vectors
 *
 * @param[in]  v1    The v 1
 * @param[in]  v2    The v 2
 * @param[in]  t     { parameter_description }
 *
 * @tparam     T     { description }
 * @tparam     N     { description }
 *
 * @return     { description_of_the_return_value }
 */
template <typename T, int N>
cv::Vec<T, N> Interpolate(const cv::Vec<T, N>& v1, const cv::Vec<T, N>& v2,
                          float t) {
  return v1 * (1.0f - t) + v2 * t;
}

/**
 * @brief      Creates a translation matrix.
 *
 * @param[in]  v     { parameter_description }
 *
 * @return     { description_of_the_return_value }
 */
cv::Matx44f CreateMatrix44(const cv::Vec3f& v);

/**
 * @brief      Creates a 4x4 rotation matrix.
 *
 * @param[in]  r     { parameter_description }
 *
 * @return     { description_of_the_return_value }
 */
cv::Matx44f CreateMatrix44(const cv::Matx33f& r);

/**
 * @brief      Creates a matrix 4 x 4.
 *
 * @param[in]  r     { parameter_description }
 * @param[in]  t     { parameter_description }
 *
 * @return     { description_of_the_return_value }
 */
cv::Matx44f CreateMatrix44(const cv::Matx33f& r,
                           const cv::Vec3f& t = cv::Vec3f(0, 0, 0));

/**
 * @brief      Creates a quaternion.
 *
 * @param[in]  axis   The axis
 * @param[in]  theta  The theta, in radian
 *
 * @return     Quaternion
 */
Quaternion CreateQuaternion(const cv::Vec3f& axis, float theta);

/**
 * @brief      Creates a quaternion.
 *
 * @param[in]  m     { parameter_description }
 *
 * @return     { description_of_the_return_value }
 */
Quaternion CreateQuaternion(const cv::Matx33f& m);

/**
 * @brief      Slerp
 *
 * @param[in]  q1    The quarter 1
 * @param[in]  q2    The quarter 2
 * @param[in]  t     { parameter_description }
 *
 * @return     { description_of_the_return_value }
 */
Quaternion Slerp(const Quaternion& q1, const Quaternion& q2, float t);

Quaternion Log(const Quaternion& q);

Quaternion Exp(const Quaternion& q);

/// @brief  対数クォータニオンを利用した、複数クォータニオンの補間
/// @param  quaternions     複数のクォータニオン
/// @param  weights         任意のウェイト（quaternions
/// と同じ数である必要がある）
/// @return 補間されたクォータニオン
Quaternion ExpMap(const std::vector<Quaternion>& quaternions,
                  const std::vector<float>& weights);

/**
 * @brief      Creates a rotation matrix.
 *
 * @param[in]  q     The quarter
 *
 * @return     { description_of_the_return_value }
 */
cv::Matx33f CreateRotationMatrix(const Quaternion& q);

/**
 * @brief      Creates a rotation matrix.
 *
 * @param[in]  axis   The axis
 * @param[in]  theta  The theta
 *
 * @return     { description_of_the_return_value }
 */
cv::Matx33f CreateRotationMatrix(const cv::Vec3f& axis, float theta);

/**
 * @brief      Creates a rotation matrix x.
 *
 * @param[in]  theta  The theta of rotation around X axis
 *
 * @return     { description_of_the_return_value }
 */
cv::Matx33f CreateRotationMatrixX(float theta);

/**
 * @brief      Creates a rotation matrix y.
 *
 * @param[in]  theta  The theta of rotation around Y axis
 *
 * @return     { description_of_the_return_value }
 */
cv::Matx33f CreateRotationMatrixY(float theta);

/**
 * @brief      Creates a rotation matrix z.
 *
 * @param[in]  theta  The theta of rotation around Z axis
 *
 * @return     { description_of_the_return_value }
 */
cv::Matx33f CreateRotationMatrixZ(float theta);

/**
 * @brief      Creates a rotation matrix between 2 vectors
 *
 * @param[in]  v1    The v 1
 * @param[in]  v2    The v 2
 *
 * @return     { description_of_the_return_value }
 */
cv::Matx33f CreateRotationMatrix(const cv::Vec3f& v1,
                                 const cv::Vec3f& v2);  // v1 → v2 への回転

/**
 * @brief      Interpolate between 2 4x4 transformation matrices
 *
 * @param[in]  m1    The m 1
 * @param[in]  m2    The m 2
 * @param[in]  t     { parameter_description }
 *
 * @return     { description_of_the_return_value }
 */
cv::Matx44f Interpolate(const cv::Matx44f& m1, const cv::Matx44f& m2, float t);

}  // namespace MapCreator

// Operators are defined in global name space

inline cv::Vec4f operator*(const cv::Vec4f& v, const cv::Matx44f& m) {
  return cv::Vec4f((cv::Matx<float, 1, 4>(v.val) * m).val);
}

inline cv::Vec3f operator*(const cv::Vec3f& v, const cv::Matx44f& m) {
  const cv::Vec4f w = cv::Vec4f(v(0), v(1), v(2), 1.0f) * m;
  return cv::Vec3f(w(0), w(1), w(2));
}

inline cv::Vec3f operator*(const cv::Vec3f& v, const cv::Matx33f& m) {
  return cv::Vec3f((cv::Matx<float, 1, 3>(v.val) * m).val);
}

inline cv::Matx33f& operator<<(cv::Matx33f& r, const cv::Matx44f& m) {
  r = cv::Mat(m)(cv::Rect(0, 0, 3, 3));
  return r;
}

inline cv::Vec3f& operator<<(cv::Vec3f& t, const cv::Matx44f& m) {
  t = cv::Mat(m, false)(cv::Rect(0, 3, 3, 1));
  return t;
}

inline cv::Matx44f& operator<<(cv::Matx44f& m, const cv::Matx33f& r) {
  cv::Mat(r).copyTo(cv::Mat(m, false)(cv::Rect(0, 0, 3, 3)));
  return m;
}

inline cv::Matx44f& operator<<(cv::Matx44f& m, const cv::Vec3f& t) {
  cv::Mat(cv::Matx13f(t(0), t(1), t(2)))
      .copyTo(cv::Mat(m, false)(cv::Rect(0, 3, 3, 1)));
  return m;
}

inline cv::Matx44f& operator<<(cv::Matx44f& dst, const cv::Matx44f& src) {
  dst = src;
  return dst;
}

#endif //MAPCREATOR_MATH_H
