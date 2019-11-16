//
// Created by LinKun on 9/12/15.
//

#include "Core/MyMath.h"

namespace MapCreator {

Quaternion CreateQuaternion(const cv::Vec3f &axis, float theta) {
  const cv::Vec3f v = Normalize(axis);
  const float c = cos(theta / 2);
  const float s = sin(theta / 2);

  Quaternion q;
  q(0) = c;
  q(1) = s * v(0);
  q(2) = s * v(1);
  q(3) = s * v(2);

  return q;
}

Quaternion CreateQuaternion(const cv::Matx33f &m) {
  Quaternion q;

  const float trace = m(0, 0) + m(1, 1) + m(2, 2) + 1.0f;
  if (trace >= 1.0f) {
    const float s = 0.5f / sqrt(trace);
    q(0) = 0.25f / s;
    q(1) = (m(1, 2) - m(2, 1)) * s;
    q(2) = (m(2, 0) - m(0, 2)) * s;
    q(3) = (m(0, 1) - m(1, 0)) * s;
  } else {
    const float max = m(1, 1) > m(2, 2) ? m(1, 1) : m(2, 2);
    if (max < m(0, 0)) {
      float s = sqrt(m(0, 0) - (m(1, 1) + m(2, 2)) + 1.0f);
      q(1) = s * 0.5f;
      s = 0.5f / s;
      q(2) = (m(0, 1) + m(1, 0)) * s;
      q(3) = (m(2, 0) + m(0, 2)) * s;
      q(0) = (m(1, 2) - m(2, 1)) * s;
    } else if (max == m(1, 1)) {
      float s = sqrt(m(1, 1) - (m(2, 2) + m(0, 0)) + 1.0f);
      q(2) = s * 0.5f;
      s = 0.5f / s;
      q(1) = (m(0, 1) + m(1, 0)) * s;
      q(3) = (m(1, 2) + m(2, 1)) * s;
      q(0) = (m(2, 0) - m(0, 2)) * s;
    } else {
      float s = sqrt(m(2, 2) - (m(0, 0) + m(1, 1)) + 1.0f);
      q(3) = s * 0.5f;
      s = 0.5f / s;
      q(1) = (m(2, 0) + m(0, 2)) * s;
      q(2) = (m(1, 2) + m(2, 1)) * s;
      q(0) = (m(0, 1) - m(1, 0)) * s;
    }
  }

  return Normalize(q);
}

Quaternion Slerp(const Quaternion &q1, const Quaternion &q2, float t) {
  Quaternion q;

  const float qr =
      q1(0) * q2(0) + q1(1) * q2(1) + q1(2) * q2(2) + q1(3) * q2(3);
  const float ss = 1.0f - (qr * qr);

  if (ss <= 0.0) {
    q(0) = q1(0);
    q(1) = q1(1);
    q(2) = q1(2);
    q(3) = q1(3);
  } else {
    const float ph = acos(qr);
    const float s1 = sin(ph * (1.0f - t)) / sin(ph);
    const float s2 = sin(ph * t) / sin(ph);

    if (qr < 0.0) {
      q(0) = q1(0) * s1 - q2(0) * s2;
      q(1) = q1(1) * s1 - q2(1) * s2;
      q(2) = q1(2) * s1 - q2(2) * s2;
      q(3) = q1(3) * s1 - q2(3) * s2;
    } else {
      q(0) = q1(0) * s1 + q2(0) * s2;
      q(1) = q1(1) * s1 + q2(1) * s2;
      q(2) = q1(2) * s1 + q2(2) * s2;
      q(3) = q1(3) * s1 + q2(3) * s2;
    }
  }

  return Normalize(q);
}

Quaternion Log(const Quaternion &q) {
  Quaternion dst = q;

  const float t = acosf(dst(0));
  const float sin_t = sinf(t);

  if (sin_t > 0 || sin_t < 0) {
    const float s = t / sin_t;
    dst(0) = 0;
    dst(1) *= s;
    dst(2) *= s;
    dst(3) *= s;
  } else {
    dst = Quaternion(1, 0, 0, 0);
  }

  return dst;
}

cv::Matx44f CreateMatrix44(const cv::Vec3f &v) {
  cv::Matx44f m = cv::Matx44f::eye();
  m(3, 0) = v(0);
  m(3, 1) = v(1);
  m(3, 2) = v(2);
  return m;
}

cv::Matx44f CreateMatrix44(const cv::Matx33f& r) {
    cv::Matx44f m = cv::Matx44f::eye();
    m(0, 0) = r(0, 0); m(0, 1) = r(0, 1); m(0, 2) = r(0, 2);
    m(1, 0) = r(1, 0); m(1, 1) = r(1, 1); m(1, 2) = r(1, 2);
    m(2, 0) = r(2, 0); m(2, 1) = r(2, 1); m(2, 2) = r(2, 2);
    return m;
}

cv::Matx44f CreateMatrix44(const cv::Matx33f &r, const cv::Vec3f &t) {
  cv::Matx44f m = cv::Matx44f::eye();
  m(3, 0) = t(0);
  m(3, 1) = t(1);
  m(3, 2) = t(2);
  m(0, 0) = r(0, 0); m(0, 1) = r(0, 1); m(0, 2) = r(0, 2);
  m(1, 0) = r(1, 0); m(1, 1) = r(1, 1); m(1, 2) = r(1, 2);
  m(2, 0) = r(2, 0); m(2, 1) = r(2, 1); m(2, 2) = r(2, 2);
  return m;
}

Quaternion Exp(const Quaternion &q) {
  Quaternion dst = q;

  const float x = dst(1);
  const float y = dst(2);
  const float z = dst(3);
  const float t = sqrtf(x * x + y * y + z * z);

  if (t > 0 || t < 0) {
    const float s = sinf(t) / t;
    dst(0) = cosf(t);
    dst(1) *= s;
    dst(2) *= s;
    dst(3) *= s;
  } else {
    dst = Quaternion(1, 0, 0, 0);
  }

  return dst;
}

Quaternion ExpMap(const std::vector<Quaternion> &quaternions,
                  const std::vector<float> &weights) {
  Quaternion q;

  float norm_weight = 0;
  for (const float weight : weights) {
    norm_weight += weight;
  }

  for (size_t i = 0; i < quaternions.size(); ++i) {
    q += Log(quaternions[i]) * (weights[i] / norm_weight);
  }

  return Normalize(Exp(q));
}

cv::Matx33f CreateRotationMatrix(const Quaternion &q) {
  const float x2 = q(1) * q(1) * 2;
  const float y2 = q(2) * q(2) * 2;
  const float z2 = q(3) * q(3) * 2;
  const float xy = q(1) * q(2) * 2;
  const float yz = q(2) * q(3) * 2;
  const float zx = q(3) * q(1) * 2;
  const float xw = q(1) * q(0) * 2;
  const float yw = q(2) * q(0) * 2;
  const float zw = q(3) * q(0) * 2;

  cv::Matx33f m;
  m(0, 0) = 1.0f - y2 - z2;
  m(0, 1) = xy + zw;
  m(0, 2) = zx - yw;
  m(1, 0) = xy - zw;
  m(1, 1) = 1.0f - z2 - x2;
  m(1, 2) = yz + xw;
  m(2, 0) = zx + yw;
  m(2, 1) = yz - xw;
  m(2, 2) = 1.0f - x2 - y2;

  return m;
}

cv::Matx33f CreateRotationMatrix(const cv::Vec3f &axis, float theta) {
  return CreateRotationMatrix(CreateQuaternion(axis, theta));
}

cv::Matx33f CreateRotationMatrixX(float theta) {
  return CreateRotationMatrix(cv::Vec3f(1, 0, 0), theta);
}

cv::Matx33f CreateRotationMatrixY(float theta) {
  return CreateRotationMatrix(cv::Vec3f(0, 1, 0), theta);
}

cv::Matx33f CreateRotationMatrixZ(float theta) {
  return CreateRotationMatrix(cv::Vec3f(0, 0, 1), theta);
}

cv::Matx33f CreateRotationMatrix(const cv::Vec3f &v1, const cv::Vec3f &v2) {
  const cv::Vec3f axis = v1.cross(v2);
  const float theta = acos(v1.dot(v2));

  if (std::isfinite(theta) != 0 && cv::norm(axis) > 0) {
    return CreateRotationMatrix(axis, theta);
  } else {
    return cv::Matx33f::eye();
  }
}

cv::Matx44f Interpolate(const cv::Matx44f &m1, const cv::Matx44f &m2, float t) {
  // 回転をクォータニオンで補間
  cv::Matx33f r1;
  r1 << m1;
  cv::Matx33f r2;
  r2 << m2;
  const Quaternion q1 = CreateQuaternion(r1);
  const Quaternion q2 = CreateQuaternion(r2);
  const Quaternion q = Slerp(q1, q2, t);
  const cv::Matx33f r = CreateRotationMatrix(q);
  // 並進の補間
  cv::Vec3f v1;
  v1 << m1;
  cv::Vec3f v2;
  v2 << m2;
  const cv::Vec3f v = Interpolate(v1, v2, t);

  cv::Matx44f m = cv::Matx44f::eye();
  m << r << v;
  return m;
}

}  // namespace MapCreator
