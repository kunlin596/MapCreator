// Unit tests for the MapCreator::Core math utilities (MyMath.h / MyMath.cpp).
#include <gtest/gtest.h>

#include <cmath>

#include "Core/MyMath.h"

using namespace MapCreator;

namespace {
constexpr float kEps = 1e-4f;

void ExpectMatx33Near(const cv::Matx33f& a, const cv::Matx33f& b,
                      float eps = kEps) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      EXPECT_NEAR(a(i, j), b(i, j), eps) << "mismatch at (" << i << "," << j << ")";
}
}  // namespace

TEST(MyMathAngles, PiScaling) {
  EXPECT_NEAR(PI(1.0f), static_cast<float>(M_PI), kEps);
  EXPECT_NEAR(PI(2.0), 2.0 * M_PI, 1e-9);
}

TEST(MyMathAngles, DegreeRadianRoundTrip) {
  EXPECT_NEAR(Radian(180.0f), static_cast<float>(M_PI), kEps);
  EXPECT_NEAR(Degree(static_cast<float>(M_PI)), 180.0f, 1e-2f);
  for (float d : {0.0f, 30.0f, 45.0f, 90.0f, 137.0f})
    EXPECT_NEAR(Degree(Radian(d)), d, 1e-2f);
}

TEST(MyMathVec, NormalizeProducesUnitLength) {
  const cv::Vec3f n = Normalize(cv::Vec3f(3.0f, 0.0f, 4.0f));
  EXPECT_NEAR(cv::norm(n), 1.0f, kEps);
}

TEST(MyMathRotation, IdentityAtZeroAngle) {
  ExpectMatx33Near(CreateRotationMatrixX(0.0f), cv::Matx33f::eye());
  ExpectMatx33Near(CreateRotationMatrixY(0.0f), cv::Matx33f::eye());
  ExpectMatx33Near(CreateRotationMatrixZ(0.0f), cv::Matx33f::eye());
}

TEST(MyMathRotation, IsOrthonormal) {
  const cv::Matx33f r = CreateRotationMatrixZ(Radian(37.0f));
  ExpectMatx33Near(r * r.t(), cv::Matx33f::eye());
  EXPECT_NEAR(cv::determinant(r), 1.0f, kEps);
}

TEST(MyMathRotation, PreservesVectorLength) {
  const cv::Matx33f r = CreateRotationMatrixY(Radian(63.0f));
  const cv::Vec3f v(1.0f, -2.0f, 0.5f);
  EXPECT_NEAR(cv::norm(v * r), cv::norm(v), kEps);
}

TEST(MyMathRotation, ZRotationByNinetyMapsXAxisOntoYAxis) {
  const cv::Matx33f r = CreateRotationMatrixZ(Radian(90.0f));
  const cv::Vec3f rotated = cv::Vec3f(1.0f, 0.0f, 0.0f) * r;
  EXPECT_NEAR(std::abs(rotated(1)), 1.0f, kEps);  // direction-sign agnostic
  EXPECT_NEAR(rotated(0), 0.0f, kEps);
  EXPECT_NEAR(rotated(2), 0.0f, kEps);
}

TEST(MyMathMatrix44, TranslationStoredInBottomRow) {
  const cv::Matx44f m = CreateMatrix44(cv::Vec3f(1.0f, 2.0f, 3.0f));
  EXPECT_NEAR(m(3, 0), 1.0f, kEps);
  EXPECT_NEAR(m(3, 1), 2.0f, kEps);
  EXPECT_NEAR(m(3, 2), 3.0f, kEps);
  EXPECT_NEAR(m(3, 3), 1.0f, kEps);
}

TEST(MyMathQuaternion, RotationMatrixRoundTrip) {
  const cv::Matx33f r =
      CreateRotationMatrixZ(Radian(40.0f)) * CreateRotationMatrixX(Radian(20.0f));
  const Quaternion q = CreateQuaternion(r);
  ExpectMatx33Near(CreateRotationMatrix(q), r, 1e-3f);
}
