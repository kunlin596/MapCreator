// Unit tests for the rigid-transform estimation in Transformation.h.
#include <gtest/gtest.h>

#include "Core/MyMath.h"
#include "Core/Transformation.h"

using namespace MapCreator;

TEST(Transformation, RecoversKnownRigidTransform) {
  const Points points1 = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f},
                          {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
                          {1.0f, 1.0f, 0.0f}, {0.5f, 0.2f, 0.9f}};

  const cv::Matx33f R = CreateRotationMatrixZ(Radian(30.0f));
  const cv::Vec3f t(0.5f, -0.3f, 1.2f);

  // points2 = points1 * R + t  (row-vector convention, matching ComputeErrors).
  Points points2;
  for (const auto& p : points1) {
    const cv::Vec3f r = cv::Vec3f(p.x, p.y, p.z) * R;
    points2.emplace_back(r[0] + t[0], r[1] + t[1], r[2] + t[2]);
  }

  const cv::Matx44f M = ComputeTransformationMatrix(points1, points2);

  for (size_t i = 0; i < points1.size(); ++i) {
    const cv::Vec4f v =
        cv::Vec4f(points1[i].x, points1[i].y, points1[i].z, 1.0f) * M;
    EXPECT_NEAR(v[0], points2[i].x, 1e-3f);
    EXPECT_NEAR(v[1], points2[i].y, 1e-3f);
    EXPECT_NEAR(v[2], points2[i].z, 1e-3f);
  }
}
