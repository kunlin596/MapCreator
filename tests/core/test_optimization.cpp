// Unit tests for the Levenberg-Marquardt reprojection-error optimizer.
#include <gtest/gtest.h>

#include "Core/Optimization.h"

using namespace MapCreator;

// The identity transform applied to identical correspondences must reproject
// onto themselves, giving zero error.
TEST(LevenbergMarquardt, IdentityGivesZeroErrorForIdenticalPoints) {
  const CoordinateConverter conv;
  const Points pts = {{0.1f, 0.2f, -1.0f},
                      {0.3f, -0.1f, -1.5f},
                      {-0.2f, 0.05f, -0.8f}};
  const float err =
      LevenbergMarquardt::GetError(cv::Matx44f::eye(), conv, pts, pts);
  EXPECT_NEAR(err, 0.0f, 1e-4f);
}
