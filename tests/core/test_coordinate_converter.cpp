// Unit tests for the coordinate converters used by the data handler.
#include <gtest/gtest.h>

#include "Core/CoordinateConverter.h"

using namespace MapCreator;

TEST(CoordinateConverter, UnprojectPreservesElementCountAndType) {
  const cv::Mat depth(4, 5, CV_16U, cv::Scalar(800));
  const CoordinateConverter conv;
  const cv::Mat pts = conv.Unproject(depth);
  ASSERT_FALSE(pts.empty());
  EXPECT_EQ(pts.total(), depth.total());
  EXPECT_EQ(pts.type(), CV_32FC3);
}

TEST(CoordinateConverter, UnprojectZeroDepthMapsToOrigin) {
  const cv::Mat depth(3, 3, CV_16U, cv::Scalar(0));
  const CoordinateConverter conv;
  const cv::Mat pts = conv.Unproject(depth).reshape(0, 3);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      EXPECT_NEAR(pts.at<cv::Vec3f>(r, c)[2], 0.0f, 1e-6f);
}

TEST(CoordinateConverter, WorldToScreenCentersOpticalAxis) {
  const CoordinateConverter conv;
  const cv::Point2f c = conv.WorldToScreen(cv::Point3f(0.0f, 0.0f, -2.0f));
  EXPECT_NEAR(c.x, 0.5f, 1e-5f);
  EXPECT_NEAR(c.y, 0.5f, 1e-5f);
}

TEST(CoordinateConverter, WorldToScreenGuardsZeroDepth) {
  const CoordinateConverter conv;
  const cv::Point2f c = conv.WorldToScreen(cv::Point3f(1.0f, 1.0f, 0.0f));
  EXPECT_NEAR(c.x, 0.5f, 1e-5f);
  EXPECT_NEAR(c.y, 0.5f, 1e-5f);
}

// The calibrated converter has not yet been ported (Unproject is stubbed to
// return an empty matrix); pin that contract so the regression is visible when
// it is implemented.
TEST(CalibratedCoordinateConverter, UnprojectIsStubbedEmpty) {
  const cv::Mat depth(2, 2, CV_16U, cv::Scalar(500));
  const CalibratedCoordinateConverter conv;
  EXPECT_TRUE(conv.Unproject(depth).empty());
}
