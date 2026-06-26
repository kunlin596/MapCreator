// Unit tests for MapCreator::KeyFrame (immutable, point-cloud-constructed).
#include <gtest/gtest.h>

#include "Core/KeyFrame.h"

using namespace MapCreator;

namespace {
PointCloudXYZRGB MakeCloud(int rows, int cols) {
  ColorImage color(rows, cols, cv::Vec3b(10, 20, 30));
  PointImage points(rows, cols, cv::Vec3f(0.0f, 0.0f, 1.0f));
  return PointCloudXYZRGB(color, points);
}
}  // namespace

TEST(KeyFrame, StoresNameTypeAndCloud) {
  const KeyFrame kf("frame_0", MakeCloud(48, 64), Feature::Type::kORB);
  EXPECT_EQ(kf.GetName(), "frame_0");
  EXPECT_EQ(kf.GetFeatureType(), Feature::Type::kORB);
  EXPECT_EQ(kf.GetPointCloud().GetColorImage().size(), cv::Size(64, 48));
  EXPECT_EQ(kf.GetPointCloud().GetPointImage().size(), cv::Size(64, 48));
}

TEST(KeyFrame, IsUsedFlagToggles) {
  KeyFrame kf("frame_1", MakeCloud(8, 8));
  EXPECT_FALSE(kf.IsUsed());
  kf.SetIsUsed(true);
  EXPECT_TRUE(kf.IsUsed());
}

TEST(KeyFrame, AssignsUniqueSequentialIds) {
  const KeyFrame a("a", MakeCloud(8, 8));
  const KeyFrame b("b", MakeCloud(8, 8));
  EXPECT_NE(a.GetId(), b.GetId());
}

TEST(KeyFrame, DerivesFeatureFromColorImage) {
  // A textured image should yield some ORB keypoints; mainly we assert the
  // feature is computed without throwing and carries the requested type.
  const KeyFrame kf("frame_2", MakeCloud(64, 64), Feature::Type::kORB);
  EXPECT_EQ(kf.GetFeature().GetType(), Feature::Type::kORB);
}
