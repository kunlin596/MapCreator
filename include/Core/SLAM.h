#pragma once

#include "Image.h"

namespace MapCreator {

using KeyPoints = std::vector<cv::KeyPoint>;

using ScreenPoint = cv::Point2f;
using WorldPoint = cv::Point3f;

// An ordered list of 3D world points (e.g. matched correspondences).
using Points = std::vector<WorldPoint>;
using CorrespondingPointsPair = std::pair<Points, Points>;

using InlierPoints = PointImage;

enum class TrackingType { Unknown, Consecutive, KeyFrameOnly, FixedNumber };

struct MatchedPoint {
  int id;
  ScreenPoint screen_point;
  WorldPoint world_point;
};

using MatchedPoints = std::vector<MatchedPoint>;

}  // namespace MapCreator
