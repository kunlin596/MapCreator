#pragma once

#include <glm/glm.hpp>

#include "Image.h"

namespace MapCreator {

using KeyPoints = std::vector<cv::KeyPoint>;

// A pair of 3D points (e.g. a rendered correspondence) in glm form.
using PointPair = std::pair<glm::vec3, glm::vec3>;

using ScreenPoint = cv::Point2f;
using WorldPoint = cv::Point3f;

// An ordered list of 3D world points (e.g. matched correspondences).
using Points = std::vector<WorldPoint>;
using CorrespondingPointsPair = std::pair<Points, Points>;

// Inliers are a sparse list of matched 3D correspondences, not a dense image.
using InlierPoints = Points;

enum class TrackingType { Unknown, Consecutive, KeyFrameOnly, FixedNumber };

struct MatchedPoint {
  int id;
  ScreenPoint screen_point;
  WorldPoint world_point;
};

using MatchedPoints = std::vector<MatchedPoint>;

}  // namespace MapCreator
