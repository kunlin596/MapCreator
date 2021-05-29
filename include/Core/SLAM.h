#pragma once

#include "Image.h"

namespace MapCreator {

// using Matrices = std::vector<glm::mat4>;
using KeyPoints = std::vector<cv::KeyPoint>;
using CorrespondingPointsPair = std::pair<PointImage, PointImage>;
using InlierPoints = PointImage;

enum class TrackingType { Unknown, Consecutive, KeyFrameOnly, FixedNumber };

// using PointPair = std::pair<glm::vec3, glm::vec3>;

using ScreenPoint = cv::Point2f;
using WorldPoint = cv::Point3f;

struct MatchedPoint {
  int id;
  ScreenPoint screen_point;
  WorldPoint world_point;
};

using MatchedPoints = std::vector<MatchedPoint>;

}  // namespace MapCreator
