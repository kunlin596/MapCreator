#ifndef MAPCREATOR_SLAMPARAMETERS_H
#define MAPCREATOR_SLAMPARAMETERS_H

#include "Core/Serialize.h"
#include "SLAM/SLAM.h"

namespace MapCreator {

struct AlgorithmParameters {
  TrackingType trackingType;
};

struct TrackerParameters {
  TrackerParameters() : type_(TrackingType::Unknown) {}

  TrackingType type;

  template <typename TrackingOptionsType>
  TrackingOptionsType GetParameters();

  template <typename Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &type;
  }
};

struct ConsecutiveTrackerParameters : TrackerParameters {
  int num_ransac_iteration;
  float threshold_outlier;
  float threshold_inlier;

  inline ConsecutiveTrackerParameters()
      : num_ransac_iteration(10000),
        threshold_outlier(0.035f),
        threshold_inlier(0.035f) {}

  inline ConsecutiveTrackerParameters(int num_ransac_iteration, float threshold_outlier,
                     float threshold_inlier)
      : num_ransac_iteration(num_ransac_iteration),
        threshold_outlier(threshold_outlier),
        threshold_inlier(threshold_inlier) {}

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &num_ransac_iteration;
    ar &threshold_outlier;
    ar &threshold_inlier;
  }
};

struct FixedNumberTrackerParameters : public ConsecutiveTrackerParameters {
  int frame_count;

  FixedNumberTrackerParameters() : Consecutive(), frame_count(1) {}

  explicit FixedNumberTrackerParameters(int frame_count)
      : ConsecutiveTrackerParameters(), frame_count(frame_count) {}

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &boost::serialization::base_object<Consecutive>(*this);
    ar &frame_count;
  }
};

struct KeyFrameOnlyTrackerParameters : public ConsecutiveTrackerParameters {
  int num_inliers;
  float threshold_1st_component_contribution;
  float threshold_1st_component_variance;
  float threshold_2nd_component_variance;
  float threshold_3rd_component_variance;

  inline ConsecutiveTrackerParameters()
      : Consecutive(),
        num_inliers(3),
        threshold_1st_component_contribution(0.85f),
        threshold_1st_component_variance(0.1f),
        threshold_2nd_component_variance(0.05f),
        threshold_3rd_component_variance(0.0f) {}

  inline KeyFrameOnly(int num_inliers,
                      float threshold_1st_component_contribution,
                      float threshold_1st_component_variance,
                      float threshold_2nd_component_variance,
                      float threshold_3rd_component_variance)
      : Consecutive(),
        num_inliers(num_inliers),
        threshold_1st_component_contribution(
            threshold_1st_component_contribution),
        threshold_1st_component_variance(threshold_1st_component_variance),
        threshold_2nd_component_variance(threshold_2nd_component_variance),
        threshold_3rd_component_variance(threshold_3rd_component_variance) {}

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &boost::serialization::base_object<Consecutive>(*this);
    ar &num_inliers;
    ar &threshold_1st_component_contribution;
    ar &threshold_1st_component_variance;
    ar &threshold_2nd_component_variance;
    ar &threshold_3rd_component_variance;
  }
};

#endif  // MAPCREATOR_SLAMPARAMETERS_H
