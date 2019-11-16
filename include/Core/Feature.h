#ifndef MAPCREATOR_FEATURE_H
#define MAPCREATOR_FEATURE_H

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "Serialize.h"

#ifdef ENABLE_OPENCV_CONTRIB
#include <opencv2/xfeatures2d.hpp>
#endif

namespace MapCreator {

class Feature {
 public:
  enum class Type {
    Unknown = -1,  ///< Unknown
    ORB = 0,       ///< ORB
#ifdef ENABLE_OPENCV_CONTRIB
    FREAK,  ///< FREAK
    SIFT,   ///< SIFT
    SURF,   ///< SURF
#endif
  };

  using KeyPoints = std::vector<cv::KeyPoint>;  ///< Keypoints
  using Descriptors = cv::Mat;                  ///< Keypoint descriptors

  Feature();
  Feature(const cv::Mat_<uchar>& image, Type type);
  ~Feature();
  Type GetType() const { return type_; }
  const KeyPoints& GetKeyPoints() const { return key_points_; }
  const Descriptors& GetDescriptors() const { return descriptors_; }

 private:
  Type type_;
  KeyPoints key_points_;                   ///< Keypoints
  Descriptors descriptors_;                ///< Keypoint descriptors
  cv::Ptr<cv::FeatureDetector> detector_;  ///< Keypoint detector
  friend class boost::serialization::access;

  BOOST_SERIALIZATION_SPLIT_MEMBER();

  template <class Archive>
  void save(Archive& ar, const unsigned int version) const {
    const cv::Mat m = descriptors_;
    ar& type_;
    ar& key_points_;
    ar& m;
  }

  template <class Archive>
  void load(Archive& ar, const unsigned int version) {
    cv::Mat m;
    ar& type_;
    ar& key_points_;
    ar& m;
    descriptors_ = m;
  }
};

bool SaveFeature(const std::string& name, const Feature& feature);
bool LoadFeature(const std::string& name, Feature& feature);

}  // namespace MapCreator

#endif  // MAPCREATOR_FEATURE_H
