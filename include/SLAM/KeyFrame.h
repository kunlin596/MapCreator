//
// Created by LinKun on 10/6/15.
//

#pragma once

#include <Eigen/Dense>

#include "Core/Camera.h"
#include "Core/Feature.h"
#include "Core/PointCloud.h"

namespace Eigen {
using Matrix4f = Matrix<float, 4, 4>;
}

namespace MapCreator {

/**
 * @brief      Class for key frame.
 */
class KeyFrame : public CameraBase {
 public:
  explicit KeyFrame(const std::string& name, const PointCloudXYZRGB& pointcloud,
                    const Feature::Type& type = Feature::Type::ORB)
      : name_(name), type_(type), is_used_(false) {}

  KeyFrame() = delete;

  ~KeyFrame() = default;

  /**
   * @brief      Gets the identifier.
   *
   * @return     The identifier.
   */
  int GetId() const noexcept { return id_; }

  /**
   * @brief      Gets the point cloud.
   *
   * @return     The point cloud.
   */
  const PointCloudXYZRGB& GetPointCloud() const { return pointCloud_; }

  /**
   * @brief      Gets the name.
   *
   * @return     The name.
   */
  const std::string& GetName() const noexcept { return name_; }

  /**
   * @brief      Gets the feature.
   *
   * @return     The feature.
   */
  const Feature& GetFeature() const noexcept { return feature_; }

  /**
   * @brief      Gets the feature type.
   *
   * @return     The feature type.
   */
  const Feature::Type& GetFeatureType() const noexcept { return type_; }

  /**
   * @brief      Sets the is used.
   *
   * @param[in]  is_used  Indicates if used
   */
  void SetIsUsed(bool is_used) { is_used_ = is_used; }

  /**
   * @brief      Determines if used.
   *
   * @return     True if used, False otherwise.
   */
  bool IsUsed() const noexcept { return is_used_; }

 private:
  int id_;  ///< Unique id of the key frame

  std::string name_;
  Feature::Type type_;
  bool is_used_;

  Feature feature_;
  PointCloudXYZRGB pointCloud_;

  // Boost serialization methods
  // friend class boost::serialization::access;

  // BOOST_SERIALIZATION_SPLIT_MEMBER();
  // template <class Archive>
  // void save(Archive& ar, const unsigned int version) const {
  //   const cv::Mat color = colorimage;
  //   const cv::Mat point = pointimage;

  //   ar& color;
  //   ar& point;
  //   ar& name_;
  //   ar& feature_;
  // }
  // template <class Archive>
  // void load(Archive& ar, const unsigned int version) {
  //   cv::Mat color;
  //   cv::Mat point;
  //   std::string name;
  //   Feature feature;

  //   ar& color;
  //   ar& point;
  //   ar& name;
  //   ar& feature;

  //   colorimage = color;
  //   pointimage = point;
  //   name_ = name;
  //   feature_ = feature;
  // }

  // inline void CreateFeature() {
  //     QFileInfo info(QString::fromStdString(name_));

  //     auto path = info.absolutePath();
  //     auto name = info.completeBaseName();

  //     // QString feature_prefix;
  //     switch (type_) {
  //       case Feature::Type::kTypeORB:
  //         feature_prefix = "ORB/orb_";
  //         break;
  // #ifdef ENABLE_OPENCV_CONTRIB
  //       case Feature::Type::kTypeSIFT:
  //         feature_prefix = "SIFT/sift_";
  //         break;
  //       case Feature::Type::kTypeSURF:
  //         feature_prefix = "SURF/surf_";
  //         break;
  //       case Feature::Type::kTypeFREAK:
  //         feature_prefix = "FREAK/freak_";
  //         break;
  // #endif
  //       default:
  //         break;
  //     }

  //   QString feature_folder_path = path + "/Features/";

  //   QDir dir(feature_folder_path);
  //   if (!dir.exists()) dir.mkdir(feature_folder_path);

  //   if (!LoadFeature(
  //           (feature_folder_path + name + "." + "feature").toStdString(),
  //           feature_)) {
  //     assert(!colorimage.empty());
  //     cv::Mat cvt_color_image;
  //     cv::cvtColor(colorimage, cvt_color_image, cv::COLOR_RGB2GRAY);
  //     feature_ = Feature(cvt_color_image, type_);
  //     SaveFeature(
  //         QString(feature_folder_path + name + ".feature").toStdString(),
  //         feature_);
  //   }
  // }
};

using KeyFrames = std::vector<KeyFrame>;

};  // namespace MapCreator
