//
// Created by LinKun on 10/6/15.
//

#pragma once

#include <eigen3/Eigen/Dense>

#include <atomic>

#include <glm/glm.hpp>

#include "Camera.h"
#include "Feature.h"
#include "PointCloud.h"

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
                    const Feature::Type& type = Feature::Type::kORB,
                    const std::shared_ptr<FrameBase>& parent = nullptr)
      : CameraBase(parent),
        id_(NextId()),
        name_(name),
        type_(type),
        is_used_(false),
        pointCloud_(pointcloud) {
    // Derive the 2D feature from the cloud's color image (grayscale). Empty
    // clouds (e.g. default-constructed) leave the feature default-constructed.
    const ColorImage& color = pointCloud_.GetColorImage();
    if (!color.empty()) {
      cv::Mat gray;
      cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
      feature_ = Feature(cv::Mat_<uchar>(gray), type_);
    }
  }

  // Default-constructible (empty) so KeyFrames can be stored as members and in
  // resizable containers; populated instances use the constructor above.
  KeyFrame() : id_(NextId()), type_(Feature::Type::kUnknown), is_used_(false) {}

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

  // Convenience accessors delegating to the point cloud's images.
  const ColorImage& GetColorImage() const { return pointCloud_.GetColorImage(); }
  const PointImage& GetPointImage() const { return pointCloud_.GetPointImage(); }

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

  // Alignment (pose) matrices computed by the tracker, in glm form for the GL
  // viewers: the estimated transform and the marker/ground-truth ("answer").
  void SetAlignmentMatrix(const glm::mat4& m) { alignment_matrix_ = m; }
  const glm::mat4& GetAlignmentMatrix() const { return alignment_matrix_; }
  void SetAnswerAlignmentMatrix(const glm::mat4& m) { answer_alignment_matrix_ = m; }
  const glm::mat4& GetAnswerAlignmentMatrix() const {
    return answer_alignment_matrix_;
  }

  /**
   * @brief      Determines if used.
   *
   * @return     True if used, False otherwise.
   */
  bool IsUsed() const noexcept { return is_used_; }

 private:
  // Sequential, process-unique id. The redesigned ctor no longer takes an
  // external frame id, so one is assigned automatically on construction.
  static int NextId() {
    static std::atomic<int> counter{0};
    return counter++;
  }

  int id_;  ///< Unique id of the key frame

  std::string name_;
  Feature::Type type_;
  bool is_used_;

  Feature feature_;
  PointCloudXYZRGB pointCloud_;

  glm::mat4 alignment_matrix_{1.0f};        ///< Estimated alignment (identity)
  glm::mat4 answer_alignment_matrix_{1.0f};  ///< Marker/ground-truth alignment

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
