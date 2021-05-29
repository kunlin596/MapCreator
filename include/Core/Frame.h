#pragma once

#include <eigen3/Eigen/Dense>

#include "MyMath.h"

namespace MapCreator {

class FrameBase {
 public:
  FrameBase(const std::shared_ptr<FrameBase>& parent = nullptr)
      : parent_(parent) {}
  std::weak_ptr<FrameBase> GetParent() const { return parent_; }
  virtual ~FrameBase() {}

  /**
   * @brief      Gets the pose.
   *
   * @return     The pose.
   */
  const Pose& GetPose() const { return pose_; }

  /**
   * @brief      Gets the rotation.
   *
   * @return     The rotation.
   */
  cv::Matx33f GetRotation() const {
    return CreateRotationMatrix(pose_.quaternion);
  }

  /**
   * @brief      Gets the translation.
   *
   * @return     The translation.
   */
  cv::Vec3f GetTranslation() const { return pose_.translation; }

  /**
   * @brief      Gets the transform.
   *
   * @return     The transform.
   */
  cv::Matx44f GetTransform() const {
    return CreateMatrix44(CreateRotationMatrix(pose_.quaternion),
                          pose_.translation);
  }

 protected:
  Pose pose_;

  std::weak_ptr<FrameBase> parent_;
};

using FrameBaseSharedPtr = std::shared_ptr<FrameBase>;
using FrameBaseWeakPtr = std::weak_ptr<FrameBase>;

}  // namespace MapCreator
