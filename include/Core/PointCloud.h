//
// Created by LinKun on 9/12/15.
//

#ifndef MAPCREATOR_POINTCLOUD_H
#define MAPCREATOR_POINTCLOUD_H

#include <opencv2/opencv.hpp>
#include "Image.h"

namespace MapCreator {

class PointCloudXYZRGB {
 public:
  PointCloudXYZRGB() = default;

  PointCloudXYZRGB(const ColorImage& color_image,
                   const PointImage& point_image);

  PointCloudXYZRGB operator=(const PointCloudXYZRGB& other) {
    return this->Clone();
  }

  PointCloudXYZRGB(const PointCloudXYZRGB& other) {
    color_image_ = other.GetColorImage().clone();
    point_image_ = other.GetPointImage().clone();
  }

  ~PointCloudXYZRGB() = default;

  PointCloudXYZRGB Clone() const;

  const ColorImage& GetColorImage() const { return color_image_; }
  const PointImage& GetPointImage() const { return point_image_; }

 private:
  ColorImage color_image_;
  PointImage point_image_;
};

using PointCloudXYZRGBs = std::vector<PointCloudXYZRGB>;
using PointCloudXYZRGBPtr = std::shared_ptr<PointCloudXYZRGB>;
using PointCloudXYZRGBConstPtr = std::shared_ptr<const PointCloudXYZRGB>;

}  // namespace MapCreator

#endif  // MAPCREATOR_POINTCLOUD_H
