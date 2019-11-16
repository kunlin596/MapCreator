//
// Created by LinKun on 9/12/15.
//

#include "Core/PointCloud.h"

namespace MapCreator {

PointCloudXYZRGB::PointCloudXYZRGB(const ColorImage &color_image,
                                   const PointImage &point_image)
    : color_image_(color_image), point_image_(point_image) {}

PointCloudXYZRGB PointCloudXYZRGB::Clone() const {
  return PointCloudXYZRGB(color_image_.clone(), point_image_.clone());
}

}  // namespace MapCreator