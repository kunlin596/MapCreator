//
// Created by LinKun on 9/12/15.
//

#ifndef MAPCREATOR_POINTCLOUD_H
#define MAPCREATOR_POINTCLOUD_H

#include "Image.h"
#include <opencv2/opencv.hpp>

namespace MapCreator {

class PointCloudXYZRGB {
public:
    PointCloudXYZRGB() = default;

    PointCloudXYZRGB(const ColorImage& color_image,
        const PointImage& point_image);

    PointCloudXYZRGB operator=(const PointCloudXYZRGB& other)
    {
        return this->Clone();
    }

    PointCloudXYZRGB(const PointCloudXYZRGB& other)
    {
        color_image_ = other.GetColorImage().clone();
        point_image_ = other.GetPointImage().clone();
    }

    ~PointCloudXYZRGB() = default;

    PointCloudXYZRGB Clone() const;

    const ColorImage& GetColorImage() const { return color_image_; }
    const PointImage& GetPointImage() const { return point_image_; }

    using Ptr = std::shared_ptr<PointCloudXYZRGB>;
    using ConstPtr = std::shared_ptr<const PointCloudXYZRGB>;

private:
    ColorImage color_image_;
    PointImage point_image_;
};

} // namespace MapCreator

#endif // MAPCREATOR_POINTCLOUD_H
