//
// Created by LinKun on 9/12/15.
//

#ifndef LK_SLAM_POINTCLOUD_H
#define LK_SLAM_POINTCLOUD_H

#include <opencv2/opencv.hpp>

namespace MapCreator {

    class PointCloud {

    public:

        using ColorImage = cv::Mat_<cv::Vec3b>;
        using PointImage = cv::Mat_<cv::Vec3f>;

        PointCloud();

        PointCloud(const ColorImage &color_image, const PointImage &point_image);

        ~PointCloud();

        PointCloud Clone() const;

        // void Render() const;

        const ColorImage &GetColorImage() const { return color_image_; }
        const PointImage &GetPointImage() const { return point_image_; }

    private:

        ColorImage color_image_;
        PointImage point_image_;

    };

    using PointClouds = std::vector<PointCloud>;
}

#endif //LK_SLAM_POINTCLOUD_H
