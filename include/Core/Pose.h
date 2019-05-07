#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>

namespace MapCreator {
    template <typename T>
    struct Pose_ {
        cv::Vec<T, 3> translation;
        cv::Matx<T, 3, 3> rotation;
    };
    using Pose = Pose_<float>;
}