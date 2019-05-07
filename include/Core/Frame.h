#pragma once

#include "Core/Pose.h"
#include "SLAM/SLAM.h"

namespace MapCreator {
    template <typename T>
    struct Frame_ {
        Pose_<T> pose;
        float width;
        float height;
    };

    using Frame = Frame_<float>;

    template <typename T>
    struct RGBDFrame_: Frame_<T> {
        ColorImage colorimage;
        DepthImage depthimage;
    };

    using RGBDFrame = RGBDFrame_<float>;
}