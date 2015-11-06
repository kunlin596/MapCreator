//
// Created by LinKun on 10/29/15.
//

#include "SLAM/GlobalOptimization.h"

namespace NiS {

    void LevenbergMarquardt::Compute(cv::Matx44f const &m) {

        // 回転軸
        const auto axis = cv::normalize(cv::Vec3f(m(1, 2) - m(2, 1), m(2, 0) - m(0, 2), m(0, 1) - m(1, 0)));

        // 回転角（ラジアン）
        const auto theta = static_cast<float>(acos((cv::trace(m) - 2) / 2));

        // 並進
        const cv::Vec3f t(m(3, 0), m(3, 1), m(3, 2));

        const auto n = 7;

        Eigen::VectorXf parameters(n);

        parameters << axis(0), axis(1), axis(2), theta, t(0), t(1), t(2);

    }


}