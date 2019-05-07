//
// Created by LinKun on 10/26/15.
//

#include "SLAM/SlamParameters.h"

namespace MapCreator {

    template<>
    TrackerParameters::Consecutive TrackerParameters::GetParameters() { return paramsConsectutive; }

    template<>
    TrackerParameters::FixedNumber TrackerParameters::GetParameters() { return paramsFixedNumber; }

    template<>
    TrackerParameters::KeyFrameOnly TrackerParameters::GetParameters() { return paramsKeyFramesOnly; }
}