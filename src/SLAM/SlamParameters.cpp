//
// Created by LinKun on 10/26/15.
//

#include "SLAM/SlamParameters.h"

namespace MapCreator {

    template<>
    Parameters::Consecutive Parameters::GetParameters() { return paramsConsectutive; }

    template<>
    Parameters::FixedNumber Parameters::GetParameters() { return paramsFixedNumber; }

    template<>
    Parameters::KeyFrameOnly Parameters::GetParameters() { return paramsKeyFramesOnly; }
}