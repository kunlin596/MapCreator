//
// Created by LinKun on 11/15/15.
//

#ifndef MAPCREATOR_ARUCOMARKERUTILS_H
#define MAPCREATOR_ARUCOMARKERUTILS_H

#include <opencv2/opencv.hpp>

#ifdef ENABLE_ARUCO
#include <opencv2/aruco/charuco.hpp>
#endif

#include "SLAM/KeyFrame.h"
#include "SLAM/SLAM.h"

namespace MapCreator {

//        using Markers = std::vector < aruco::Markers >;

    struct ArucoMarkerUtils
    {
// TODO: Fix aruco markers
//      static CorrespondingPointsPair CreatePoints ( Markers & markers_set_1 ,
//                                                    Markers & markers_set_2 ,
//                                                    const KeyFrame & keyframe1 ,
//                                                    const KeyFrame & keyframe2 );
    };


}

#endif //MAPCREATOR_ARUCOMARKERUTILS_H
