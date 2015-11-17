//
// Created by LinKun on 11/15/15.
//

#ifndef NIS_ARUCOMARKERUTILS_H
#define NIS_ARUCOMARKERUTILS_H

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>

#include "SLAM/KeyFrame.h"
#include "SLAM/CommonDefinitions.h"

namespace NiS {

	using Markers = std::vector < aruco::Marker >;

	struct ArucoMarkerUtils
	{
		static CorrespondingPointsPair CreatePoints ( Markers & markers_set_1 ,
		                                              Markers & markers_set_2 ,
		                                              KeyFrame & keyframe1 ,
		                                              KeyFrame & keyframe2 );
	};


}

#endif //NIS_ARUCOMARKERUTILS_H
