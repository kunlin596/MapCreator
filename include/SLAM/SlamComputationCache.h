//
// Created by LinKun on 10/23/15.
//

#ifndef NIS_SLAMOPTIONCACHE_H
#define NIS_SLAMOPTIONCACHE_H

#endif //NIS_SLAMOPTIONCACHE_H

#include "SLAM/Option.h"

namespace NiS {


	template < class TrackingOptionType >
	struct SlamComputationCache
	{
		TrackingOptionType        options;
		std::vector < int >       keyframe_id;
		std::vector < glm::mat4 > matrices;
	};

	template < >
	struct SlamComputationCache
	{
		BasicSlamOptions          options;
		std::vector < int >       keyframe_id;
		std::vector < glm::mat4 > matrices;
	};

	template < >
	struct SlamComputationCache
	{
		PCAKeyFrameTrackingOptions options;
		std::vector < int >        keyframe_id;
		std::vector < glm::mat4 >  matrices;
	};

	template < >
	struct SlamComputationCache
	{
		FixedFrameCountTrackingOptions options;
		std::vector < int >            keyframe_id;
		std::vector < glm::mat4 >      matrices;
	};
}