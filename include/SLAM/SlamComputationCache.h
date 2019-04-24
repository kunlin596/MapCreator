//
// Created by LinKun on 10/23/15.
//

#ifndef MAPCREATOR_SLAMOPTIONCACHE_H
#define MAPCREATOR_SLAMOPTIONCACHE_H

#endif //MAPCREATOR_SLAMOPTIONCACHE_H

#include "SLAM/Option.h"

namespace MapCreator {


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