#ifndef NIS_COMMON_DEFINITIONS_H
#define NIS_COMMON_DEFINITIONS_H

#include <opencv2/opencv.hpp>
#include <glm/glm.hpp>

namespace NiS {

	using Matrices = std::vector < glm::mat4 >;
	using Points = std::vector < cv::Point3f >;
	using CorrespondingPointsPair = std::pair < Points , Points >;
	using InlierPoints = Points;
	using PointImage = cv::Mat_ < cv::Vec3f >;
	using ColorImage = cv::Mat_ < cv::Vec3b >;
	using DepthImage = cv::Mat_ < ushort >;

	enum TrackingType
	{
		Unknown  = -1 ,
		OneByOne = 0 ,
		PcaKeyFrame ,
		FixedFrameCount
	};

	using PointPair = std::pair < glm::vec3 , glm::vec3 >;

	struct MatchedPoint
	{
		int       id;
		glm::vec2 screen_coordinate;
		glm::vec3 world_coordinate;
	};

	using MatchedPoints = std::vector < MatchedPoint >;

}

#endif