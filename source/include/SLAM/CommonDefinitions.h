#ifndef NIS_COMMON_DEFINITIONS_H
#define NIS_COMMON_DEFINITIONS_H

#include <opencv2/opencv.hpp>
#include <glm/glm.hpp>
#include <memory>

namespace NiS {

	using Matrices = std::vector < glm::mat4 >;
	using KeyPoints = std::vector < cv::KeyPoint >;
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

	using ScreenPoint = cv::Point2f;
	using WorldPoint = cv::Point3f;

	struct MatchedPoint
	{
		int         id;
		ScreenPoint screen_point;
		WorldPoint  world_point;
	};

	using MatchedPoints = std::vector < MatchedPoint >;

}

#endif