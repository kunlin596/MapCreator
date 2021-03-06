//
// Created by LinKun on 9/13/15.
//

#ifndef MAPCREATOR_TRANFORMATION_H
#define MAPCREATOR_TRANFORMATION_H


#include <Core/MyMath.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <random>
#include <ctime>

#include "SLAM/SLAM.h"


namespace MapCreator {

	// ２つの点群間の変換行列（point1 → points2）を求める
	cv::Matx44f ComputeTransformationMatrix (
			const Points & points1 ,
			const Points & points2 ,
			const int num_ransac ,
			const double outlier_threshold ,
			const double inlier_threshold );


	// こっちは RANSAC を用いずにそのまま全点に対して計算を行うバージョン
	cv::Matx44f ComputeTransformationMatrix (
			const Points & points1 ,
			const Points & points2 );


	std::pair < InlierPoints , InlierPoints > ComputeInliers ( const Points & points1 ,
	                                                           const Points & points2 ,
	                                                           const int num_ransac ,
	                                                           const double outlier_threshold ,
	                                                           const double inlier_threshold );

	std::pair < InlierPoints , InlierPoints > ComputeInliersWithFlow ( const Points & points1 ,
	                                                                   const Points & points2 ,
	                                                                   const int num_ransac ,
	                                                                   const double outlier_threshold ,
	                                                                   const double inlier_threshold );

}    // MapCreator

#endif //MAPCREATOR_TRANFORMATION_H
