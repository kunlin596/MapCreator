//
// Created by LinKun on 11/15/15.
//

#include "SLAM/ArucoMarkerUtils.h"
#include <algorithm>


namespace NiS {

	CorrespondingPointsPair ArucoMarkerUtils::CreatePoints ( Markers & markers_set_1 , Markers & markers_set_2 ,
	                                                         const KeyFrame & keyframe1 , const KeyFrame & keyframe2 ) {

		using namespace std;

		std::cout << "#  " << keyframe1.GetId ( ) << " - " << keyframe2.GetId ( ) << std::endl;

		auto itr1 = markers_set_1.begin ( );
		auto itr2 = markers_set_2.begin ( );

		Points points1;
		Points points2;

		std::cout << "markers_set1 : ";
		for ( auto i = 0 ; i < markers_set_1.size ( ) ; ++i ) { std::cout << " " << markers_set_1[ i ].id << " "; }
		std::cout << " | markers_set2 : ";
		for ( auto i = 0 ; i < markers_set_2.size ( ) ; ++i ) { std::cout << " " << markers_set_2[ i ].id << " "; }
		std::cout << std::endl;

		while ( itr1 != markers_set_1.end ( ) and itr2 != markers_set_2.end ( ) ) {

			if ( itr1->id == itr2->id ) {

				const auto & marker1 = * itr1;
				const auto & marker2 = * itr2;

				for ( auto i = 0 ; i < marker1.size ( ) ; ++i ) {

					const auto & point1 = marker1[ i ];
					const auto & point2 = marker2[ i ];

					const auto x1 = cvRound ( point1.x );
					const auto y1 = cvRound ( point1.y );
					const auto x2 = cvRound ( point2.x );
					const auto y2 = cvRound ( point2.y );

					const auto point3d1 = keyframe1.GetPointImage ( ).at < cv::Vec3f > ( y1 , x1 );
					const auto point3d2 = keyframe2.GetPointImage ( ).at < cv::Vec3f > ( y2 , x2 );

					if ( std::isfinite ( point3d1 ( 0 ) ) and std::isfinite ( point3d2 ( 0 ) ) and
					     point3d1 != cv::Vec3f ( 0.0f , 0.0f , 0.0f ) and point3d2 != cv::Vec3f ( 0.0f , 0.0f , 0.0f ) ) {
						points1.push_back ( point3d1 );
						points2.push_back ( point3d2 );
					}
				}

				itr1 += 1;
				itr2 += 1;

			}

			else if ( itr1->id < itr2->id ) {
				itr1 += 1;
			}

			else {
				itr2 += 1;
			}
		}

		return std::make_pair ( points1 , points2 );

	}

}
