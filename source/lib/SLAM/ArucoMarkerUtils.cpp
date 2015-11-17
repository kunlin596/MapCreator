//
// Created by LinKun on 11/15/15.
//

#include "SLAM/ArucoMarkerUtils.h"
#include <algorithm>


namespace {

	void SwapMarker ( aruco::Marker & marker1 , aruco::Marker & marker2 ) {

		auto temp = marker2;
		marker2 = marker1;
		marker1 = temp;
	}

	void SortMarkers ( std::vector < aruco::Marker > & markers_set ) {

		for ( auto i = 0 ; i < markers_set.size ( ) ; ++i ) {
			for ( auto j = i + 1 ; j < markers_set.size ( ) ; ++j ) {

				if ( markers_set[ j ].id < markers_set[ i ].id ) {
//					swap ( markers_set[ i ] , markers_set[ j ] );

					std::cout << "before swap : " << markers_set[ i ].id << ", " << markers_set[ j ].id << std::endl;

					std::swap ( markers_set[ i ] , markers_set[ j ] );

					std::cout << "after swap : " << markers_set[ i ].id << ", " << markers_set[ j ].id << std::endl;

				}
			}
		}
	}


}


namespace NiS {

	CorrespondingPointsPair ArucoMarkerUtils::CreatePoints ( Markers & markers_set_1 , Markers & markers_set_2 ,
	                                                         KeyFrame & keyframe1 , KeyFrame & keyframe2 ) {

		using namespace std;

		// Markers are sorted, thus no need to sort again.
		// Anyway the std::sort doesn't work.
//		auto sort_by_marker_id = [ ] ( const aruco::Marker & marker1 , const aruco::Marker & marker2 ) -> bool { return marker1.id < marker2.id; };
//		std::sort ( markers_set_1.begin ( ) , markers_set_1.end ( ) , sort_by_marker_id );


		std::cout << "#  " << keyframe1.GetId ( ) << " - " << keyframe2.GetId ( ) << std::endl;

//		SortMarkers ( markers_set_1 );
//		SortMarkers ( markers_set_2 );

		auto itr1 = markers_set_1.begin ( );
		auto itr2 = markers_set_2.begin ( );

		Points points1;
		Points points2;

		std::cout << "markers_set1 : ";
		for ( auto i = 0 ; i < markers_set_1.size ( ) ; ++i ) {
			std::cout << " " << markers_set_1[ i ].id << " ";
		}
		std::cout << " | markers_set2 : ";
		for ( auto i = 0 ; i < markers_set_2.size ( ) ; ++i ) {
			std::cout << " " << markers_set_2[ i ].id << " ";
		}
		std::cout << std::endl;

		while ( itr1 != markers_set_1.end ( ) and itr2 != markers_set_2.end ( ) ) {

			std::cout << "Checking : " << itr1->id << ", " << itr2->id << std::endl;

			if ( itr1->id == itr2->id ) {

//				ColorImage color_image1 = keyframe1.GetColorImage ( );
//				ColorImage color_image2 = keyframe1.GetColorImage ( );
//
//				itr1->draw ( color_image1 , cv::Scalar ( 0 , 0 , 255 ) , 2 );
//				itr2->draw ( color_image2 , cv::Scalar ( 0 , 0 , 255 ) , 2 );
//
//				keyframe1.SetColorImage ( color_image1 );
//				keyframe2.SetColorImage ( color_image2 );

				const auto & marker1 = * itr1;
				const auto & marker2 = * itr2;

				for ( int i = 0 ; i < marker1.size ( ) ; ++i ) {

					const auto & point1 = marker1[ i ];
					const auto & point2 = marker2[ i ];

					const auto x1 = cvRound ( point1.x );
					const auto y1 = cvRound ( point1.y );
					const auto x2 = cvRound ( point2.x );
					const auto y2 = cvRound ( point2.y );

					const auto point3d1 = keyframe1.GetPointImage ( ).at < cv::Vec3f > ( y1 , x1 );
					const auto point3d2 = keyframe2.GetPointImage ( ).at < cv::Vec3f > ( y2 , x2 );

					if ( std::isfinite ( point3d1 ( 0 ) ) and std::isfinite ( point3d2 ( 0 ) ) and
					     point3d1 != cv::Vec3f ( 0.0f , 0.0f , 0.0f ) and point3d2 != cv::Vec3f ( 0.0f , 0.0f , 0.0f ) and
					     std::abs ( point3d1 ( 2 ) ) > 0.3f and std::abs ( point3d1 ( 2 ) ) > 0.3f ) {
						points1.push_back ( point3d1 );
						points2.push_back ( point3d2 );
					}

//					std::cout << point3d1 << ", " << point3d2 << std::endl;
				}

				itr1 += 1;
				itr2 += 1;

			}

			else if ( itr1->id < itr2->id ) {
//				 std::cout << itr1->id << " < " << itr2->id << std::endl;
				itr1 += 1;


			}

			else {
//				 std::cout << itr1->id << " > " << itr2->id << std::endl;
				itr2 += 1;

			}
		}

		return std::make_pair ( points1 , points2 );

	}

}
