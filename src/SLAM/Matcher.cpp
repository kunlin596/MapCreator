//
// Created by LinKun on 9/13/15.
//

#include "SLAM/Matcher.h"

namespace {

	using Feature = MapCreator::Feature;
	using Match = MapCreator::Matcher::Match;
	using Matches = MapCreator::Matcher::Matches;

	template < class MatcherType >
	Matches CreateMatches ( const Feature & feature1 , const Feature & feature2 , bool cross_check ) {

		typedef std::vector < cv::DMatch > DMatches;

		Matches matches;

		if ( cross_check ) {

			MatcherType matcher;
			DMatches    dmatches1;
			DMatches    dmatches2;

			matcher.match ( feature1.GetDescriptors ( ) , feature2.GetDescriptors ( ) , dmatches1 );
			matcher.match ( feature2.GetDescriptors ( ) , feature1.GetDescriptors ( ) , dmatches2 );

			// クロスチェックを行い、両方から一番近い時だけ採用する
			for ( const cv::DMatch & dmatch1 : dmatches1 ) {
				const cv::DMatch & dmatch2 = dmatches2[ dmatch1.trainIdx ];

				if ( dmatch1.queryIdx == dmatch2.trainIdx ) {
					matches.push_back ( Match ( dmatch1.queryIdx , dmatch1.trainIdx ) );
				}
			}
		}
		else {

			MatcherType matcher;
			DMatches    dmatches;

			matcher.match ( feature1.GetDescriptors ( ) , feature2.GetDescriptors ( ) , dmatches );

			// 距離が平均以下の近いものだけを採用する
			const float threshold = std::accumulate ( dmatches.begin ( ) , dmatches.end ( ) , 0.0f ,
			                                          [ ] ( float sum , const cv::DMatch & match ) {
				                                          return sum + match.distance;
			                                          } ) / dmatches.size ( );

			for ( const cv::DMatch & dmatch : dmatches ) {
				if ( dmatch.distance <= threshold ) {
					matches.push_back ( Match ( dmatch.queryIdx , dmatch.trainIdx ) );
				}
			}
		}

		return matches;
	}

}    // namespace


namespace MapCreator {

	Matcher::Matcher ( const Feature & feature1 , const Feature & feature2 , bool cross_check ) {

		matches_ = CreateMatches ( feature1 , feature2 , cross_check );
	}


	Matcher::Matcher ( const Feature & feature1 , const Feature & feature2 , const PointImage & point_image1 ,
	                   const PointImage & point_image2 , bool cross_check ) {

		matches_ = CreateValidMatches ( feature1 , feature2 , point_image1 , point_image2 , cross_check );
	}


	Matcher::Matcher ( const Matches & matches )
			: matches_ ( matches ) { }


	Matcher::Matcher ( ) { }


	Matcher::~Matcher ( ) { }


	Matcher::Matches Matcher::CreateMatches ( const Feature & feature1 , const Feature & feature2 , bool cross_check ) const {

		Matches matches;

		if ( !feature1.GetKeyPoints ( ).empty ( ) && !feature2.GetKeyPoints ( ).empty ( ) &&
		     feature1.GetType ( ) == feature2.GetType ( ) ) {
			switch ( feature1.GetType ( ) ) {
#ifdef ENABLE_OPENCV_CONTRIB
				case Feature::kTypeSIFT:
				case Feature::kTypeSURF:
					matches = ::CreateMatches < cv::FlannBasedMatcher > ( feature1 , feature2 , cross_check );
					break;
#endif
				case Feature::kTypeORB:
#ifdef ENABLE_OPENCV_CONTRIB
				case Feature::kTypeFREAK:
#endif
                    matches = ::CreateMatches < cv::BFMatcher >
					          ( feature1 , feature2 , cross_check );
					break;
				case Feature::kTypeUnknown:
					break;
				default:
					break;
			}
		}

		return matches;
	}


	Matcher::Matches Matcher::CreateValidMatches ( const Feature & feature1 , const Feature & feature2 ,
	                                               const PointImage & point_image1 , const PointImage & point_image2 ,
	                                               bool cross_check ) const {

		const Matches matches = CreateMatches ( feature1 , feature2 , cross_check );

		Matches valid_matches;

		for ( const Match & match : matches ) {
			const cv::Point2f & pt1 = feature1.GetKeyPoints ( )[ match.first ].pt;
			const cv::Point2f & pt2 = feature2.GetKeyPoints ( )[ match.second ].pt;

			const auto & point1 = point_image1 ( cvRound ( pt1.y ) , cvRound ( pt1.x ) );
			const auto & point2 = point_image2 ( cvRound ( pt2.y ) , cvRound ( pt2.x ) );

			if ( std::isfinite ( point1 ( 0 ) ) && std::isfinite ( point2 ( 0 ) ) ) {
				valid_matches.push_back ( match );
			}
		}

		return valid_matches;
	}

}    // a



