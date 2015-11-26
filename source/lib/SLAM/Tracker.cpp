//
// Created by LinKun on 10/9/15.
//

#include "SLAM/Matcher.h"
#include "SLAM/Tracker.h"
#include "SLAM/Transformation.h"
#include "SLAM/GlobalOptimization.h"

#include <Core/Utility.h>

#include <boost/tuple/tuple.hpp>

namespace NiS {

	CorrespondingPointsPair CreateCorrespondingPointsPair ( const NiS::KeyFrame & key_frame1 ,
	                                                        const NiS::KeyFrame & key_frame2 ) {

		std::vector < cv::Point3f > points1;
		std::vector < cv::Point3f > points2;

		const auto & feature1 = key_frame1.GetFeature ( );
		const auto & feature2 = key_frame2.GetFeature ( );

		const auto & image1 = key_frame1.GetPointImage ( );
		const auto & image2 = key_frame2.GetPointImage ( );

		assert ( !key_frame1.GetFeature ( ).GetKeyPoints ( ).empty ( ) );

		const NiS::Matcher matcher ( feature1 , feature2 , true );
		const auto & matches = matcher.GetMatches ( );

		assert ( !matches.empty ( ) );

		std::cout << "Creating point pairs of " << key_frame2.GetId ( ) << " - " << key_frame1.GetId ( ) << " : Matches size : " <<
		matches.size ( ) <<
		std::endl;

		for ( const auto & match : matches ) {

			const auto & key_point1 = feature1.GetKeyPoints ( )[ match.first ].pt;
			const auto & key_point2 = feature2.GetKeyPoints ( )[ match.second ].pt;

			const cv::Point3f pt1 = image1 ( cvRound ( key_point1.y ) , cvRound ( key_point1.x ) );
			const cv::Point3f pt2 = image2 ( cvRound ( key_point2.y ) , cvRound ( key_point2.x ) );

			if ( std::isfinite ( pt1.x ) and std::isfinite ( pt2.x ) and
			     ( pt1 != cv::Point3f ( 0.0f ) ) and ( pt2 != cv::Point3f ( 0.0f ) ) ) {

				points1.push_back ( pt1 );
				points2.push_back ( pt2 );
			}
		}

		return std::make_pair ( points1 , points2 );
	}

	bool ValidateInliersDistribution ( const InlierPoints & inliers ,
	                                   int threshold_inliers_number ,
	                                   float threshold_1st_principal_component_contribution ,
	                                   float threshold_1st_principal_component_variance ,
	                                   float threshold_2nd_principal_component_variance ,
	                                   float threshold_3rd_principal_component_variance ,
	                                   QString & error_msg ) {

		// Performing PCA using OpenCV

		cv::Mat inliers_mat ( ( int ) inliers.size ( ) , 3 , CV_32FC1 , ( void * ) inliers.data ( ) );
		cv::PCA pca ( inliers_mat , cv::Mat ( ) , CV_PCA_DATA_AS_ROW );

		float _1st_principal_component_variance = pca.eigenvalues.at < float > ( 0 , 0 );
		float _2nd_principal_component_variance = pca.eigenvalues.at < float > ( 1 , 0 );
		float _3rd_principal_component_variance = pca.eigenvalues.at < float > ( 2 , 0 );

		float _1st_principal_component_contribution = _1st_principal_component_variance /
		                                              ( _1st_principal_component_variance +
		                                                _2nd_principal_component_variance +
		                                                _3rd_principal_component_variance );

		// Checking thresholds
		if ( inliers.size ( ) < threshold_inliers_number ) {
			error_msg = QString ( "#Inliers < TH(%1)" ).arg ( threshold_inliers_number );
			return false;
		}
		if ( _1st_principal_component_variance < threshold_1st_principal_component_variance ) {
			error_msg = QString ( "1st PC variance < TH(%1)" ).arg ( threshold_1st_principal_component_variance );
			return false;
		}
		if ( _2nd_principal_component_variance < threshold_2nd_principal_component_variance ) {
			error_msg = QString ( "2nd PC variance < TH(%1)" ).arg ( threshold_2nd_principal_component_variance );
			return false;
		}
		if ( _3rd_principal_component_variance < threshold_3rd_principal_component_variance ) {
			error_msg = QString ( "3rd PC variance < TH(%1)" ).arg ( threshold_3rd_principal_component_variance );
			return false;
		}
		if ( _1st_principal_component_contribution > threshold_1st_principal_component_contribution ) {
			error_msg = QString ( "1st PC contribution > TH(%1)" ).arg ( threshold_1st_principal_component_contribution );
			return false;
		}

		error_msg = "Passed";

		return true;
	}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	template < > void Tracker < TrackingType::OneByOne >::Initialize ( ) {

		if ( auto keyframes = keyframes_ptr_.lock ( ) ) {
			for ( auto & keyframe : * keyframes ) {
				keyframe.SetAlignmentMatrix ( std::move ( glm::mat4 ( ) ) );
				keyframe.SetUsed ( false );
			}
			iterator1_ = keyframes->begin ( );
			iterator2_ = iterator1_;
		}
	}
	template < > void Tracker < TrackingType::FixedFrameCount >::Initialize ( ) {

		if ( auto keyframes = keyframes_ptr_.lock ( ) ) {

			for ( auto & keyframe :* keyframes ) {
				keyframe.SetAlignmentMatrix ( std::move ( glm::mat4 ( ) ) );
				keyframe.SetUsed ( false );
			}

			iterator1_ = keyframes->begin ( );
			iterator2_ = iterator1_;
			offset_    = keyframes->size ( ) / ( options_.options_fixed_frame_count.frame_count - 1 ) + 1;
		}

	}
	template < > void Tracker < TrackingType::PcaKeyFrame >::Initialize ( ) {

		if ( auto keyframes = keyframes_ptr_.lock ( ) ) {

			for ( auto & keyframe : * keyframes ) {
				keyframe.SetAlignmentMatrix ( std::move ( glm::mat4 ( ) ) );
				keyframe.SetUsed ( false );
			}

			iterator1_ = keyframes->begin ( );
			iterator2_ = iterator1_;

			// For initial inliers computation in order to to compute next.
			CorrespondingPointsPair corresponding_points_pair = CreateCorrespondingPointsPair ( * iterator1_ , * iterator2_ );

			boost::tie ( inliers2_ , inliers1_ ) = ComputeInliers ( corresponding_points_pair.second ,
			                                                        corresponding_points_pair.first ,
			                                                        options_.options_pca_keyframe.num_ransac_iteration ,
			                                                        options_.options_pca_keyframe.threshold_outlier ,
			                                                        options_.options_pca_keyframe.threshold_inlier );
		}
	}

	template < > bool Tracker < TrackingType::OneByOne >::Update ( ) {

		if ( auto keyframes = keyframes_ptr_.lock ( ) ) {

			if ( iterator1_ == iterator2_ ) {
				std::advance ( iterator2_ , 1 );
			} else {
				std::advance ( iterator1_ , 1 );
				std::advance ( iterator2_ , 1 );
			}

			std::cout << "Updated to : " << iterator2_->GetId ( ) << " - " << iterator1_->GetId ( ) << std::endl;

			return ( iterator2_ != keyframes->end ( ) );
		}
	}
	template < > bool Tracker < TrackingType::FixedFrameCount >::Update ( ) {

		if ( auto keyframes = keyframes_ptr_.lock ( ) ) {
			// iter2 has reached the end of data array, no need to compute anymore.
			if ( iterator2_ == keyframes->end ( ) - 1 ) {
				return false;
			}

			else if ( iterator1_ == iterator2_ ) {
				std::advance ( iterator2_ , offset_ );
			}

			else if ( iterator2_->GetId ( ) + offset_ < keyframes->size ( ) ) {
				std::advance ( iterator1_ , offset_ );
				std::advance ( iterator2_ , offset_ );
			}

			else if ( iterator2_->GetId ( ) + offset_ >= keyframes->size ( ) ) {
				iterator2_ = keyframes->end ( ) - 1;
				std::advance ( iterator1_ , offset_ );
			}

			return true;
		}
	}
	template < > bool Tracker < TrackingType::PcaKeyFrame >::Update ( ) {

		if ( auto keyframes = keyframes_ptr_.lock ( ) ) {

			if ( iterator2_ == keyframes->end ( ) )
				return false;

			do {

				CorrespondingPointsPair corresponding_points_pair = CreateCorrespondingPointsPair ( * iterator1_ , * iterator2_ );

				boost::tie ( inliers2_ , inliers1_ ) = ComputeInliers ( corresponding_points_pair.second ,
				                                                        corresponding_points_pair.first ,
				                                                        options_.options_pca_keyframe.num_ransac_iteration ,
				                                                        options_.options_pca_keyframe.threshold_outlier ,
				                                                        options_.options_pca_keyframe.threshold_inlier );

				if ( iterator2_ == keyframes->end ( ) - 1 ) {
					return true;
				}

				QString error_msg;

				bool is_valid = ValidateInliersDistribution ( inliers2_ ,
				                                              options_.options_pca_keyframe.num_inliers ,
				                                              options_.options_pca_keyframe.threshold_1st_component_contribution ,
				                                              options_.options_pca_keyframe.threshold_1st_component_variance ,
				                                              options_.options_pca_keyframe.threshold_2nd_component_variance ,
				                                              options_.options_pca_keyframe.threshold_3rd_component_variance ,
				                                              error_msg );

				message_ = QString ( " - Checking %1 - %2 : %3" )
						.arg ( QString::number ( iterator2_->GetId ( ) ) )
						.arg ( QString::number ( iterator1_->GetId ( ) ) )
						.arg ( error_msg );

				if ( is_valid ) {
					++iterator2_;
				}
				else {

					if ( iterator1_ + 1 != iterator2_ ) { --iterator2_; }
					// if the iterator2 != iterator1, which means it's safe to move backwards by one.
					// otherwise, just use these 2 frame to compute bruntly, since there's nowhere to go
					return true;

				}

			} while ( true );

		}
	}

	template < > void Tracker < TrackingType::OneByOne >::ComputeNext ( ) {

		CorrespondingPointsPair corresponding_points_pair = CreateCorrespondingPointsPair ( * iterator1_ , * iterator2_ );

		assert ( !corresponding_points_pair.first.empty ( ) and !corresponding_points_pair.second.empty ( ) );

		// 2 -> 1
		auto local_transformation_matrix = ComputeTransformationMatrix ( corresponding_points_pair.second ,
		                                                                 corresponding_points_pair.first ,
		                                                                 options_.options_one_by_one.num_ransac_iteration ,
		                                                                 options_.options_one_by_one.threshold_outlier ,
		                                                                 options_.options_one_by_one.threshold_inlier );

		const auto & world_points1 = corresponding_points_pair.first;
		const auto & world_points2 = corresponding_points_pair.second;

		auto local_transformation_matrix_after_global_optimization = LevenbergMarquardt::Compute ( local_transformation_matrix ,
		                                                                                           * converter_pointer_ ,
		                                                                                           world_points1 ,
		                                                                                           world_points2 );

		auto error_after_global_optimization = LevenbergMarquardt::GetError ( local_transformation_matrix_after_global_optimization ,
		                                                                      * converter_pointer_ ,
		                                                                      world_points1 ,
		                                                                      world_points2 );

		std::cout << "Before LM : " << local_transformation_matrix << std::endl;
		std::cout << "After LM : " << local_transformation_matrix_after_global_optimization << std::endl;

		glm::mat4 m = Convert_OpenCV_Matx44f_To_GLM_mat4 ( local_transformation_matrix_after_global_optimization );

		iterator2_->SetAlignmentMatrix ( m );

		message_ = QString ( " + Computed : %1 - %2. #inliers : %3. (using Levenberg Marquardt, total error : %4.)" )
				.arg ( QString::number ( iterator2_->GetId ( ) ) )
				.arg ( QString::number ( iterator1_->GetId ( ) ) )
				.arg ( world_points1.size ( ) )
				.arg ( error_after_global_optimization );

		iterator2_->SetUsed ( true );
	}
	template < > void Tracker < TrackingType::FixedFrameCount >::ComputeNext ( ) {

		auto corresponding_points_pair = CreateCorrespondingPointsPair ( * iterator1_ , * iterator2_ );

		assert ( !corresponding_points_pair.first.empty ( ) and !corresponding_points_pair.second.empty ( ) );

		// 2 -> 1
		auto local_transformation_matrix = ComputeTransformationMatrix ( corresponding_points_pair.second ,
		                                                                 corresponding_points_pair.first ,
		                                                                 options_.options_fixed_frame_count.num_ransac_iteration ,
		                                                                 options_.options_fixed_frame_count.threshold_outlier ,
		                                                                 options_.options_fixed_frame_count.threshold_inlier );

		const auto & world_points1 = corresponding_points_pair.first;
		const auto & world_points2 = corresponding_points_pair.second;

		auto local_transformation_matrix_after_global_optimization = LevenbergMarquardt::Compute ( local_transformation_matrix ,
		                                                                                           * converter_pointer_ ,
		                                                                                           world_points1 ,
		                                                                                           world_points2 );

		auto error_after_global_optimization = LevenbergMarquardt::GetError ( local_transformation_matrix_after_global_optimization ,
		                                                                      * converter_pointer_ ,
		                                                                      world_points1 ,
		                                                                      world_points2 );

		glm::mat4 m = Convert_OpenCV_Matx44f_To_GLM_mat4 ( local_transformation_matrix_after_global_optimization );

		iterator2_->SetAlignmentMatrix ( m );

		message_ = QString ( " + Computed : %1 - %2. (using Levenberg Marquardt, total error : %3.)" )
				.arg ( QString::number ( iterator2_->GetId ( ) ) )
				.arg ( QString::number ( iterator1_->GetId ( ) ) )
				.arg ( error_after_global_optimization );

		iterator2_->SetUsed ( true );

//		return * iterator2_;
	}
	template < > void Tracker < TrackingType::PcaKeyFrame >::ComputeNext ( ) {

		const auto local_transformation_matrix = ComputeTransformationMatrix ( inliers2_ , inliers1_ );
		const auto & world_points1 = inliers1_;
		const auto & world_points2 = inliers2_;

		auto local_transformation_matrix_after_global_optimization = LevenbergMarquardt::Compute ( local_transformation_matrix ,
		                                                                                           * converter_pointer_ ,
		                                                                                           world_points1 ,
		                                                                                           world_points2 );

		auto error_after_global_optimization = LevenbergMarquardt::GetError ( local_transformation_matrix_after_global_optimization ,
		                                                                      * converter_pointer_ ,
		                                                                      world_points1 ,
		                                                                      world_points2 );

		std::cout << "Before LM : " << local_transformation_matrix << std::endl;
		std::cout << "After LM : " << local_transformation_matrix_after_global_optimization << std::endl;

		auto m = Convert_OpenCV_Matx44f_To_GLM_mat4 ( local_transformation_matrix_after_global_optimization );

		iterator2_->SetAlignmentMatrix ( m );

		message_ = QString ( " + Computed : %1 - %2. #inliers : %3. (using Levenberg Marquardt, total error : %4.)" )
				.arg ( QString::number ( iterator2_->GetId ( ) ) )
				.arg ( QString::number ( iterator1_->GetId ( ) ) )
				.arg ( world_points1.size ( ) )
				.arg ( error_after_global_optimization );

		auto current_keyframe_itr = iterator2_;

		iterator1_ = iterator2_;     // assign iterator2 to iterator1 to make the frame the current keyframe
		iterator2_ = iterator1_ + 1; // move iterator 1 step forward to prepare the inlier distribution validation

		current_keyframe_itr->SetUsed ( true );

//		return res;
	}

}