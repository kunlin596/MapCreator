//
// Created by lk on 15/12/09.
//

#ifndef NIS_PCAKEYFRAMETRACKER_H
#define NIS_PCAKEYFRAMETRACKER_H

#include "SLAM/Tracker.h"

namespace {

	bool ValidateInliersDistribution ( const NiS::InlierPoints & inliers ,
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
}

namespace NiS {

	class PcaKeyFrameTracker : public AbstractTracker
	{
	Q_OBJECT

	public:

		void Initialize ( ) override {

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
		bool Update ( ) override {

			if ( auto keyframes = keyframes_ptr_.lock ( ) ) {

				if ( iterator2_ == keyframes->end ( ) )
					return false;

				do {

					emit SetProgressValue ( iterator2_->GetId ( ) );

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
		void ComputeNext ( ) override {

			const auto local_transformation_matrix = ComputeTransformationMatrix ( inliers2_ , inliers1_ );
			const auto & world_points1 = inliers1_;
			const auto & world_points2 = inliers2_;

			auto local_transformation_matrix_after_global_optimization = LevenbergMarquardt::Compute ( local_transformation_matrix ,
			                                                                                           * converter_ptr_ ,
			                                                                                           world_points1 ,
			                                                                                           world_points2 );

			auto error_after_global_optimization = LevenbergMarquardt::GetError ( local_transformation_matrix_after_global_optimization ,
			                                                                      * converter_ptr_ ,
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

		}

	};


}

#endif //NIS_PCAKEYFRAMETRACKER_H
