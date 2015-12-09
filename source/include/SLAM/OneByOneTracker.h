//
// Created by lk on 15/12/09.
//

#ifndef NIS_ONEBYONETRACKER_H
#define NIS_ONEBYONETRACKER_H

#include "SLAM/Tracker.h"

namespace NiS {

	class OneByOneTracker : public AbstractTracker
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
			}
		}
		bool Update ( ) override {

			if ( auto keyframes = keyframes_ptr_.lock ( ) ) {

				emit SetProgressValue ( iterator2_->GetId ( ) );

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
		void ComputeNext ( ) override {

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
			                                                                                           * converter_ptr_ ,
			                                                                                           world_points1 ,
			                                                                                           world_points2 );

			auto error_after_global_optimization = LevenbergMarquardt::GetError ( local_transformation_matrix_after_global_optimization ,
			                                                                      * converter_ptr_ ,
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

	};
}

#endif //NIS_ONEBYONETRACKER_H
