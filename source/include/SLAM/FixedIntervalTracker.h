//
// Created by lk on 15/12/09.
//

#ifndef NIS_FIXEDINTERVALTRACKER_H
#define NIS_FIXEDINTERVALTRACKER_H

#include "SLAM/Tracker.h"

namespace NiS {

	class FixedIntervalTracker : public AbstractTracker
	{
	Q_OBJECT

	public:

		void Initialize ( ) override {

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

		bool Update ( ) override {

			emit SetProgressValue ( iterator2_->GetId ( ) );

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

		void ComputeNext ( ) override {

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
			                                                                                           * converter_ptr_ ,
			                                                                                           world_points1 ,
			                                                                                           world_points2 );

			auto error_after_global_optimization = LevenbergMarquardt::GetError ( local_transformation_matrix_after_global_optimization ,
			                                                                      * converter_ptr_ ,
			                                                                      world_points1 ,
			                                                                      world_points2 );

			glm::mat4 m = Convert_OpenCV_Matx44f_To_GLM_mat4 ( local_transformation_matrix_after_global_optimization );

			iterator2_->SetAlignmentMatrix ( m );

			message_ = QString ( " + Computed : %1 - %2. (using Levenberg Marquardt, total error : %3.)" )
					.arg ( QString::number ( iterator2_->GetId ( ) ) )
					.arg ( QString::number ( iterator1_->GetId ( ) ) )
					.arg ( error_after_global_optimization );

			iterator2_->SetUsed ( true );
		}

	private:

		int offset_;

	};
}


#endif //NIS_FIXEDINTERVALTRACKER_H
