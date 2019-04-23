//
// Created by LinKun on 10/11/15.
//

#ifndef NIS_KEYFRAMEGL_H
#define NIS_KEYFRAMEGL_H

#include "BasicViewer/PrimitiveGL.h"
#include <SLAM/KeyFrame.h>
#include <memory>

namespace MapCreator {

	/***
	 *  This is the wrapper of KeyFrame data for OpenGL rendering
	 */
	class KeyFrameGL : public PrimitiveGL
	{
	public:

		KeyFrameGL ( QOpenGLFunctions_4_1_Core * GL ,
		             const KeyFrame & keyframe ,
		             const int & point_cloud_density_step = 5 );

		~KeyFrameGL ( ) { /*ReleaseData ( );*/ }

		void Render ( ) override;

		void SetupData ( ) override;

		inline void SetPointDensityStep ( const int & step ) {

			point_cloud_density_step_ = step;
			gpu_data_is_new_          = false;
		}
		std::string GetName ( ) const { return keyframe_.GetName ( ); }
		const glm::mat4 & GetAlignmentMatrix ( ) const { return keyframe_.GetAlignmentMatrix ( ); }
		const glm::mat4 & GetAnswerAlignmentMatrix ( ) const { return keyframe_.GetAnswerAlignmentMatrix ( ); }
		bool IsUsed ( ) const { return keyframe_.IsUsed ( ); }

	private:

		int point_cloud_density_step_;

		KeyFrame keyframe_;
	};

	using KeyFramesGL = std::vector < KeyFrameGL >;

}


#endif //NIS_KEYFRAMEGL_H
