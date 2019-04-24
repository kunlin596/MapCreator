//
// Created by LinKun on 10/28/15.
//

#ifndef MAPCREATOR_POINTPAIRGL_H
#define MAPCREATOR_POINTPAIRGL_H

#include "BasicViewer/PrimitiveGL.h"
#include <SLAM/KeyFrame.h>

namespace MapCreator {

	class PointPairGL : public PrimitiveGL
	{
	public:

		PointPairGL ( QOpenGLFunctions_4_1_Core * GL , const std::pair < glm::vec3 , glm::vec3 > & point_pair );

		void SetupData ( ) override;

		void Render ( ) override;

	protected:

		KeyFrames                           keyframes_;
		std::pair < glm::vec3 , glm::vec3 > point_pair_;

	};

	class AnswerPointPairGL : public PointPairGL
	{
	public:

		AnswerPointPairGL ( QOpenGLFunctions_4_1_Core * GL , const std::pair < glm::vec3 , glm::vec3 > & point_pair );

		void SetupData ( ) override;

	};

}
#endif //MAPCREATOR_POINTPAIRGL_H
