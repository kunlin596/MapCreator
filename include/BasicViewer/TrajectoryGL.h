//
// Created by LinKun on 10/12/15.
//

#ifndef MAPCREATOR_TRAJECTORYGL_H
#define MAPCREATOR_TRAJECTORYGL_H

#include "PrimitiveGL.h"

#include <SLAM/KeyFrame.h>

namespace MapCreator {

	class TrajectoryGL : public PrimitiveGL
	{
	public:

		TrajectoryGL ( QOpenGLFunctions_4_1_Core * GL ,
		               const KeyFrames & keyframes );

		void SetupData ( ) override;
		void Render ( ) override;

	protected:

		KeyFrames keyframes_;

	};


	class AnswerTrajectoryGL : public TrajectoryGL
	{
	public:

		AnswerTrajectoryGL ( QOpenGLFunctions_4_1_Core * GL , const KeyFrames & keyframes );

		void SetupData ( ) override;

	};
}

#endif //MAPCREATOR_TRAJECTORYGL_H
