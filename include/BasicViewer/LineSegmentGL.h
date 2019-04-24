//
// Created by LinKun on 11/24/15.
//

#ifndef MAPCREATOR_LINESEGMENTGL_H
#define MAPCREATOR_LINESEGMENTGL_H


#include "PrimitiveGL.h"

#include <glm/glm.hpp>

namespace MapCreator {

	class LineSegmentGL : public PrimitiveGL
	{
	public:

		LineSegmentGL ( QOpenGLFunctions_4_1_Core * GL ,
		                const glm::mat4 transformation_matrix1 , // for frame1
		                const glm::mat4 transformation_matrix2 , // for frame2
		                const glm::vec3 begin_color = glm::vec3 ( 1.0f , 0.0f , 1.0f ) ,
		                const glm::vec3 end_color = glm::vec3 ( 1.0f , 0.0f , 1.0f ) );

		void SetupData ( ) override;
		void Render ( ) override;

	private:

		glm::vec3 begin_position_;
		glm::vec3 end_position_;
		glm::vec3 begin_color_;
		glm::vec3 end_color_;

	};

}


#endif //MAPCREATOR_LINESEGMENTGL_H
