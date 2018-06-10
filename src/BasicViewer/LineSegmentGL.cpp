//
// Created by LinKun on 11/24/15.
//

#include "BasicViewer/LineSegmentGL.h"

namespace NiS {

	LineSegmentGL::LineSegmentGL ( QOpenGLFunctions_4_1_Core * GL ,
	                               const glm::mat4 transformation_matrix1 , // for frame1
	                               const glm::mat4 transformation_matrix2 , // for frame2
	                               const glm::vec3 begin_color ,
	                               const glm::vec3 end_color ) :
			PrimitiveGL ( GL ) ,
			begin_color_ ( begin_color ) ,
			end_color_ ( end_color ) {

		begin_position_ = glm::vec3 ( transformation_matrix1 * glm::vec4 ( glm::vec3 ( ) , 1.0f ) );
		end_position_   = glm::vec3 ( transformation_matrix2 * glm::vec4 ( glm::vec3 ( ) , 1.0f ) );

//		std::cout << be

	}

	void LineSegmentGL::SetupData ( ) {

		data_.clear ( );

		VertexGL begin_vertex;
		VertexGL end_vertex;

		begin_vertex.position = begin_position_;
		end_vertex.position   = end_position_;

		begin_vertex.color = begin_color_;
		end_vertex.color   = end_color_;

		data_.push_back ( begin_vertex );
		data_.push_back ( end_vertex );

		GL->glGenVertexArrays ( 1 , & vao_id_ );
		GL->glGenBuffers ( 1 , & vbo_id_ );

		GL->glBindVertexArray ( vao_id_ );
		GL->glBindBuffer ( GL_ARRAY_BUFFER , vbo_id_ );
		GL->glBufferData ( GL_ARRAY_BUFFER ,
		                   sizeof ( VertexGL ) * data_.size ( ) ,
		                   reinterpret_cast<float *> (data_.data ( )) ,
		                   GL_STATIC_DRAW );

		GL->glEnableVertexAttribArray ( 0 );
		GL->glEnableVertexAttribArray ( 1 );

		GL->glVertexAttribPointer ( 0 , 3 , GL_FLOAT , GL_FALSE , sizeof ( VertexGL ) , nullptr );
		GL->glVertexAttribPointer ( 1 , 3 , GL_FLOAT , GL_FALSE , sizeof ( VertexGL ) ,
		                            ( void * ) ( sizeof ( VertexGL::Position ) ) );

		GL->glBindBuffer ( GL_ARRAY_BUFFER , 0 );
		GL->glBindVertexArray ( 0 );

	}

	void LineSegmentGL::Render ( ) {

		using namespace std;
		GLint transformation_matrix_uniform_location = GL->glGetUniformLocation ( shader_program_->programId ( ) ,
		                                                                          "transformation_matrix" );

		GL->glUniformMatrix4fv ( transformation_matrix_uniform_location , 1 , GL_FALSE ,
		                         & transformation_matrix_[ 0 ][ 0 ] );

		GL->glUseProgram ( shader_program_->programId ( ) );
		GL->glBindVertexArray ( vao_id_ );
		GL->glDrawArrays ( GL_LINES , 0 , data_.size ( ) );
		GL->glBindVertexArray ( 0 );

	}

}
