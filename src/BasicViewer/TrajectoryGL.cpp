//
// Created by LinKun on 10/12/15.
//

#include "BasicViewer/TrajectoryGL.h"

#include <Core/Utility.h>

namespace MapCreator {


	TrajectoryGL::TrajectoryGL ( QOpenGLFunctions_4_1_Core * GL , const KeyFrames & keyframes ) :
			PrimitiveGL ( GL ) ,
			keyframes_ ( keyframes ) { }


	void TrajectoryGL::Render ( ) {

		using namespace std;
		GLint transformation_matrix_uniform_location = GL->glGetUniformLocation ( shader_program_->programId ( ) ,
		                                                                          "transformation_matrix" );

		GL->glUniformMatrix4fv ( transformation_matrix_uniform_location , 1 , GL_FALSE ,
		                         & transformation_matrix_[ 0 ][ 0 ] );

		GL->glUseProgram ( shader_program_->programId ( ) );
		GL->glBindVertexArray ( vao_id_ );
		GL->glDrawArrays ( GL_LINE_STRIP , 0 , data_.size ( ) );
		GL->glBindVertexArray ( 0 );
	}

	void TrajectoryGL::SetupData ( ) {

		glm::vec3 color ( 1.0f , 0.0f , 1.0f );

		data_.clear ( );

		glm::mat4 accumulated_matrix;

		for ( const auto & frame : keyframes_ ) {

			if ( frame.IsUsed ( ) ) {

				accumulated_matrix = accumulated_matrix * frame.GetAlignmentMatrix ( );

				VertexGL point;

				glm::vec4 position = accumulated_matrix * glm::vec4 ( glm::vec3 ( 0.0f ) , 1.0f );

				point.position = glm::vec3 ( position.x , position.y , position.z );
				point.color    = color;

				data_.push_back ( point );
			}
		}

		assert ( !data_.empty ( ) );

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

	AnswerTrajectoryGL::AnswerTrajectoryGL ( QOpenGLFunctions_4_1_Core * GL , const KeyFrames & keyframes ) :
			TrajectoryGL ( GL , keyframes ) { }

	void AnswerTrajectoryGL::SetupData ( ) {

		glm::vec3 color ( 0.0f , 1.0f , 0.0f );

		data_.clear ( );

		glm::mat4 accumulated_matrix;

		for ( const auto & frame : keyframes_ ) {

			std::cout << frame.GetId ( ) << std::endl;
			std::cout << frame.GetAnswerAlignmentMatrix ( ) << std::endl;

			accumulated_matrix = accumulated_matrix * frame.GetAnswerAlignmentMatrix ( );

			VertexGL point;

			glm::vec4 position = accumulated_matrix * glm::vec4 ( glm::vec3 ( 0.0f ) , 1.0f );

			point.position = glm::vec3 ( position.x , position.y , position.z );
			point.color    = color;

			data_.push_back ( point );
		}

		assert ( !data_.empty ( ) );

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


}
