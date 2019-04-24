//
// Created by LinKun on 10/6/15.
//

#ifndef MAPCREATOR_PRIMITIVEGL_H
#define MAPCREATOR_PRIMITIVEGL_H

#include <memory>

#include <QtGui/QOpenGLFunctions_4_1_Core>
#include <QtGui/QOpenGLFunctions_4_1_Compatibility>
#include <QtGui/QOpenGLShaderProgram>
#include <QObject>

#include <glm/glm.hpp>

namespace MapCreator {

	struct VertexGL
	{
		using Position = glm::vec3;
		using Color = glm::vec3;

		Position position;
		Color    color;
	};

	class PrimitiveGL
	{
	public:

		using VertexData = std::vector < VertexGL >;

		inline PrimitiveGL ( ) : GL ( nullptr ) { }

		explicit inline PrimitiveGL ( QOpenGLFunctions_4_1_Core * GL ) :
				GL ( GL ) ,
				gpu_data_is_new_ ( false ) {

			vbo_id_ = 0;
			vao_id_ = 0;
		}

		inline virtual ~PrimitiveGL ( ) { }

		inline virtual void Render ( ) = 0;

		inline virtual void SetupData ( ) = 0;

		inline void ReleaseData ( ) {

			GL->glDeleteBuffers ( 1 , & vbo_id_ );
			GL->glDeleteVertexArrays ( 1 , & vao_id_ );
		}

		inline bool GpuDataIsNew ( ) const { return gpu_data_is_new_; }

		inline GLuint GetVboId ( ) const { return vbo_id_; }

		inline GLuint GetVaoId ( ) const { return vao_id_; }

		inline GLuint GetIboId ( ) const { return ibo_id_; }

		inline QOpenGLShaderProgram * GetShaderProgram ( ) const { return shader_program_; }

		inline const glm::mat4 & GetTransformationMatrix ( ) const { return transformation_matrix_; }

		inline const glm::mat4 & GetModelMatrix ( ) const { return model_matrix_; }

		inline const glm::mat4 & GetViewMatrix ( ) const { return view_matrix_; }

		inline const glm::mat4 & GetProjectionMatrix ( ) const { return projection_matrix_; }

		inline const VertexData & GetVertexData ( ) const { return data_; }

		inline void SetVboId ( const GLuint vbo_id ) { vbo_id_ = vbo_id; }

		inline void SetVaoId ( const GLuint vao_id ) { vao_id_ = vao_id; }

		inline void SetIboId ( const GLuint ibo_id ) { ibo_id_ = ibo_id; }

		inline void SetShaderProgram ( QOpenGLShaderProgram * shader_program ) { shader_program_ = shader_program; }

		inline void SetTransformationMatrix ( const glm::mat4 & m ) { transformation_matrix_ = m; }

		inline void SetVertexData ( const VertexData & data ) { data_ = data; }

		inline void SetModelMatrix ( const glm::mat4 & model_matrix ) {

			model_matrix_ = model_matrix;
			UpdateTransformationMatrix ( );
		}

		inline void SetViewMatrix ( const glm::mat4 & view_matrix ) {

			view_matrix_ = view_matrix;
			UpdateTransformationMatrix ( );
		}

		inline void SetProjectionMatrix ( const glm::mat4 & projection_matrix ) {

			projection_matrix_ = projection_matrix;
			UpdateTransformationMatrix ( );
		}

		inline void UpdateTransformationMatrix ( ) {

			transformation_matrix_ = projection_matrix_ * view_matrix_ * model_matrix_;
		}

	protected:

		bool       gpu_data_is_new_;
		glm::mat4  model_matrix_;
		glm::mat4  view_matrix_;
		glm::mat4  projection_matrix_;
		glm::mat4  transformation_matrix_;
		GLuint     vbo_id_;
		GLuint     vao_id_;
		GLuint     ibo_id_;
		VertexData data_;

		QOpenGLShaderProgram      * shader_program_;
		QOpenGLFunctions_4_1_Core * GL;
	};
}


#endif //MAPCREATOR_PRIMITIVEGL_H
