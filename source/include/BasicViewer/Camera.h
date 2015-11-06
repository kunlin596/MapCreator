//
// Created by LinKun on 10/3/15.
//

#ifndef QT5GLVIEWER_CAMERA_H
#define QT5GLVIEWER_CAMERA_H

#include <glm/glm.hpp>

namespace NiS {

	class Camera
	{

	public:

		enum class Move
		{
			LookUp ,
			LookDown ,
			LookLeft ,
			LookRight ,
			StrafeForward ,
			StrafeBackward ,
			StrafeLeft ,
			StrafeRight ,
			StrafeUp ,
			StrafeDown ,
		};

		Camera ( );

		~Camera ( );

		glm::mat4 ViewMatrix ( ) const;

		inline const glm::vec3 & GetPosition ( ) const { return camera_position_; }
		inline const glm::vec3 & GetLookAt ( ) const { return look_at_; }
		inline const glm::vec3 & GetUp ( ) const { return up_; }
		
		inline glm::vec3 GetHorizontalStrifeMatrix ( ) { return glm::cross ( look_at_ , up_ ); }

		inline void SetView ( const glm::mat4 & m ) { view_mat_ = m; }

		void SetView ( glm::vec3 eye , glm::vec3 center , glm::vec3 up );

		void SetView ( float eye_x , float eye_y , float eye_z ,
		               float center_x , float center_y , float center_z ,
		               float up_x , float up_y , float up_z );

		void MouseUpdate ( const glm::vec2 & mouse_delta );

		void KeyUpdate ( const Move & move , const float distance );

		void UpdateCameraPosition ( float x , float y , float z );
		void UpdateCameraPosition ( const glm::vec3 & new_position );
		void UpdateCameraLookAt ( float x , float y , float z );
		void UpdateCameraLookAt ( const glm::vec3 & new_look_at );
		void UpdateCameraUp ( float x , float y , float z );
		void UpdateCameraUp ( const glm::vec3 & new_up );

		void Reset ( );

		inline void SetCurrentMousePosition ( const float & x , const float & y ) { current_mouse_position_ = glm::vec2 ( x , y ); }

	private:

		glm::vec3 camera_position_;
		glm::vec3 look_at_;
		glm::vec3 up_;

		glm::mat4 view_mat_;

		glm::vec2 current_mouse_position_;

	};
}

#endif //QT5GLVIEWER_CAMERA_H
