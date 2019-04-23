//
// Created by LinKun on 10/3/15.
//

#include <glm/gtc/matrix_transform.hpp>
#include "BasicViewer/Camera.h"

#include <iostream>

#include <QEasingCurve>

using namespace std;

namespace MapCreator {

	Camera::Camera ( ) {

		Reset ( );
	}

	Camera::~Camera ( ) { }


	void Camera::SetView ( glm::vec3 eye , glm::vec3 center , glm::vec3 up ) {

		view_mat_ = glm::lookAt ( eye , eye + center , up );
	}

	void Camera::SetView ( float eye_x , float eye_y , float eye_z ,
	                       float center_x , float center_y , float center_z ,
	                       float up_x , float up_y , float up_z ) {

		glm::vec3 eye ( eye_x , eye_y , eye_z );
		glm::vec3 center ( center_x , center_y , center_z );
		glm::vec3 up ( up_x , up_y , up_z );

		camera_position_ = eye;
		look_at_         = center;
		up_              = up;
		view_mat_        = glm::lookAt ( camera_position_ , camera_position_ + look_at_ , up_ );
	}

	glm::mat4 Camera::ViewMatrix ( ) const {

		return view_mat_;
	}


	void Camera::UpdateCameraPosition ( float x , float y , float z ) {

		camera_position_ = glm::vec3 ( x , y , z );
		view_mat_        = glm::lookAt ( camera_position_ , camera_position_ + look_at_ , up_ );
	}

	void Camera::UpdateCameraPosition ( const glm::vec3 & new_position ) {

		camera_position_ = new_position;
		view_mat_        = glm::lookAt ( camera_position_ , camera_position_ + look_at_ , up_ );
	}

	void Camera::UpdateCameraLookAt ( float x , float y , float z ) {

		look_at_  = glm::vec3 ( x , y , z );
		view_mat_ = glm::lookAt ( camera_position_ , camera_position_ + look_at_ , up_ );

	}

	void Camera::UpdateCameraLookAt ( const glm::vec3 & new_look_at ) {

		look_at_  = new_look_at;
		view_mat_ = glm::lookAt ( camera_position_ , camera_position_ + look_at_ , up_ );
	}

	void Camera::UpdateCameraUp ( float x , float y , float z ) {

		up_       = glm::vec3 ( x , y , z );
		view_mat_ = glm::lookAt ( camera_position_ , camera_position_ + look_at_ , up_ );
	}

	void Camera::UpdateCameraUp ( const glm::vec3 & new_up ) {

		up_       = new_up;
		view_mat_ = glm::lookAt ( camera_position_ , camera_position_ + look_at_ , up_ );
	}

	void Camera::MouseUpdate ( const glm::vec2 & mouse_delta ) {

		look_at_  = glm::mat3 ( glm::rotate ( glm::mat4 ( ) , -mouse_delta.x * 0.1f , up_ ) ) * look_at_;
		look_at_  = glm::mat3 ( glm::rotate ( glm::mat4 ( ) , mouse_delta.y * 0.1f , glm::cross ( up_ , look_at_ ) ) ) * look_at_;
		view_mat_ = glm::lookAt ( camera_position_ , camera_position_ + look_at_ , up_ );
	}

	void Camera::KeyUpdate ( const Move & move , const float rate ) {

		glm::vec3 vertical_direction = glm::normalize ( glm::cross ( up_ , look_at_ ) );
		glm::vec3 head_direction     = glm::normalize ( glm::cross ( look_at_ , vertical_direction ) );

		switch ( move ) {

			case Move::StrafeForward:
				camera_position_ += glm::normalize ( look_at_ ) * rate;
				break;
			case Move::StrafeBackward:
				camera_position_ -= glm::normalize ( look_at_ ) * rate;
				break;
			case Move::StrafeLeft:
				camera_position_ += vertical_direction * rate;
				break;
			case Move::StrafeRight:
				camera_position_ -= vertical_direction * rate;
				break;
			case Move::StrafeUp:
				camera_position_ += head_direction * rate;
				break;
			case Move::StrafeDown:
				camera_position_ -= head_direction * rate;
				break;
			default:
				break;
		}

		view_mat_ = glm::lookAt ( camera_position_ , camera_position_ + look_at_ , up_ );
	}

	void Camera::Reset ( ) {

		SetView ( 0.0f , 0.5f , 5.0f ,
		          0.0f , 0.0f , -0.5f ,
		          0.0f , 1.0f , 0.0f );
	}

}