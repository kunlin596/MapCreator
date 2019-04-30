//
// Created by LinKun on 9/12/15.
//

#ifndef MAPCREATOR_UTILITY_H
#define MAPCREATOR_UTILITY_H

#include <algorithm>
#include <string>
#include <opencv2/opencv.hpp>

#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_access.hpp>

#include <QString>

namespace MapCreator {

	using namespace std;

	void PrintMessage ( const std::string msg , int status );

	/// [min, max] の区間に入っていなかったら true を返す
	template < typename T >
	inline bool OutOfRange ( T value , T range_min , T range_max ) {

		return ( ( value < range_min ) || ( range_max < value ) );
	}

	/// [min, max] の区間に入るように制限する
	template < typename T >
	inline T LimitRange ( T value , T range_min , T range_max ) {

		value = ( std::max ) ( value , range_min );
		value = ( std::min ) ( value , range_max );
		return value;
	}

	std::string ConvertConstCStrToStdString ( const unsigned char * c_str , size_t len );

	glm::mat4 ConvertMat ( const cv::Matx44f & m );

	glm::mat4   Convert_OpenCV_Matx44f_To_GLM_mat4 ( const cv::Matx44f & m );
	cv::Matx44f Convert_GLM_mat4_To_OpenCV_Matx44f ( const glm::mat4 & m );

	inline std::ostream & operator << ( std::ostream & os , const glm::vec4 & v ) {

		os << v.x << " " << v.y << " " << v.z << " " << v.w << std::endl;
		return os;
	}

	inline std::ostream & operator << ( std::ostream & os , const glm::vec3 & v ) {

		os << v.x << " " << v.y << " " << v.z << std::endl;
		return os;
	}

	inline std::ostream & operator << ( std::ostream & os , const glm::mat4 & m ) {

		os << glm::row ( m , 0 );
		os << glm::row ( m , 1 );
		os << glm::row ( m , 2 );
		os << glm::row ( m , 3 );
		return os;
	}

	inline std::ostream & operator << ( std::ostream & os , const glm::mat3 & m ) {

		os << glm::row ( m , 0 );
		os << glm::row ( m , 1 );
		os << glm::row ( m , 2 );
		return os;
	}

	inline QString ConvertTime ( int ms ) {

		int s = ms / 1000;
		ms %= 1000;
		int m = s / 60;
		s %= 60;
		int h = m / 60;
		m %= 60;

		return QString ( "%1h: %2m: %3s: %4ms" ).arg ( h ).arg ( m ).arg ( s ).arg ( ms );
	}


}

#endif //MAPCREATOR_UTILITY_H
