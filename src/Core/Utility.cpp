//
// Created by linkun on 15/09/15.
//

#include"Core/Utility.h"
#include <string>
#include <iostream>


namespace MapCreator {

	void PrintMessage ( const std::string msg , int status ) {

		using namespace std;

		string m = "[SLAM]";
		switch ( status ) {
			case 1:
				m += " [PROCESSING]  - ";
				break;
			case 2:
				m += " [OK]          - ";
				break;
			case 3:
				m += " [WARNING]     - ";
				break;
			case 4:
				m += " [ERROR]       - ";
				break;
			default:
				break;
		}

		cout << m << msg << endl;
	}

	std::string ConvertConstCStrToStdString ( const unsigned char * c_str , size_t len ) {

		std::string str;
		str.resize ( len );
		for ( auto i = 0 ; i < len ; ++i ) {
			str[ i ] = * ( c_str + i );
		}
		return str;
	}

	glm::mat4 ConvertMat ( const cv::Matx44f & m ) {

		return glm::mat4 ( );
	}


	glm::mat4 Convert_OpenCV_Matx44f_To_GLM_mat4 ( const cv::Matx44f & m ) {

		return glm::make_mat4 ( m.val );

	}

	cv::Matx44f Convert_GLM_mat4_To_OpenCV_Matx44f ( const glm::mat4 & m ) {

		return cv::Matx44f ( glm::value_ptr ( m ) );

	}


}