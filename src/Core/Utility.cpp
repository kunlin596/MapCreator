//
// Created by linkun on 15/09/15.
//

#include"Core/Utility.h"
#include <string>
#include <iostream>


namespace MapCreator {

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


    glm::mat4 ConvertCVMatx44fToGLMmat4 ( const cv::Matx44f & m ) {

        return glm::make_mat4 ( m.val );

    }

    cv::Matx44f ConvertGLMmat4ToCVMatx44f ( const glm::mat4 & m ) {

        return cv::Matx44f ( glm::value_ptr ( m ) );

    }


}