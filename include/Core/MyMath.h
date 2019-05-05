//
// Created by LinKun on 9/12/15.
//

#ifndef MAPCREATOR_MATH_H
#define MAPCREATOR_MATH_H

#if !defined(_USE_MATH_DEFINES)
#define    _USE_MATH_DEFINES
#endif

#include <cmath>

#include <opencv2/opencv.hpp>
#include <glm/glm.hpp>

namespace MapCreator {

    // πをかけた値を返す
    inline float PI ( float x ) {

        return static_cast< float >( M_PI ) * x;
    }

    inline double PI ( double x ) {

        return M_PI * x;
    }

    // ラジアンとディグリーの変換
    inline double Degree ( double radian ) {

        return radian * 180 / M_PI;
    }

    inline float Degree ( float radian ) {

        return radian * 180.0f / static_cast< float >( M_PI );
    }

    inline double Radian ( double degree ) {

        return degree * M_PI / 180;
    }

    inline float Radian ( float degree ) {

        return degree * static_cast< float >( M_PI ) / 180.0f;
    }


    /// @brief  ベクトルの正規化
    /// @param  v   元のベクトル
    /// @return 正規化されたベクトル
    template < typename T , int N >
    cv::Vec < T , N > Normalize ( const cv::Vec < T , N > & v ) {

        cv::Vec < T , N > tmp;
        cv::normalize ( v , tmp );
        return tmp;
    }


    /// @brief  ベクトル同士の補間（v1～v2を補間する）
    /// @param  v1  ベクトル１
    /// @param  v2  ベクトル２
    /// @param  t   時間（0.0～1.0）
    /// @return 補間されたベクトル
    template < typename T , int N >
    cv::Vec < T , N > Interpolate ( const cv::Vec < T , N > & v1 , const cv::Vec < T , N > & v2 , float t ) {

        return v1 * ( 1.0f - t ) + v2 * t;
    }

    /// @brief  並進行列を作成
    /// @param  v   並進ベクトル
    /// @return 回転量ゼロの 4x4 の行列
    cv::Matx44f CreateTranslationMatrix ( const cv::Vec3f & v );


    /// @brief  回転並進行列を作成
    /// @param  3x3 の回転行列
    /// @return 4x4 の回転並進行列
    cv::Matx44f CreateMatrix4x4 ( const cv::Matx33f & r , const cv::Vec3f & t = cv::Vec3f ( 0 , 0 , 0 ) );


    /// @brief  クォータニオン (t; x, y, z)
    typedef cv::Vec < float , 4 > Quaternion;

    /// @brief  クォータニオン作成
    /// @param  axis    回転軸
    /// @param  theta   回転角（ラジアン）
    /// @return クォータニオン
    Quaternion CreateQuaternion ( const cv::Vec3f & axis , float theta );


    /// @brief  クォータニオン作成
    /// @param  3x3 の回転行列
    /// @return クォータニオン
    Quaternion CreateQuaternion ( const cv::Matx33f & m );


    /// @brief  球面線形補間
    /// @param  q1  クォータニオン１
    /// @param  q2  クォータニオン２
    /// @param  t   時間（0.0～1.0）
    /// @return 補間されたクォータニオン
    Quaternion Slerp ( const Quaternion & q1 , const Quaternion & q2 , float t );


    // 対数クォータニオン
    Quaternion Log ( const Quaternion & q );

    Quaternion Exp ( const Quaternion & q );

    /// @brief  対数クォータニオンを利用した、複数クォータニオンの補間
    /// @param  quaternions     複数のクォータニオン
    /// @param  weights         任意のウェイト（quaternions と同じ数である必要がある）
    /// @return 補間されたクォータニオン
    Quaternion ExpMap ( const std::vector < Quaternion > & quaternions , const std::vector < float > & weights );


    /// @brief  回転行列を作成
    /// @param  q   クォータニオン
    /// @return 3x3 の回転行列
    cv::Matx33f CreateRotationMatrix ( const Quaternion & q );


    /// @brief  回転行列を作成
    /// @param  axis    回転軸
    /// @param  theta   回転角（ラジアン）
    /// @return 3x3 の回転行列
    cv::Matx33f CreateRotationMatrix ( const cv::Vec3f & axis , float theta );


    /// @brief  Ｘ軸回転の回転行列を作成
    /// @param  theta   回転角（ラジアン）
    /// @return 3x3 の回転行列
    cv::Matx33f CreateRotationMatrixX ( float theta );


    /// @brief  Ｙ軸回転の回転行列を作成
    /// @param  theta   回転角（ラジアン）
    /// @return 3x3 の回転行列
    cv::Matx33f CreateRotationMatrixY ( float theta );


    /// @brief  Ｚ軸回転の回転行列を作成
    /// @param  theta   回転角（ラジアン）
    /// @return 3x3 の回転行列
    cv::Matx33f CreateRotationMatrixZ ( float theta );


    /// @brief  ２つのベクトル間の回転行列を作成
    /// @param  v1  ベクトル１
    /// @param  v2  ベクトル２
    /// @return v1 → v2 への回転を表す 3x3 の回転行列
    cv::Matx33f CreateRotationMatrix ( const cv::Vec3f & v1 , const cv::Vec3f & v2 );    // v1 → v2 への回転


    /// @brief  4x4 行列の補間
    /// @param  m1  行列１
    /// @param  m2  行列２
    /// @param  t   時間（0.0～1.0）
    /// @return 補間された行列
    cv::Matx44f Interpolate ( const cv::Matx44f & m1 , const cv::Matx44f & m2 , float t );

}


// 演算子のオーバーロードはグローバル名前空間に定義しておく

/// @brief  v * m を計算する<br>
inline cv::Vec4f operator * ( const cv::Vec4f & v , const cv::Matx44f & m ) {

    return cv::Vec4f ( ( cv::Matx < float , 1 , 4 > ( v.val ) * m ).val );
}


/// @brief  v * m を計算する<br>
///         v の w 成分は 1.0 であるとみなして計算する
inline cv::Vec3f operator * ( const cv::Vec3f & v , const cv::Matx44f & m ) {

    const cv::Vec4f w = cv::Vec4f ( v ( 0 ) , v ( 1 ) , v ( 2 ) , 1.0f ) * m;
    return cv::Vec3f ( w ( 0 ) , w ( 1 ) , w ( 2 ) );
}


/// @brief  v * m を計算する<br>
inline cv::Vec3f operator * ( const cv::Vec3f & v , const cv::Matx33f & m ) {

    return cv::Vec3f ( ( cv::Matx < float , 1 , 3 > ( v.val ) * m ).val );
}


/// @brief  4x4 行列から回転行列を取り出す
inline cv::Matx33f & operator << ( cv::Matx33f & r , const cv::Matx44f & m ) {

    r = cv::Mat ( m ) ( cv::Rect ( 0 , 0 , 3 , 3 ) );
    return r;
}


/// @brief  4x4 行列から並進ベクトルを取り出す
inline cv::Vec3f & operator << ( cv::Vec3f & t , const cv::Matx44f & m ) {

    t = cv::Mat ( m , false ) ( cv::Rect ( 0 , 3 , 3 , 1 ) );
    return t;
}


/// @brief  4x4 行列に回転行列を代入する
inline cv::Matx44f & operator << ( cv::Matx44f & m , const cv::Matx33f & r ) {

    cv::Mat ( r ).copyTo ( cv::Mat ( m , false ) ( cv::Rect ( 0 , 0 , 3 , 3 ) ) );
    return m;
}


/// @brief  4x4 行列に並進ベクトルを代入する
inline cv::Matx44f & operator << ( cv::Matx44f & m , const cv::Vec3f & t ) {

    cv::Mat ( cv::Matx13f ( t ( 0 ) , t ( 1 ) , t ( 2 ) ) ).copyTo ( cv::Mat ( m , false ) ( cv::Rect ( 0 , 3 , 3 , 1 ) ) );
    return m;
}


/// @brief  4x4 行列に 4x4 行列をコピーする
inline cv::Matx44f & operator << ( cv::Matx44f & dst , const cv::Matx44f & src ) {

    dst = src;
    return dst;
}

inline glm::mat3 ExtractRotationMatrix ( const glm::mat4 & m ) {

    glm::mat3 r = glm::mat3 ( m );
    return r;
}

inline glm::vec3 ExtractTranslationVector ( const glm::mat4 & m ) {

    glm::vec3 t;
    t.x = m[ 3 ][ 0 ];
    t.y = m[ 3 ][ 1 ];
    t.z = m[ 3 ][ 2 ];
    return t;

}

//inline glm::mat3 & operator << ( glm::mat3 & r , const glm::mat4 & m ) {
//
//  r = glm::mat3 ( m );
//
//  return r;
//}
//
//inline glm::vec3 & operator << ( glm::vec3 & t , const glm::mat4 & m ) {
//
//  t.x = m[ 3 ][ 0 ];
//  t.y = m[ 3 ][ 1 ];
//  t.z = m[ 3 ][ 2 ];
//
//  return t;
//}


#endif //MAPCREATOR_MATH_H
