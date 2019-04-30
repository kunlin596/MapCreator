//
// Created by LinKun on 9/12/15.
//

#ifndef MAPCREATOR_IMAGE_H
#define MAPCREATOR_IMAGE_H

#include <opencv2/opencv.hpp>

namespace MapCreator {
	/// 画像のサイズ変更
	template < typename T >
	cv::Mat_ < T > Resize ( const cv::Mat_ < T > & image , int w , int h , int interpolation = cv::INTER_LINEAR ) {

		cv::Mat_ < T > cvt_image;

		if ( !image.empty ( ) && w > 0 && h > 0 ) {
			cv::resize ( image , cvt_image , cv::Size ( w , h ) , 0 , 0 , interpolation );
		}

		return cvt_image;
	}


	/// 画像の反転
	template < typename T >
	cv::Mat_ < T > Flip ( const cv::Mat_ < T > & image , int code ) {

		cv::Mat_ < T > cvt_image;
		cv::flip ( image , cvt_image , code );
		return cvt_image;
	}


	/// 画像の転置
	template < typename T >
	cv::Mat_ < T > Transpose ( const cv::Mat_ < T > & image ) {

		cv::Mat_ < T > cvt_image;
		cv::transpose ( image , cvt_image );
		return cvt_image;
	}


	/// RGB→BGR（またはその逆）
	cv::Mat_ < cv::Vec3b > SwapChannel ( const cv::Mat_ < cv::Vec3b > & image );


	/// RGB 画像(24bit)をグレースケール(8bit)に変換する
	cv::Mat_ < uchar > RGBToGray ( const cv::Mat_ < cv::Vec3b > & image );


	/// BGR 画像(24bit)をグレースケール(8bit)に変換する
	cv::Mat_ < uchar > BGRToGray ( const cv::Mat_ < cv::Vec3b > & image );


	/// グレー画像をカラー画像にする
	cv::Mat_ < cv::Vec3b > GrayToColor ( const cv::Mat_ < uchar > & image );
}


#endif //MAPCREATOR_IMAGE_H
