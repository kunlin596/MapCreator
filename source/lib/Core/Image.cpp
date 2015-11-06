//
// Created by LinKun on 9/12/15.
//

#include "Core/Image.h"

namespace NiS {

    // RGB→BGR（またはその逆）
    cv::Mat_<cv::Vec3b> SwapChannel(const cv::Mat_<cv::Vec3b> &image) {
        cv::Mat_<cv::Vec3b> cvt_image;
        cv::cvtColor(image, cvt_image, CV_RGB2BGR);
        return cvt_image;
    }


    // RGB 画像(24bit)をグレースケール(8bit)に変換する
    cv::Mat_<uchar> RGBToGray(const cv::Mat_<cv::Vec3b> &image) {
        cv::Mat_<uchar> cvt_image;
        cv::cvtColor(image, cvt_image, CV_RGB2GRAY);
        return cvt_image;
    }


    // BGR 画像(24bit)をグレースケール(8bit)に変換する
    cv::Mat_<uchar> BGRToGray(const cv::Mat_<cv::Vec3b> &image) {
        cv::Mat_<uchar> cvt_image;
        cv::cvtColor(image, cvt_image, CV_BGR2GRAY);
        return cvt_image;
    }


    // グレースケール(8bit)をカラー画像(24bit)に変換する
    cv::Mat_<cv::Vec3b> GrayToColor(const cv::Mat_<uchar> &image) {
        cv::Mat_<cv::Vec3b> cvt_image;
        cv::cvtColor(image, cvt_image, CV_GRAY2BGR);
        return cvt_image;
    }
}    // a
