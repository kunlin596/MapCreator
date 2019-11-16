//
// Created by LinKun on 9/12/15.
//

#include "Image.h"

namespace MapCreator {

cv::Mat_<cv::Vec3b> SwapChannel(const cv::Mat_<cv::Vec3b> &image) {
  cv::Mat_<cv::Vec3b> cvt_image;
  cv::cvtColor(image, cvt_image, cv::COLOR_RGB2BGR);
  return cvt_image;
}

cv::Mat_<uchar> RGBToGray(const cv::Mat_<cv::Vec3b> &image) {
  cv::Mat_<uchar> cvt_image;
  cv::cvtColor(image, cvt_image, cv::COLOR_RGB2GRAY);
  return cvt_image;
}

cv::Mat_<uchar> BGRToGray(const cv::Mat_<cv::Vec3b> &image) {
  cv::Mat_<uchar> cvt_image;
  cv::cvtColor(image, cvt_image, cv::COLOR_BGR2GRAY);
  return cvt_image;
}

cv::Mat_<cv::Vec3b> GrayToColor(const cv::Mat_<uchar> &image) {
  cv::Mat_<cv::Vec3b> cvt_image;
  cv::cvtColor(image, cvt_image, cv::COLOR_GRAY2BGR);
  return cvt_image;
}
}  // namespace MapCreator
