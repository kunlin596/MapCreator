//
// Created by LinKun on 9/12/15.
//

#ifndef MAPCREATOR_IMAGE_H
#define MAPCREATOR_IMAGE_H

#include <opencv2/opencv.hpp>

namespace MapCreator {

using ColorImage = cv::Mat_<cv::Vec3b>;
using PointImage = cv::Mat_<cv::Vec3f>;

/**
 * @brief      Resice the image
 *
 * @param[in]  image          The image
 * @param[in]  w              { parameter_description }
 * @param[in]  h              { parameter_description }
 * @param[in]  interpolation  The interpolation
 *
 * @tparam     T              { description }
 *
 * @return     { description_of_the_return_value }
 */
template <typename T>
cv::Mat_<T> Resize(const cv::Mat_<T>& image, int w, int h,
                   int interpolation = cv::INTER_LINEAR) {
  cv::Mat_<T> cvt_image;

  if (!image.empty() && w > 0 && h > 0) {
    cv::resize(image, cvt_image, cv::Size(w, h), 0, 0, interpolation);
  }

  return cvt_image;
}

/**
 * @brief      Flip image
 *
 * @param[in]  image  The image
 * @param[in]  code   The code
 *
 * @tparam     T      { description }
 *
 * @return     { description_of_the_return_value }
 */
template <typename T>
cv::Mat_<T> Flip(const cv::Mat_<T>& image, int code) {
  cv::Mat_<T> cvt_image;
  cv::flip(image, cvt_image, code);
  return cvt_image;
}

/**
 * @brief      Transpose the image
 *
 * @param[in]  image  The image
 *
 * @tparam     T      { description }
 *
 * @return     { description_of_the_return_value }
 */
template <typename T>
cv::Mat_<T> Transpose(const cv::Mat_<T>& image) {
  cv::Mat_<T> cvt_image;
  cv::transpose(image, cvt_image);
  return cvt_image;
}

/**
 * @brief      Swap channels
 *
 * @param[in]  image  The image
 *
 * @return     { description_of_the_return_value }
 */
cv::Mat_<cv::Vec3b> SwapChannel(const cv::Mat_<cv::Vec3b>& image);

/**
 * @brief      Convert RGB to gray image
 *
 * @param[in]  image  The image
 *
 * @return     { description_of_the_return_value }
 */
cv::Mat_<uchar> RGBToGray(const cv::Mat_<cv::Vec3b>& image);

/**
 * @brief      Convert color image to
 *
 * @param[in]  image  The image
 *
 * @return     { description_of_the_return_value }
 */
cv::Mat_<uchar> BGRToGray(const cv::Mat_<cv::Vec3b>& image);

/**
 * @brief      Convert gray scale image to RGB image
 *
 * @param[in]  image  The image
 *
 * @return     { description_of_the_return_value }
 */
cv::Mat_<cv::Vec3b> GrayToColor(const cv::Mat_<uchar>& image);
}  // namespace MapCreator

#endif  // MAPCREATOR_IMAGE_H
