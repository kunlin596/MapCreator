//
// Created by LinKun on 9/12/15.
//

#ifndef MAPCREATOR_IMAGE_H_
#define MAPCREATOR_IMAGE_H_

#include <opencv2/opencv.hpp>

namespace MapCreator {

using ColorImage = cv::Mat_<cv::Vec3b>;
using PointImage = cv::Mat_<cv::Vec3f>;

namespace Utils {
    
template <typename T>
cv::Mat_<T> Resize(
    const cv::Mat_<T>& image,
    int w,
    int h,
    int interpolation = cv::INTER_LINEAR)
{
    cv::Mat_<T> convertedImage;

    if (!image.empty() && w > 0 && h > 0) {
        cv::resize(image, convertedImage, cv::Size(w, h), 0, 0, interpolation);
    }

    return convertedImage;
}

template <typename T>
cv::Mat_<T> Flip(const cv::Mat_<T>& image, int code)
{
    cv::Mat_<T> convertedImage;
    cv::flip(image, convertedImage, code);
    return convertedImage;
}

template <typename T>
cv::Mat_<T> Transpose(const cv::Mat_<T>& image)
{
    cv::Mat_<T> cvt_image;
    cv::transpose(image, cvt_image);
    return cvt_image;
}

cv::Mat_<cv::Vec3b> SwapChannel(const cv::Mat_<cv::Vec3b>& image);

cv::Mat_<uchar> RGBToGray(const cv::Mat_<cv::Vec3b>& image);

cv::Mat_<uchar> BGRToGray(const cv::Mat_<cv::Vec3b>& image);

cv::Mat_<cv::Vec3b> GrayToColor(const cv::Mat_<uchar>& image);

} // namespace Utils
} // namespace MapCreator

#endif // MAPCREATOR_IMAGE_H_
