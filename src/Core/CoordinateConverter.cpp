#include "Core/CoordinateConverter.h"

namespace {
// TODO: make them configurable
static const float kXtionHorizontalFOV = 1.0225999f;
static const float kXtionVerticalFOV = 0.79661566f;
}  // namespace

namespace MapCreator {

CoordinateConverter::CoordinateConverter()
    : universal_xz_factor_(GetDepthFactor(kXtionHorizontalFOV)),
      universal_yz_factor_(GetDepthFactor(kXtionVerticalFOV)) {}

cv::Mat CoordinateConverter::Unproject(cv::Mat const& image) const {
  // TODO: store the image info data
  int channels = image.channels();
  int n_rows = image.rows;
  int n_cols = image.cols;

  if (image.isContinuous()) {
    n_cols *= n_rows;
    n_rows = 1;
  }

  cv::Mat_<cv::Point3f> ret(n_rows, n_cols);

  int row, col;
  for (row = 0; row < n_rows; ++row) {
    const uchar* row_data = image.ptr<const uchar>(row);
    for (col = 0; col < n_cols; ++col) {
      cv::Point3f p;
      auto gz = 0.001f * static_cast<float>(row_data[col]);
      p.x = gz * universal_xz_factor_ * (col / n_cols - 0.5f);
      p.y = -gz * universal_yz_factor_ * (row / n_rows - 0.5f);
      p.z = -gz;
      ret.at<cv::Point3f>(row, col) = p;
    }
  }
  return std::move(ret);
}

cv::Point2f CoordinateConverter::WorldToScreen(
    const cv::Point3f& world_point) const {
  // Inverse of Unproject: depth gz = -z, screen coordinate normalized to [0,1].
  if (std::abs(world_point.z) < 1e-9f) return cv::Point2f(0.5f, 0.5f);
  const float x = -world_point.x / (world_point.z * universal_xz_factor_) + 0.5f;
  const float y = world_point.y / (world_point.z * universal_yz_factor_) + 0.5f;
  return cv::Point2f(x, y);
}

cv::Mat CoordinateConverter::Project(cv::Mat const& image) const {
  // TODO: store the image info data
  int channels = image.channels();
  int n_rows = image.rows;
  int n_cols = image.cols;

  cv::Mat_<cv::Point2f> ret(n_rows, n_cols);

  if (image.isContinuous()) {
    n_cols *= n_rows;
    n_rows = 1;
  }

  return std::move(ret);
}

cv::Mat CalibratedCoordinateConverter::Unproject(cv::Mat const& image) const {
  // const auto& coef =
  //     internal_calibration_data_.local_calibration_table[row][col];
  // return coef.empty() ? 0.0f : depth * NthDegreeEquation(coef, depth);

  cv::Point3f pt;

  // if (depth > 0) {
  //   const float gz = CorrectDepth(CorrectDistortion(
  //                        static_cast<int>(screen_point.y),
  //                        static_cast<int>(screen_point.x), depth)) *
  //                    0.001f;

  //   const float xz_factor = NthDegreeEquation(
  //       internal_calibration_data_.hfov_calibration_vector, gz);
  //   const float yz_factor = NthDegreeEquation(
  //       internal_calibration_data_.vfov_calibration_vector, gz);
  //   const float gx = xz_factor * (static_cast<float>(screen_point.x) /
  //                                     XtionFrameProperty::kXtionWidth -
  //                                 0.5f);
  //   const float gy = yz_factor * (static_cast<float>(screen_point.y) /
  //                                     XtionFrameProperty::kXtionHeight -
  //                                 0.5f);

  //   pt.x = gx;
  //   pt.y = -gy;
  //   pt.z = -gz;
  // }

  return cv::Mat();
}

cv::Point2f CalibratedCoordinateConverter::WorldToScreen(
    const cv::Point3f& /*world_point*/) const {
  // TODO: implement the calibrated projection (see commented Project above).
  // The calibrated path is not yet ported; return the image center as a stub.
  return cv::Point2f(0.5f, 0.5f);
}

cv::Mat CalibratedCoordinateConverter::Project(cv::Mat const& image) const {
  // const float xz_factor = NthDegreeEquation(
  //     internal_calibration_data_.hfov_calibration_vector, world_point.z);
  // const float yz_factor = NthDegreeEquation(
  //     internal_calibration_data_.vfov_calibration_vector, world_point.z);
  // const float x =
  //     (world_point.x / xz_factor + 0.5f) * XtionFrameProperty::kXtionWidth;
  // const float y =
  //     (world_point.y / yz_factor + 0.5f) * XtionFrameProperty::kXtionHeight;
  // return cv::Point2f(x, y);
  return cv::Mat();
}

float CalibratedCoordinateConverter::NthDegreeEquation(
    std::vector<float> const& coef, float x) const {
  float y = 0;
  float xx = 1.0f;

  for (const auto& c : coef) {
    y += c * xx;
    xx *= x;
  }

  return y;
}

float CalibratedCoordinateConverter::CorrectDepth(float depth) const {
  // const auto& coef = internal_calibration_data_.global_calibration_vector;
  // return coef.empty() ? depth : depth * NthDegreeEquation(coef, depth);
  return 0.0f;
}

float CalibratedCoordinateConverter::CorrectDistortion(int row, int col,
                                                       float depth) const {
  return 0.0f;
  // const auto& coef = internal_calibration_data_.table[row][col];
  // return coef.empty() ? 0.0f : depth * NthDegreeEquation(coef, depth);
}

}  // namespace MapCreator
