//
// Created by LinKun on 9/13/15.
//

#include "SLAM/Calibrator.h"
#include <Core/Serialize.h>

namespace {

using namespace std;

const std::string kFileHeader = "InternalCalibration";

}  // namespace

namespace MapCreator {

Calibrator::Calibrator(const std::string &path) {
  internal_calibration_data_ = ReadHelper(path);
}

Calibrator::Calibrator() {}

Calibrator::~Calibrator() {}

PointImage Calibrator::CalibrateImage(const cv::Mat &depth_image) {
  const int rows = depth_image.rows;
  const int cols = depth_image.cols;

  PointImage point_image(rows, cols);

  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      point_image(row, col) =
          ScreenToWorld(row, col, depth_image.at<ushort>(row, col),
                        depth_image.rows, depth_image.cols);
    }
  }

  return point_image;
}

Calibrator::InternalCalibrationData Calibrator::ReadHelper(
    const std::string &path) {
  using namespace std;

  std::ifstream in(path, std::ios::binary);

  InternalCalibrationData data;

  if (in) {
    const std::string head = MapCreator::ReadString(
        in, static_cast<unsigned int>(kFileHeader.size()));
    const int version = MapCreator::Read<int>(in);

    LocalCalibrationTable table(
        static_cast<unsigned long>(MapCreator::Read<int>(in)));

    for (auto &coef_line : table) {
      coef_line.resize(static_cast<unsigned long>(MapCreator::Read<int>(in)));
      for (auto &coef : coef_line) {
        coef = MapCreator::ReadVector<CoefficientsVector::value_type>(in);
      }
    }

    data.local_calibration_table = table;

    //
    data.global_calibration_vector =
        MapCreator::ReadVector<CoefficientsVector::value_type>(in);

    //
    data.hfov_calibration_vector =
        MapCreator::ReadVector<CoefficientsVector::value_type>(in);
    data.vfov_calibration_vector =
        MapCreator::ReadVector<CoefficientsVector::value_type>(in);
  }

  return data;
}

cv::Point3f Calibrator::ScreenToWorld(int row, int col, float depth, int rows,
                                      int cols) {
  cv::Point3f pt(std::numeric_limits<float>::quiet_NaN(),
                 std::numeric_limits<float>::quiet_NaN(),
                 std::numeric_limits<float>::quiet_NaN());

  if (depth > 0) {
    const float gz = CorrectDepth(CorrectDistortion(row, col, depth)) * 0.001f;
    const float xz_factor = NthDegreeEquation(
        internal_calibration_data_.hfov_calibration_vector, gz);
    const float yz_factor = NthDegreeEquation(
        internal_calibration_data_.vfov_calibration_vector, gz);
    const float gx = xz_factor * (static_cast<float>(col) / cols - 0.5f);
    const float gy = yz_factor * (static_cast<float>(row) / rows - 0.5f);

    pt.x = gx;
    pt.y = -gy;
    pt.z = -gz;
  }

  return pt;
}

cv::Point2f Calibrator::WorldToScreen(cv::Point3f const &point, int rows,
                                      int cols) {
  const float xz_factor = NthDegreeEquation(
      internal_calibration_data_.hfov_calibration_vector, point.z);
  const float yz_factor = NthDegreeEquation(
      internal_calibration_data_.vfov_calibration_vector, point.z);
  const float x = (point.x / xz_factor + 0.5f) * cols;
  const float y = (point.y / yz_factor + 0.5f) * rows;
  return cv::Point2f(x, y);
}

}  // namespace MapCreator