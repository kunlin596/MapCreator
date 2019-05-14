#ifndef MAPCREATOR_COORDINATECONVERTER_H
#define MAPCREATOR_COORDINATECONVERTER_H

#include <opencv2/opencv.hpp>

#include "SLAM/Calibrator.h" // for InternalCalibrationData

namespace MapCreator {

class AbstractCoordinateConverter {
 public:

  virtual cv::Mat Project(const cv::Mat& image) = 0;
  virtual cv::Mat Unproject(const cv::Mat& image) = 0;
};

class CoordinateConverter : public AbstractCoordinateConverter {
 public:
  CoordinateConverter();
  ~CoordinateConverter() = default;
  cv::Mat Project(cv::Mat const& image) const override;
  cv::Mat Unproject(cv::Mat const& image) const override;

 private:
  float GetDepthFactor(float fov) { return tanf(fov / 2) * 2; }

  float Init();

  float universal_xz_factor_;
  float universal_yz_factor_;
};

class CalibratedCoordinateConverter : public AbstractCoordinateConverter {
 public:
  CalibratedCoordinateConverter() = default;
  ~CalibratedCoordinateConverter() = default;

  cv::Mat Project(cv::Mat const& image) const override;
  cv::Mat Unproject(cv::Mat const& image) const override;

  // TODO: Pass in JSON object
  // inline LoadCalibrationData(std::string const& file_name) {
  //   internal_calibration_data_ = InternalCalibrationReader::Read(file_name);
  // }

 private:
  float NthDegreeEquation(std::vector<float> const& coef, float x) const;
  float CorrectDepth(float depth) const;
  float CorrectDistortion(int row, int col, float depth) const;

  InternalCalibrationData internal_calibration_data_;
};
};  // namespace MapCreator

#endif  // MAPCREATOR_COORDINATECONVERTER_H
