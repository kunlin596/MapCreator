#ifndef MAPCREATOR_CALIBRATOR_H
#define MAPCREATOR_CALIBRATOR_H

#include <opencv2/opencv.hpp>
#include <string>

#include "Serialize.h"
#include "SLAM.h"

namespace {

static const char kFeatureDataHeader[] = "FeatureData";
}

namespace MapCreator {

struct InternalCalibrationInfo {
  using CoefficientsVector = std::vector<float>;
  using LocalCalibrationRow = std::vector<CoefficientsVector>;
  using LocalCalibrationTable = std::vector<LocalCalibrationRow>;

  LocalCalibrationTable table;
  CoefficientsVector global_vector;
  CoefficientsVector hfov_vector;
  CoefficientsVector vfov_vector;

  bool IsValid() const {
    return !table.empty() and !global_vector.empty() and
           !hfov_vector.empty() and !vfov_vector.empty();
  }
};

struct InternalCalibrationReader {
  static InternalCalibrationInfo Read(const std::string &file_name) {
    using namespace std;

    std::ifstream in(file_name, std::ios::binary);

    InternalCalibrationInfo info;

    const std::string kFileHeader = "InternalCalibration";

    if (in) {
      const auto head =
          ReadString(in, static_cast<unsigned long>(kFileHeader.size()));
      // const auto version = ::MapCreator::Read<int>(in);

      InternalCalibrationInfo::LocalCalibrationTable table(
          static_cast<unsigned long>(::MapCreator::Read<int>(in)));

      for (auto &coef_line : table) {
        coef_line.resize(
            static_cast<unsigned long>(::MapCreator::Read<int>(in)));
        for (auto &coef : coef_line) {
          coef = ::MapCreator::ReadVector<
              InternalCalibrationInfo::CoefficientsVector::value_type>(in);
        }
      }

      info.table = table;
      info.global_vector = ::MapCreator::ReadVector<
          InternalCalibrationInfo::CoefficientsVector::value_type>(in);
      info.hfov_vector = ::MapCreator::ReadVector<
          InternalCalibrationInfo::CoefficientsVector::value_type>(in);
      info.vfov_vector = ::MapCreator::ReadVector<
          InternalCalibrationInfo::CoefficientsVector::value_type>(in);
    }

    return info;
  }
};

class Calibrator {
  using CoefficientsVector = std::vector<float>;
  using LocalCalibrationRow = std::vector<CoefficientsVector>;
  using LocalCalibrationTable = std::vector<LocalCalibrationRow>;

  struct InternalCalibrationData {
    LocalCalibrationTable table;
    CoefficientsVector global_vector;
    CoefficientsVector hfov_vector;
    CoefficientsVector vfov_vector;

    bool IsValid() const {
      return !table.empty() and !global_vector.empty() and
             !hfov_vector.empty() and !vfov_vector.empty();
    }
  };

 public:
  Calibrator();

  Calibrator(const std::string &path);

  ~Calibrator();

  PointImage CalibrateImage(const cv::Mat &depth_image);

  inline bool IsValid() const { return internal_calibration_data_.IsValid(); }

  cv::Point2f WorldToScreen(cv::Point3f const &point, int rows, int cols);

  cv::Point3f ScreenToWorld(int row, int col, float depth, int rows, int cols);

 private:
  InternalCalibrationData ReadHelper(const std::string &path);

  float NthDegreeEquation(const CoefficientsVector &coef, float x) const {
    float y = 0;
    float xx = 1.0f;

    for (const auto &c : coef) {
      y += c * xx;
      xx *= x;
    }

    return y;
  }

  float CorrectDepth(float depth) const {
    const CoefficientsVector &coef = internal_calibration_data_.global_vector;
    return coef.empty() ? depth : depth * NthDegreeEquation(coef, depth);
  }

  float CorrectDistortion(int row, int col, float depth) const {
    const CoefficientsVector &coef = internal_calibration_data_.table[row][col];
    return coef.empty() ? 0.0f : depth * NthDegreeEquation(coef, depth);
  }

  InternalCalibrationData internal_calibration_data_;
};

}  // namespace MapCreator

#endif  // MAPCREATOR_CALIBRATOR_H
