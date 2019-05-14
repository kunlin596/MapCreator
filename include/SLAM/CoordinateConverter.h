//
// Created by LinKun on 11/4/15.
//

#ifndef MAPCREATOR_COORDINATECONVERTER_H
#define MAPCREATOR_COORDINATECONVERTER_H

#include <opencv2/opencv.hpp>

#include <fstream>

#include "Core/Serialize.h"
#include "SLAM/Calibrator.h"

namespace MapCreator {

class CoordinateConverter {
 public:
  virtual ScreenPoint WorldToScreen(const WorldPoint& world_point) const {
    return ScreenPoint();
  };

  virtual WorldPoint ScreenToWorld(const ScreenPoint& screen_point,
                                   ushort const depth) const {
    return WorldPoint();
  };

  struct XtionFrameProperty {
    static const float kXtionHorizontalFOV;
    static const float kXtionVerticalFOV;
    static const float kXtionWidth;
    static const float kXtionHeight;
  };
};

class XtionCoordinateConverter : public CoordinateConverter {
 public:
  XtionCoordinateConverter()
      : universal_xz_factor_(
            GetXtionDepthFactor(XtionFrameProperty::kXtionHorizontalFOV)),
        universal_yz_factor_(
            GetXtionDepthFactor(XtionFrameProperty::kXtionVerticalFOV)) {}

  ~XtionCoordinateConverter() = default;

  ScreenPoint WorldToScreen(WorldPoint const& world_point) const override;

  WorldPoint ScreenToWorld(ScreenPoint const& screen_point,
                           ushort const depth) const override;

 private:
  float GetXtionDepthFactor(float fov) { return tanf(fov / 2) * 2; }

  float universal_xz_factor_;
  float universal_yz_factor_;
};

class AistCoordinateConverter : public CoordinateConverter {
 public:
  AistCoordinateConverter() = default;
  ~AistCoordinateConverter() = default;
  inline AistCoordinateConverter(std::string const& file_name) {
    internal_calibration_info_ = InternalCalibrationReader::Read(file_name);
  }

  ScreenPoint WorldToScreen(WorldPoint const& world_point) const override;

  WorldPoint ScreenToWorld(ScreenPoint const& screen_point,
                           ushort const depth) const override;

 private:
  float NthDegreeEquation(
      const InternalCalibrationInfo::CoefficientsVector& coef, float x) const {
    float y = 0;
    float xx = 1.0f;

    for (const auto& c : coef) {
      y += c * xx;
      xx *= x;
    }

    return y;
  }

  float CorrectDepth(float depth) const {
    const auto& coef = internal_calibration_info_.global_calibration_vector;
    return coef.empty() ? depth : depth * NthDegreeEquation(coef, depth);
  }

  float CorrectDistortion(int row, int col, float depth) const {
    const auto& coef =
        internal_calibration_info_.local_calibration_table[row][col];
    return coef.empty() ? 0.0f : depth * NthDegreeEquation(coef, depth);
  }

  InternalCalibrationInfo internal_calibration_info_;
};
};  // namespace MapCreator

#endif  // MAPCREATOR_COORDINATECONVERTER_H
