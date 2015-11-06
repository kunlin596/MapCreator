//
// Created by LinKun on 11/4/15.
//

#ifndef NIS_COORDINATECONVERTER_H
#define NIS_COORDINATECONVERTER_H

#include <Core/Serialize.h>

#include <opencv2/opencv.hpp>
#include <fstream>

#include "SLAM/Calibrator.h"

namespace NiS {

    using ScreenPoint = cv::Point2f;
    using WorldPoint = cv::Point3f;

    class CoordinateConverter {
    protected:

        virtual ScreenPoint WorldToScreen(WorldPoint const &world_point) = 0;

        virtual WorldPoint ScreenToWorld(ScreenPoint const &screen_point, ushort const depth) = 0;

        struct XtionFrameProperty {
            static const float kXtionHorizontalFOV;
            static const float kXtionVerticalFOV;
            static const float kXtionWidth;
            static const float kXtionHeight;
        };

    };

    class XtionCoordinateConverter : public CoordinateConverter {
    public:

        XtionCoordinateConverter() :
                universal_xz_factor_(GetXtionDepthFactor(XtionFrameProperty::kXtionHorizontalFOV)),
                universal_yz_factor_(GetXtionDepthFactor(XtionFrameProperty::kXtionVerticalFOV)) { }


        ScreenPoint WorldToScreen(WorldPoint const &world_point) override;

        WorldPoint ScreenToWorld(ScreenPoint const &screen_point, ushort const depth) override;

    private:

        float GetXtionDepthFactor(float fov) {

            return tanf(fov / 2) * 2;
        }

        float universal_xz_factor_;
        float universal_yz_factor_;

    };

    class AistCoordinateConverter : public CoordinateConverter {
    public:

        inline AistCoordinateConverter(std::string const &file_name) {

            internal_calibration_info_ = InternalCalibrationReader::Read(file_name);
        }

        ScreenPoint WorldToScreen(WorldPoint const &world_point) override;

        WorldPoint ScreenToWorld(ScreenPoint const &screen_point, ushort const depth) override;

    private:


        float NthDegreeEquation(const InternalCalibrationInfo::CoefficientsVector &coef, float x) const {

            float y = 0;
            float xx = 1.0f;

            for (const auto &c : coef) {
                y += c * xx;
                xx *= x;
            }

            return y;
        }

        float CorrectDepth(float depth) const {

            const auto &coef = internal_calibration_info_.global_calibration_vector;
            return coef.empty() ? depth : depth * NthDegreeEquation(coef, depth);
        }

        float CorrectDistortion(int row, int col, float depth) const {

            const auto &coef = internal_calibration_info_.local_calibration_table[row][col];
            return coef.empty() ? 0.0f : depth * NthDegreeEquation(coef, depth);
        }

        InternalCalibrationInfo internal_calibration_info_;

    };
};


#endif //NIS_COORDINATECONVERTER_H
