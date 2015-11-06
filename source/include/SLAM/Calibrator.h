//
// Created by LinKun on 9/13/15.
//

#ifndef LK_SLAM_CALIBRATOR_H
#define LK_SLAM_CALIBRATOR_H


#include <string>
#include <opencv2/opencv.hpp>

#include "SLAM/CommonDefinitions.h"

#include <Core/Serialize.h>

namespace {

    static const char kFeatureDataHeader[] = "FeatureData";

}

namespace NiS {


    struct InternalCalibrationInfo {
        using CoefficientsVector = std::vector<float>;
        using LocalCalibrationRow = std::vector<CoefficientsVector>;
        using LocalCalibrationTable = std::vector<LocalCalibrationRow>;

        LocalCalibrationTable local_calibration_table;
        CoefficientsVector global_calibration_vector;
        CoefficientsVector hfov_calibration_vector;
        CoefficientsVector vfov_calibration_vector;

        bool IsValid() const {

            return !local_calibration_table.empty() and
                   !global_calibration_vector.empty() and
                   !hfov_calibration_vector.empty() and
                   !vfov_calibration_vector.empty();
        }
    };

    struct InternalCalibrationReader {
        static InternalCalibrationInfo Read(const std::string &file_name) {

            using namespace std;

            std::ifstream in(file_name, std::ios::binary);

            InternalCalibrationInfo info;

            const std::string kFileHeader = "InternalCalibration";

            if (in) {

                const auto head = NiS::ReadString(in, static_cast < unsigned long >( kFileHeader.size()));
                const auto version = NiS::Read<int>(in);

                InternalCalibrationInfo::LocalCalibrationTable table(static_cast<unsigned long>(NiS::Read<int>(in)));

                for (auto &coef_line : table) {
                    coef_line.resize(static_cast<unsigned long>(NiS::Read<int>(in)));
                    for (auto &coef : coef_line) {
                        coef = NiS::ReadVector<InternalCalibrationInfo::CoefficientsVector::value_type>(in);
                    }
                }

                info.local_calibration_table = table;
                info.global_calibration_vector = NiS::ReadVector<InternalCalibrationInfo::CoefficientsVector::value_type>(
                        in);
                info.hfov_calibration_vector = NiS::ReadVector<InternalCalibrationInfo::CoefficientsVector::value_type>(
                        in);
                info.vfov_calibration_vector = NiS::ReadVector<InternalCalibrationInfo::CoefficientsVector::value_type>(
                        in);

            }

            return info;
        }
    };

    class Calibrator {
        using CoefficientsVector = std::vector<float>;
        using LocalCalibrationRow = std::vector<CoefficientsVector>;
        using LocalCalibrationTable = std::vector<LocalCalibrationRow>;

        struct InternalCalibrationData {
            LocalCalibrationTable local_calibration_table;
            CoefficientsVector global_calibration_vector;
            CoefficientsVector hfov_calibration_vector;
            CoefficientsVector vfov_calibration_vector;

            bool IsValid() const {

                return !local_calibration_table.empty() and
                       !global_calibration_vector.empty() and
                       !hfov_calibration_vector.empty() and
                       !vfov_calibration_vector.empty();
            }
        };

    public:

        Calibrator();

        Calibrator(const std::string &path);

        ~Calibrator();

        PointImage CalibrateImage(const cv::Mat &depth_image);

        inline bool IsValid() const {

            return internal_calibration_data_.IsValid();
        }

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

            const CoefficientsVector &coef = internal_calibration_data_.global_calibration_vector;
            return coef.empty() ? depth : depth * NthDegreeEquation(coef, depth);
        }

        float CorrectDistortion(int row, int col, float depth) const {

            const CoefficientsVector &coef = internal_calibration_data_.local_calibration_table[row][col];
            return coef.empty() ? 0.0f : depth * NthDegreeEquation(coef, depth);
        }

        InternalCalibrationData internal_calibration_data_;

    };

}


#endif //LK_SLAM_CALIBRATOR_H
