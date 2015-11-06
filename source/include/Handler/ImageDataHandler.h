//
// Created by LinKun on 9/13/15.
//

#ifndef LK_SLAM_IMAGEDATAHANDLER_H
#define LK_SLAM_IMAGEDATAHANDLER_H

// Core library includes

#include <QObject>
#include <QFileInfoList>
#include <Core/Serialize.h>
#include <boost/tuple/tuple.hpp>

#include <SLAM/CommonDefinitions.h>
#include <SLAM/KeyFrame.h>
#include <SLAM/Calibrator.h>

#include <QFutureWatcher>

namespace NiS {

    class ImageHandler2 : public QObject {

    Q_OBJECT

    public:

        struct XtionFrameProperty {
            static const float kXtionHorizontalFOV;
            static const float kXtionVerticalFOV;
            static const float kXtionWidth;
            static const float kXtionHeight;
        };

        inline ImageHandler2(QFileInfoList file_list, QObject *parent = 0) :
                file_list_(file_list),
                xz_factor_(GetXtionDepthFactor(XtionFrameProperty::kXtionHorizontalFOV)),
                yz_factor_(GetXtionDepthFactor(XtionFrameProperty::kXtionVerticalFOV)) { }

        inline ~ImageHandler2() { }

        inline void SetInternalCalibrator(Calibrator const &calibrator) { calibrator_ = calibrator; }

        inline RawDataFrames GetRawDataFrames() const { return raw_data_frames_; }

        inline const KeyFrames &GetKeyFrames() const { return keyframes_; }

    signals:

        void SendData(KeyFrames);

        void DoneReading();

        void Message(QString);

    public slots:

        void StartReading();

        void ConvertToPointImages();

        void ConvertToPointImagesWithInternalCalibration();

    private:

        void ConvertHelper();

        void ConvertHelper(bool calibrated);

        float GetXtionDepthFactor(float fov) {

            return tanf(fov / 2) * 2;
        }

        inline cv::Point3f ScreenToWorld(int x, int y, float depth) {

            const float gz = 0.001f * depth;
            cv::Point3f p;
            p.x = gz * xz_factor_ * (static_cast<float>(x) / XtionFrameProperty::kXtionWidth - 0.5f);
            p.y = -gz * yz_factor_ * (static_cast<float>(y) / XtionFrameProperty::kXtionHeight - 0.5f);
            p.z = -gz;

            return p;
        }

        inline cv::Point2f WorldToScreen(const cv::Point3f &point) {

            const float x = (point.x / (point.z * xz_factor_) + 0.5f) * XtionFrameProperty::kXtionWidth;
            const float y = (point.y / (point.z * yz_factor_) + 0.5f) * XtionFrameProperty::kXtionHeight;
            return cv::Point2f(x, y);
        }

        PointImage ConvertDepthImageToPointImage(DepthImage const &depth_image);

        PointImage ConvertDepthImageToPointImage(DepthImage const &depth_image, int choice);

        bool calibrated_;
        Calibrator calibrator_;
        QFileInfoList file_list_;
        RawDataFrames raw_data_frames_;
        KeyFrames keyframes_;

        float xz_factor_;
        float yz_factor_;
    };


}


#endif //LK_SLAM_IMAGEDATAHANDLER_H
