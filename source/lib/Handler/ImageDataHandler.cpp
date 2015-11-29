//
// Created by LinKun on 9/13/15.
//

#include "Handler/ImageDataHandler.h"

#include <QThread>
#include <QtConcurrent>

#include <Core/Utility.h>


namespace NiS {

    const float ImageHandler2::XtionFrameProperty::kXtionHorizontalFOV = 1.0225999f;
    const float ImageHandler2::XtionFrameProperty::kXtionVerticalFOV = 0.79661566f;
    const float ImageHandler2::XtionFrameProperty::kXtionWidth = 640;
    const float ImageHandler2::XtionFrameProperty::kXtionHeight = 480;


    NiS::RawDataFrame ImageHandler2::ReadFrame(const QString &file_name) {

        RawDataFrame frame;

        ifstream in(file_name.toStdString(), ios::binary);
        if (in) {

            frame = (NiS::Read<RawDataFrames>(in))[0];
            frame.name = file_name.toStdString();
            frame.id = -1;
        }

        return frame;
    }

    ///////////////////	///////////////////	///////////////////	///////////////////	///////////////////	///////////////////	///////////////////

    void ImageHandler2::StartReading() {

        using namespace std;

        std::cerr << "Handler thread (Reading) : " << QThread::currentThreadId() << std::endl;

        assert(not file_list_.empty());
        emit Message(QString("Start Reading ... "));

        QTime timer;
        timer.start();

        emit SetProgressRange(1, file_list_.size());

        for (auto i = 0; i < file_list_.size(); ++i) {

            auto path = file_list_[i].absoluteFilePath().toStdString();

            ifstream in(path, ios::binary);

            if (in) {

                auto raw_data_frame = NiS::Read<RawDataFrames>(in)[0];
                raw_data_frame.id = i;
                raw_data_frame.name = path;

                raw_data_frames_ptr_->push_back(raw_data_frame);

                auto message = QString("Reading - id : %1, name : %2").arg(raw_data_frame.id)
                        .arg(QString::fromStdString(raw_data_frame.name));

                std::cout << message.toStdString() << std::endl;
                emit Message(message);
                emit SetProgressValue(i + 1);

                in.close();

            } else {

                std::cout << "File open failed : " << path << endl;
            }
        }


        emit Message(QString("Done reading %1 frames. (used %2)")
                             .arg(raw_data_frames_ptr_->size())
                             .arg(ConvertTime(timer.elapsed())));

        emit DoneReading();
    }

    void ImageHandler2::ConvertToPointImages(int choice) {

        std::cout << "Handler thread (Conversion) : " << QThread::currentThreadId() << std::endl;

        if (auto keyframes = keyframes_ptr_.lock()) {

            emit SetProgressRange(1, raw_data_frames_ptr_->size());

            keyframes->clear();

            auto raw_frames = *raw_data_frames_ptr_;

            std::cout << "Data size is " << raw_frames.size() << std::endl;

            for (auto &raw_data_frame : raw_frames) {

                KeyFrame kf;
                kf.SetId(raw_data_frame.id);
                kf.SetName(raw_data_frame.name);
                kf.SetColorImage(raw_data_frame.color_image);
                kf.SetPointImage(std::move(ConvertDepthImageToPointImage(raw_data_frame.depth_image, choice)));

                keyframes->push_back(kf);

                auto message = QString("Converted frame with choice (%1) : %2.").arg(choice).arg(raw_data_frame.id);
                std::cout << message.toStdString() << std::endl;
//                emit Message(message);
                emit SetProgressValue(raw_data_frame.id + 1);
            }
        }

    }

    void ImageHandler2::ConvertToPointImagesWithInternalCalibration() {

        std::cout << "Handler thread (Calibrated conversion) : " << QThread::currentThreadId() << std::endl;

        if (calibrator_.IsValid()) {

            calibrated_ = true;

            keyframes_.clear();

            for (auto i = 0; i < raw_data_frames_.size(); ++i) {

                KeyFrame kf;
                kf.SetId(raw_data_frames_[i].id);
                kf.SetName(raw_data_frames_[i].name);
                kf.SetColorImage(raw_data_frames_[i].color_image);
                kf.SetPointImage(calibrator_.CalibrateImage(raw_data_frames_[i].depth_image));

                keyframes_.push_back(kf);

                emit Message(QString("Converted %1 (Calibrated). KP size : %2")
                                     .arg(kf.GetId())
                                     .arg(kf.GetFeature().GetKeyPoints().size()));
            }

//            emit SendData(keyframes_);
        }
    }

    PointImage ImageHandler2::ConvertDepthImageToPointImage(DepthImage const &depth_image, int choice) {

        const CoordinateConverter *converter;
        switch (choice) {
            case 0:
                converter = &xtion_converter_;
                break;
            case 1:
                converter = &aist_converter_;
                break;
            default:
                return PointImage();
        }

        PointImage point_image;
        point_image.create(depth_image.rows, depth_image.cols);

        for (auto row = 0; row < depth_image.rows; ++row) {
            for (auto col = 0; col < depth_image.cols; ++col) {
                cv::Point3f p = converter->ScreenToWorld(ScreenPoint(col, row), depth_image.at<ushort>(row, col));

                point_image.at<cv::Vec3f>(row, col)(0) = p.x;
                point_image.at<cv::Vec3f>(row, col)(1) = p.y;
                point_image.at<cv::Vec3f>(row, col)(2) = p.z;
            }
        }
        return point_image;
    }

}

