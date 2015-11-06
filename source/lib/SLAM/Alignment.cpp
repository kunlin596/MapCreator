//
// Created by LinKun on 9/13/15.
//
#include "SLAM/Alignment.h"
#include "SLAM/Calibrator.h"
#include "SLAM/Transformation.h"

#include <limits>

#include <Core/Serialize.h>
#include <Core/Utility.h>
#include <Core/MyMath.h>

#include <boost/tuple/tuple.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <QMessageBox>
#include <QMap>
#include <QFileDialog>
#include <QThread>
#include <QTime>
#include <QWaitCondition>

namespace NiS {

    SlamComputer::SlamComputer(QObject *parent) :
            running_flag_(true),
            is_computation_configured_(false),
            is_data_initialized_(false) {

    }

    void SlamComputer::SetDataDir(const QDir &data_dir) {

        data_dir_ = data_dir;
    }

    void SlamComputer::StartCompute() {

        running_flag_ = true;

        std::cout << "SlamComputer thread : " << QThread::currentThreadId() << std::endl;
        std::cout << "SlamComputer thread - data size : " << keyframes_.size() << std::endl;

        if (keyframes_.size() < 2) {
            emit SendData(keyframes_);
            return;
        }

        result_keyframes_.clear();

        QTime timer;
        timer.start();

        emit Message("Computation begins...");

        switch (options_.type_) {
            case TrackingType::OneByOne:
                ComputeHelper<TrackingType::OneByOne>();
                break;
            case TrackingType::FixedFrameCount:
                ComputeHelper<TrackingType::FixedFrameCount>();
                break;
            case TrackingType::PcaKeyFrame:
                ComputeHelper<TrackingType::PcaKeyFrame>();
                break;
            case TrackingType::Unknown:
                emit Message("Setup computation options at first.");
                return;
        }

        emit Message(QString("Done computing %1 frames. (used %2)")
                             .arg(keyframes_.size())
                             .arg(ConvertTime(timer.elapsed())));

        emit SendData(result_keyframes_);

        QDir dir(data_dir_.absolutePath() + "/Cache");
        if (!dir.exists()) dir.mkdir(data_dir_.absolutePath() + "/Cache");
        result_cache_path_ = dir.absolutePath();

        QString cache_file_name = QString("%1/%2-%3.cache")
                .arg(result_cache_path_)
                .arg(time(nullptr))
                .arg(static_cast<int> ( options_.GetType()));

        ComputationResultCache cache;

        cache.data_set_name = data_dir_.absolutePath().toStdString();
        cache.computation_time = timer.elapsed();
        cache.options = options_;

        for_each(std::begin(result_keyframes_), std::end(result_keyframes_),
                 [&cache](const KeyFrame &keyframe) -> void {
                     cache.indices.push_back(std::move(keyframe.GetId()));
                     cache.matrices.push_back(std::move(keyframe.GetAlignmentMatrix()));
                 });

        try {

            SaveComputationResultCache(cache_file_name.toStdString(), cache);

        }
        catch (const boost::archive::archive_exception &e) {

            emit Message(e.what());
            return;

        }

    }

    bool SlamComputer::WriteResult(const std::pair<glm::vec3, glm::vec3> &point_pair) {

        time_t time_stamp = time(nullptr);

        QString result_name_prefix;

        std::cout << options_.type_ << std::endl;

        switch (options_.type_) {
            case TrackingType::OneByOne:
                result_name_prefix = QString("%1_%2").arg(time_stamp).arg("OneByOne");
                break;
            case TrackingType::PcaKeyFrame:
                result_name_prefix = QString("%1_%2").arg(time_stamp).arg("PcaKeyFrame");
                break;
            case TrackingType::FixedFrameCount:
                result_name_prefix = QString("%1_%2").arg(time_stamp).arg("FixedFrameCount");
                break;
            case TrackingType::Unknown:
                emit Message("No result to be written.");
                return false;

        }

        QString data_folder_path = data_dir_.absolutePath();
        QDir dir(data_folder_path + "/Result");
        if (!dir.exists()) dir.mkdir(data_folder_path + "/Result");

        std::string result_file_name = QString(
                data_folder_path + "/Result/" + result_name_prefix + ".txt").toStdString();
        std::ofstream out(result_file_name);

        if (out) {
            out << "Current tracker type : " << static_cast<int>(options_.type_) << std::endl;

            switch (options_.type_) {

                case TrackingType::OneByOne: {
                    out << options_.options_one_by_one.Output().toStdString() << std::endl;
                    break;
                }
                case TrackingType::PcaKeyFrame: {
                    out << options_.options_pca_keyframe.Output().toStdString() << std::endl;
                    break;
                }
                case TrackingType::FixedFrameCount : {
                    out << options_.options_fixed_frame_count.Output().toStdString() << std::endl;
                    break;
                }
                default:
                    break;
            }

            out << "point1 " << glm::vec4(point_pair.first, 1.0f);
            out << "point2 " << glm::vec4(point_pair.second, 1.0f);

            out.close();

            return true;
        }

        return false;
    }

    bool SlamComputer::CheckPreviousResult() {

        std::cout << "Checking privious result." << std::endl;

        result_cache_path_ = data_dir_.absolutePath() + "/Cache";

        QDir dir(result_cache_path_);
        if (!dir.exists()) {
            dir.mkdir(result_cache_path_);
            return false;
        }

        QStringList filter_list;
        filter_list.push_back(QString("*.cache"));

        return !dir.entryInfoList(filter_list).empty();
    }

    void SlamComputer::UsePreviousResult(const QString &result_cache_name) {

        if (keyframes_.empty()) {
            emit Message("No data found, cannot apply matrices.");
        }

        result_keyframes_.clear();

        QTime timer;
        timer.start();

        ComputationResultCache cache;

        bool load_succeeded = LoadComputationResultCache(result_cache_name.toStdString(), cache);
        if (!load_succeeded) { return; }

        auto data_set_name = cache.data_set_name;
        auto computation_time = cache.computation_time;
        options_ = cache.options;

        for (auto i = 0; i < cache.indices.size(); ++i) {
            auto id = cache.indices[i];
            auto matrix = cache.matrices[i];

            auto kf = keyframes_[id];

            kf.SetId(id);
            kf.SetAlignmentMatrix(matrix);

            result_keyframes_.push_back(kf);
        }

        emit SendData(result_keyframes_);
        emit Message(QString("Done loading %1 frames' results. (used %2)")
                             .arg(keyframes_.size())
                             .arg(ConvertTime(timer.elapsed())));
    }

    bool SaveMatricesInfo(const std::string &file_name, const MatricesInfo &info) {

        std::ofstream out(file_name, std::ios::binary);

        if (out) {
            namespace bio = ::boost::iostreams;

            bio::filtering_ostream f;
            f.push(bio::gzip_compressor());
            f.push(out);

            boost::archive::binary_oarchive ar(out);
            ar << info;

            return true;
        }

        return false;
    }

    bool LoadMatricesInfo(const std::string &file_name, MatricesInfo &info) {

        std::ifstream in(file_name, std::ios::binary);

        if (in) {
            namespace bio = ::boost::iostreams;

            bio::filtering_istream f;
            f.push(bio::gzip_decompressor());
            f.push(in);

            boost::archive::binary_iarchive ar(in);
            ar >> info;

            return true;
        }

        return false;
    }

    void SlamComputer::StopCompute() {

        running_flag_ = false;
    }

}

