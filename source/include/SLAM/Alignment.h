//
// Created by LinKun on 9/13/15.
//

#ifndef NIS_MAPCREATOR_H
#define NIS_MAPCREATOR_H

#include <Core/Feature.h>

#include "SLAM/Option.h"
#include "SLAM/KeyFrame.h"
#include "SLAM/Matcher.h"
#include "SLAM/Tracker.h"
#include "SLAM/ComputationResultCache.h"

#include <limits>

namespace {

    using MatricesInfo = std::pair<std::vector<size_t>, std::vector<glm::mat4> >;

};

namespace NiS {


    class SlamComputer : public QObject {

    Q_OBJECT

    public:

        SlamComputer(QObject *parent = 0);

        inline ~SlamComputer() { }

        void SetDataDir(const QDir &data_dir);

        void SetFeatureType(Feature::Type type);

        bool WriteResult(const std::pair<glm::vec3, glm::vec3> &point_pair);

        bool CheckPreviousResult();

        void UsePreviousResult(const QString &result_cache_name);

        Options GetOptions() const { return options_; }

        inline void SetRunningFLag(bool running_flag) { running_flag_ = running_flag; };

    public slots:

        void SetOptions(const Options &options) { options_ = options; };

        inline void SetFrameData(const KeyFrames &keyframes) {

            keyframes_ = keyframes;
            is_data_initialized_ = true;
        }

        void StartCompute();

        void StopCompute();

    public:

        inline const KeyFrames &GetResultKeyFrames() const { return result_keyframes_; }

        inline const KeyFrames &GetKeyFrames() const { return keyframes_; }

        inline bool IsComputationConfigured() const { return is_computation_configured_; }

        inline bool IsDataInitialized() const { return is_data_initialized_; }

    signals:

        void SendData(KeyFrames);

        void Message(QString);

    private:

        template<TrackingType type>
        void ComputeHelper() {

            std::cout << "Computation begins" << std::endl;

            Tracker<type> tracker(keyframes_, options_);

            assert (tracker.GetIterator1() == keyframes_.begin() and tracker.GetIterator2() == keyframes_.begin());

            do {

                result_keyframes_.push_back(tracker.ComputeNext());
                emit Message(tracker.GetMessage());
            }
            while (tracker.Update());

        }

        template<TrackingType type>
        void WriteCache() { }

        bool is_computation_configured_;
        bool is_data_initialized_;

        QDir data_dir_;

        QString result_cache_path_;

        Feature::Type feature_type_;

        Options options_;

        KeyFrames keyframes_;
        KeyFrames result_keyframes_;

        bool running_flag_;

    };

    template<>
    void SlamComputer::WriteCache<TrackingType::OneByOne>();

    template<>
    void SlamComputer::WriteCache<TrackingType::FixedFrameCount>();

    template<>
    void SlamComputer::WriteCache<TrackingType::PcaKeyFrame>();


    // Serialize

    bool LoadMatricesInfo(const std::string &file_name, MatricesInfo &info);

    bool SaveMatricesInfo(const std::string &file_name, const MatricesInfo &info);

}

#endif //NIS_MAPCREATOR_H
