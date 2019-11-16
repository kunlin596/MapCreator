//
// Created by LinKun on 9/13/15.
//

#ifndef MAPCREATOR_MAPCREATOR_H
#define MAPCREATOR_MAPCREATOR_H

// #include <QObject>

#include <limits>

#include "Core/Feature.h"
#include "SLAM/ComputationResultCache.h"
#include "SLAM/CoordinateConverter.h"
#include "SLAM/KeyFrame.h"
#include "SLAM/Matcher.h"
#include "SLAM/SlamParameters.h"
#include "SLAM/Tracker.h"

namespace {
using MatricesInfo = std::pair<std::vector<size_t>, std::vector<glm::mat4> >;
};

namespace MapCreator {

class SlamAlgorithm {
  // class SlamAlgorithm : public QObject {
  // Q_OBJECT

 public:
  SlamAlgorithm() = default;
  // SlamAlgorithm(QObject* parent = 0) {}

  void SetFeatureType(Feature::Type type);
  bool WriteResult(const std::pair<glm::vec3, glm::vec3>& point_pair);
  bool WriteResult();
  bool CheckPreviousResult();
  void UsePreviousResult(const std::string& result_cache_name);
  void SetRunningFLag(bool running_flag) { running_flag_ = running_flag; };
  TrackerParameters GetParameters() const { return params_; }

 public slots:

  void SetOptions(const TrackerParameters& options) { params_ = options; };
  void SetFrameData(const KeyFrames& keyframes) {
    keyframes_ = keyframes;
    is_data_initialized_ = true;
    has_answer_ = false;
  }
  void SetCoordinateConverter(const XtionCoordinateConverter& converter) {
    xtion_converter_ = converter;
    converter_choice_ = 0;
  }
  void SetCoordinateConverter(const AistCoordinateConverter& converter) {
    aist_converter_ = converter;
    converter_choice_ = 1;
  }
  void StartCompute();
  void StopCompute();
  void StartGenerateAnswer();

 public:
  const KeyFrames& GetKeyFrames() const { return keyframes_; }
  bool IsComputationConfigured() const { return is_computation_configured_; }
  bool IsDataInitialized() const { return is_data_initialized_; }

  // signals:

  //  void SendData(KeyFrames);
  //  void Message(QString);

 private:
  template <TrackingType type>
  void ComputeHelper() {
    std::cout << "Computation begins" << std::endl;
    switch (converter_choice_) {
      case 0: {
        Tracker<type> tracker1(keyframes_, params_, xtion_converter_);
        do {
          tracker1.SpinOnce();
          // emit Message ( tracker1.GetMessage ( ) );
        } while (tracker1.Update());
        keyframes_ = tracker1.GetResults();
        break;
      }
      case 1: {
        Tracker<type> tracker2(keyframes_, params_, aist_converter_);
        do {
          tracker2.SpinOnce();
          // emit Message ( tracker2.GetMessage ( ) );
        } while (tracker2.Update());
        keyframes_ = tracker2.GetResults();
        break;
      }
      default:
        break;
    }
  }

  void WriteCache(int computation_time, QString suffix = "NoMarker") {
    QDir dir(data_dir_.absolutePath() + "/Cache");
    if (!dir.exists()) dir.mkdir(data_dir_.absolutePath() + "/Cache");
    result_cache_path_ = dir.absolutePath();

    QString cache_file_name = QString("%1/%2-%3-%4.cache")
                                  .arg(result_cache_path_)
                                  .arg(time(nullptr))
                                  .arg(static_cast<int>(params_.GetType()))
                                  .arg(suffix);

    ComputationResultCache cache;

    cache.data_set_name = data_dir_.absolutePath().toStdString();
    cache.computation_time = computation_time;
    cache.options = params_;

    for_each(std::begin(keyframes_), std::end(keyframes_),
             [&cache](const KeyFrame& keyframe) -> void {
               cache.indices.push_back(std::move(keyframe.GetId()));
               cache.used_status.push_back(keyframe.IsUsed());
               cache.estimation_matrices.push_back(
                   std::move(keyframe.GetAlignmentMatrix()));
               cache.marker_matrices.push_back(
                   std::move(keyframe.GetAnswerAlignmentMatrix()));
             });

    try {
      SaveComputationResultCache(cache_file_name.toStdString(), cache);

    } catch (const boost::archive::archive_exception& e) {
      // emit Message ( e.what ( ) );
      return;
    }
  }

  bool is_computation_configured_;
  bool is_data_initialized_;

  // QString result_cache_path_;
  Feature::Type feature_type_;
  AlgorithmParameters algorithm_parameters;
  KeyFrames keyframes_;
  int converter_choice_;
  XtionCoordinateConverter xtion_converter_;
  AistCoordinateConverter aist_converter_;
  bool running_flag_;
  bool has_answer_;
  std::vector<std::pair<Points, Points> > all_markers_points_pairs_;
};

//  template < > void SlamAlgorithm::WriteCache < TrackingType::Consecutive > (
//  ); template < > void SlamAlgorithm::WriteCache < TrackingType::FixedNumber >
//  ( ); template < > void SlamAlgorithm::WriteCache <
//  TrackingType::KeyFrameOnly > ( );

// Serialize
bool LoadMatricesInfo(const std::string& file_name, MatricesInfo& info);
bool SaveMatricesInfo(const std::string& file_name, const MatricesInfo& info);

}  // namespace MapCreator

#endif  // MAPCREATOR_MAPCREATOR_H
