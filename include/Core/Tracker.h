//
// Created by LinKun on 9/20/15.
//

#ifndef MAPCREATOR_TRACKER_H
#define MAPCREATOR_TRACKER_H

#include <QObject>

#include "Core/CoordinateConverter.h"
#include "Core/KeyFrame.h"
#include "Core/SLAM.h"
#include "Core/SlamParameters.h"
#include "Core/Transformation.h"

#include <Core/Utility.h>

#include <boost/tuple/tuple.hpp>
#include <iostream>

namespace MapCreator {

CorrespondingPointsPair CreateCorrespondingPointsPair(
    const MapCreator::KeyFrame& key_frame1,
    const MapCreator::KeyFrame& key_frame2);

template <TrackingType type>
class Tracker {
 public:
  using KeyFramesIterator = KeyFrames::iterator;

  Tracker(const TrackerParameters& params) { params_ = params; }
  Tracker(const Tracker& other) = default;
  Tracker(const KeyFrames& keyframes, const TrackerParameters& params,
          const CoordinateConverter& converter) {
    params_ = params;
    keyframes_ = keyframes;
    xtion_coordinate_converter_ = converter;
    converter_choice_ = 0;
    converter_pointer_ = &xtion_coordinate_converter_;

    assert(!keyframes_.empty());

    Initialize();
  }

  Tracker(const KeyFrames& keyframes, const TrackerParameters& params,
          const CalibratedCoordinateConverter& converter) {
    params_ = params;
    keyframes_ = keyframes;
    aist_coordinate_converter_ = converter;
    converter_choice_ = 1;
    converter_pointer_ = &aist_coordinate_converter_;

    assert(!keyframes_.empty());

    Initialize();
  }

  Tracker() = default;
  ~Tracker() = default;

  bool Update();
  void SpinOnce();

  QString GetMessage() const { return message_; };
  const KeyFramesIterator& GetIterator1() const { return iterator1_; }
  const KeyFramesIterator& GetIterator2() const { return iterator2_; }
  const KeyFrames GetResults() const { return keyframes_; }

 private:
  void Initialize();

  KeyFrames keyframes_;

  KeyFramesIterator iterator1_;
  KeyFramesIterator iterator2_;

  Points inliers1_;
  Points inliers2_;

  TrackerParameters params_;
  QString message_;

  int converter_choice_;
  CoordinateConverter xtion_coordinate_converter_;
  CalibratedCoordinateConverter aist_coordinate_converter_;
  AbstractCoordinateConverter* converter_pointer_;

  int offset_;
};

template <>
bool Tracker<TrackingType::Consecutive>::Update();
template <>
bool Tracker<TrackingType::FixedNumber>::Update();
template <>
bool Tracker<TrackingType::KeyFrameOnly>::Update();

template <>
void Tracker<TrackingType::Consecutive>::SpinOnce();
template <>
void Tracker<TrackingType::FixedNumber>::SpinOnce();
template <>
void Tracker<TrackingType::KeyFrameOnly>::SpinOnce();

template <>
void Tracker<TrackingType::Consecutive>::Initialize();
template <>
void Tracker<TrackingType::FixedNumber>::Initialize();
template <>
void Tracker<TrackingType::KeyFrameOnly>::Initialize();

}  // namespace MapCreator

#endif  // MAPCREATOR_TRACKER_H
