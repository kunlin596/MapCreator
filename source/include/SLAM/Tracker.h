//
// Created by LinKun on 9/20/15.
//

#ifndef LK_SLAM_TRACKER_H
#define LK_SLAM_TRACKER_H

#include <QObject>

#include "SLAM/Option.h"
#include "SLAM/CommonDefinitions.h"
#include "SLAM/KeyFrame.h"
#include "SLAM/Transformation.h"
#include "SLAM/CoordinateConverter.h"

#include <iostream>
#include <boost/tuple/tuple.hpp>

namespace NiS {

	CorrespondingPointsPair CreateCorrespondingPointsPair ( const NiS::KeyFrame & key_frame1 , const NiS::KeyFrame & key_frame2 );

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	/*
	 * Total new implementation of Tracker class
	 * 2015.10.26, 10:45 am
	 */
	template < TrackingType type >
	class Tracker
	{
	public:

		using KeyFramesIterator = KeyFrames::iterator;

		Tracker ( ) = default;

		Tracker ( const Options & options ) {

			options_ = options;
		}

		Tracker ( const Tracker & other ) = default;

		Tracker ( const KeyFrames & keyframes , const Options & options , const XtionCoordinateConverter & converter ) {

			options_                    = options;
			keyframes_                  = keyframes;
			xtion_coordinate_converter_ = converter;
			converter_choice_           = 0;
			converter_pointer_          = & xtion_coordinate_converter_;

			assert( !keyframes_.empty ( ) );

			Initialize ( );
		}
		Tracker ( const KeyFrames & keyframes , const Options & options , const AistCoordinateConverter & converter ) {

			options_                   = options;
			keyframes_                 = keyframes;
			aist_coordinate_converter_ = converter;
			converter_choice_          = 1;
			converter_pointer_         = & aist_coordinate_converter_;

			assert( !keyframes_.empty ( ) );

			Initialize ( );
		}

		~Tracker ( ) { }

		bool     Update ( );
		KeyFrame ComputeNext ( );

		inline QString GetMessage ( ) const { return message_; };
		inline const KeyFramesIterator & GetIterator1 ( ) const { return iterator1_; }
		inline const KeyFramesIterator & GetIterator2 ( ) const { return iterator2_; }

	private:

		void Initialize ( );

		KeyFrames keyframes_;

		KeyFramesIterator iterator1_;
		KeyFramesIterator iterator2_;

		Points inliers1_;
		Points inliers2_;

		Options options_;
		QString message_;

		int                      converter_choice_;
		XtionCoordinateConverter xtion_coordinate_converter_;
		AistCoordinateConverter  aist_coordinate_converter_;
		CoordinateConverter * converter_pointer_;

		int offset_;

	};

	template < > bool Tracker < TrackingType::OneByOne >::Update ( );
	template < > bool Tracker < TrackingType::FixedFrameCount >::Update ( );
	template < > bool Tracker < TrackingType::PcaKeyFrame >::Update ( );

	template < > KeyFrame Tracker < TrackingType::OneByOne >::ComputeNext ( );
	template < > KeyFrame Tracker < TrackingType::FixedFrameCount >::ComputeNext ( );
	template < > KeyFrame Tracker < TrackingType::PcaKeyFrame >::ComputeNext ( );

	template < > void Tracker < TrackingType::OneByOne >::Initialize ( );
	template < > void Tracker < TrackingType::FixedFrameCount >::Initialize ( );
	template < > void Tracker < TrackingType::PcaKeyFrame >::Initialize ( );

}


#endif //LK_SLAM_TRACKER_H
