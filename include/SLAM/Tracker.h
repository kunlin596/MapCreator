//
// Created by LinKun on 9/20/15.
//

#ifndef MAPCREATOR_TRACKER_H
#define MAPCREATOR_TRACKER_H

#include <QObject>

#include "SLAM/SlamParameters.h"
#include "SLAM/CommonDefinitions.h"
#include "SLAM/KeyFrame.h"
#include "SLAM/Transformation.h"
#include "SLAM/CoordinateConverter.h"

#include <Core/Utility.h>

#include <iostream>
#include <boost/tuple/tuple.hpp>

namespace MapCreator {

	CorrespondingPointsPair CreateCorrespondingPointsPair ( const MapCreator::KeyFrame & key_frame1 , const MapCreator::KeyFrame & key_frame2 );

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

		Tracker ( const Parameters & options ) {

			params_ = options;
		}
		Tracker ( const Tracker & other ) = default;
		Tracker ( const KeyFrames & keyframes , const Parameters & options , const XtionCoordinateConverter & converter ) {

			params_                    = options;
			keyframes_                  = keyframes;
			xtion_coordinate_converter_ = converter;
			converter_choice_           = 0;
			converter_pointer_          = & xtion_coordinate_converter_;

			assert( !keyframes_.empty ( ) );

			Initialize ( );
		}
		Tracker ( const KeyFrames & keyframes , const Parameters & options , const AistCoordinateConverter & converter ) {

			params_                   = options;
			keyframes_                 = keyframes;
			aist_coordinate_converter_ = converter;
			converter_choice_          = 1;
			converter_pointer_         = & aist_coordinate_converter_;

			assert( !keyframes_.empty ( ) );

			Initialize ( );
		}

		Tracker ( ) = default;
		~Tracker ( ) = default;

		bool Update ( );
		void ComputeNext ( );

		QString GetMessage ( ) const { return message_; };
		const KeyFramesIterator & GetIterator1 ( ) const { return iterator1_; }
		const KeyFramesIterator & GetIterator2 ( ) const { return iterator2_; }
		const KeyFrames GetResults ( ) const { return keyframes_; }

	private:

		void Initialize ( );

		KeyFrames keyframes_;

		KeyFramesIterator iterator1_;
		KeyFramesIterator iterator2_;

		Points inliers1_;
		Points inliers2_;

		Parameters params_;
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

	template < > void Tracker < TrackingType::OneByOne >::ComputeNext ( );
	template < > void Tracker < TrackingType::FixedFrameCount >::ComputeNext ( );
	template < > void Tracker < TrackingType::PcaKeyFrame >::ComputeNext ( );

	template < > void Tracker < TrackingType::OneByOne >::Initialize ( );
	template < > void Tracker < TrackingType::FixedFrameCount >::Initialize ( );
	template < > void Tracker < TrackingType::PcaKeyFrame >::Initialize ( );

}


#endif //MAPCREATOR_TRACKER_H
