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
#include "SLAM/GlobalOptimization.h"
#include "SLAM/Matcher.h"
#include "SLAM/Transformation.h"
#include "SLAM/GlobalOptimization.h"

#include <Core/Utility.h>
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

		Tracker ( const Options & options ) {

			options_ = options;
		}
		Tracker ( const Tracker & other ) = default;
		Tracker ( std::shared_ptr < KeyFrames > keyframes_ptr , const Options & options , const XtionCoordinateConverter & converter ) {

			options_                    = options;
			keyframes_ptr_              = keyframes_ptr;
			xtion_coordinate_converter_ = converter;
			converter_choice_           = 0;
			converter_pointer_          = & xtion_coordinate_converter_;

			assert ( not keyframes_ptr->empty ( ) );

			Initialize ( );
		}
		Tracker ( std::shared_ptr < KeyFrames > keyframes_ptr , const Options & options , const AistCoordinateConverter & converter ) {

			options_                   = options;
			keyframes_ptr_             = keyframes_ptr;
			aist_coordinate_converter_ = converter;
			converter_choice_          = 1;
			converter_pointer_         = & aist_coordinate_converter_;

			assert ( not keyframes_ptr->empty ( ) );

			Initialize ( );
		}

		Tracker ( ) = default;
		~Tracker ( ) = default;

		bool Update ( );
		void ComputeNext ( );

		QString GetMessage ( ) const { return message_; };
		const KeyFramesIterator & GetIterator1 ( ) const { return iterator1_; }
		const KeyFramesIterator & GetIterator2 ( ) const { return iterator2_; }

	private:

		void Initialize ( );

		std::weak_ptr < KeyFrames > keyframes_ptr_;

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

	template < > void Tracker < TrackingType::OneByOne >::ComputeNext ( );
	template < > void Tracker < TrackingType::FixedFrameCount >::ComputeNext ( );
	template < > void Tracker < TrackingType::PcaKeyFrame >::ComputeNext ( );

	template < > void Tracker < TrackingType::OneByOne >::Initialize ( );
	template < > void Tracker < TrackingType::FixedFrameCount >::Initialize ( );
	template < > void Tracker < TrackingType::PcaKeyFrame >::Initialize ( );

	class AbstractTracker : public QObject
	{
	Q_OBJECT

	signals:

		void SetProgressValue ( int );

	public:

		void SetKeyframes ( std::shared_ptr < KeyFrames > keyframes_ptr ) { keyframes_ptr_ = keyframes_ptr; }
		void SetOptions ( const Options & options ) { options_ = options; }
		void SetConverter ( XtionCoordinateConverter converter ) {

			xtion_converter_ = converter;
			converter_ptr_   = & xtion_converter_;
		}
		void SetConverter ( AistCoordinateConverter converter ) {

			aist_converter_ = converter;
			converter_ptr_  = & aist_converter_;
		}

		using KeyFramesIterator = KeyFrames::iterator;

		virtual void Initialize ( ) { };
		virtual bool Update ( ) { };
		virtual void ComputeNext ( ) { };

	protected:
		std::weak_ptr < KeyFrames > keyframes_ptr_;
		CoordinateConverter * converter_ptr_;
		XtionCoordinateConverter xtion_converter_;
		AistCoordinateConverter  aist_converter_;

		CoordinateConverter converter_;

		KeyFramesIterator iterator1_;
		KeyFramesIterator iterator2_;

		Points inliers1_;
		Points inliers2_;

		Options options_;
		QString message_;
	};


}


#endif //LK_SLAM_TRACKER_H
