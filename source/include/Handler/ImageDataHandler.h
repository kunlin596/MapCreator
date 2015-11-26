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
#include <SLAM/CoordinateConverter.h>

#include <QFutureWatcher>

#include <memory>

namespace NiS {

	using KeyFramesSharedPtr = std::shared_ptr < KeyFrames >;
	using KeyFramesWeakPtr = std::weak_ptr < KeyFrames >;

	class ImageHandler2 : public QObject
	{

	Q_OBJECT

	public:

		struct XtionFrameProperty
		{
			static const float kXtionHorizontalFOV;
			static const float kXtionVerticalFOV;
			static const float kXtionWidth;
			static const float kXtionHeight;
		};

		inline ImageHandler2 ( QFileInfoList file_list , KeyFramesSharedPtr keyframes_ptr , QObject * parent = 0 ) :
				file_list_ ( file_list ) ,
				xz_factor_ ( GetXtionDepthFactor ( XtionFrameProperty::kXtionHorizontalFOV ) ) ,
				yz_factor_ ( GetXtionDepthFactor ( XtionFrameProperty::kXtionVerticalFOV ) ) {

			raw_data_frames_ptr_ = std::unique_ptr < RawDataFrames > ( new RawDataFrames );
			keyframes_ptr_       = keyframes_ptr;

		}

		inline ~ImageHandler2 ( ) { }
		inline void SetInternalCalibrator ( Calibrator const & calibrator ) { calibrator_ = calibrator; }
		inline RawDataFrames GetRawDataFrames ( ) const { return raw_data_frames_; }
		inline const KeyFrames & GetKeyFrames ( ) const { return keyframes_; }

		static NiS::RawDataFrame ReadFrame ( const QString & file_name );

	signals:

		void SendData ( KeyFrames );
		void DoneReading ( );
		void Message ( QString );

	public slots:

		void StartReading ( );
		void ConvertToPointImages ( int choice );
		void ConvertToPointImagesWithInternalCalibration ( );
		void SetXtionCoordinateConverter ( ) { converter_pointer_ = & xtion_converter_; }
		void SetAistCoordinateConverter ( const AistCoordinateConverter & converter ) {

			aist_converter_    = converter;
			converter_pointer_ = & aist_converter_;
		}

		XtionCoordinateConverter GetXtionCoordinateConverter ( ) const { return xtion_converter_; }
		AistCoordinateConverter GetAistCoordinateConverter ( ) const { return aist_converter_; }

	private:

		void ConvertHelper ( );

		void ConvertHelper ( bool calibrated );

		float GetXtionDepthFactor ( float fov ) {

			return tanf ( fov / 2 ) * 2;
		}

		inline cv::Point3f ScreenToWorld ( int x , int y , float depth ) {

			const float gz = 0.001f * depth;
			cv::Point3f p;
			p.x = gz * xz_factor_ * ( static_cast<float>(x) / XtionFrameProperty::kXtionWidth - 0.5f );
			p.y = -gz * yz_factor_ * ( static_cast<float>(y) / XtionFrameProperty::kXtionHeight - 0.5f );
			p.z = -gz;

			return p;
		}

		inline cv::Point2f WorldToScreen ( const cv::Point3f & point ) {

			const float x = ( point.x / ( point.z * xz_factor_ ) + 0.5f ) * XtionFrameProperty::kXtionWidth;
			const float y = ( point.y / ( point.z * yz_factor_ ) + 0.5f ) * XtionFrameProperty::kXtionHeight;
			return cv::Point2f ( x , y );
		}

		PointImage ConvertDepthImageToPointImage ( DepthImage const & depth_image , int choice );

		bool          calibrated_;
		Calibrator    calibrator_;
		QFileInfoList file_list_;
		RawDataFrames raw_data_frames_;
		KeyFrames     keyframes_;

		std::unique_ptr < RawDataFrames > raw_data_frames_ptr_;
		KeyFramesWeakPtr                  keyframes_ptr_;

		CoordinateConverter * converter_pointer_;
		XtionCoordinateConverter xtion_converter_;
		AistCoordinateConverter  aist_converter_;

		float xz_factor_;
		float yz_factor_;
	};


}


#endif //LK_SLAM_IMAGEDATAHANDLER_H
