//
// Created by LinKun on 9/12/15.
//

#ifndef MAPCREATOR_FEATURE_H
#define MAPCREATOR_FEATURE_H

#include "Serialize.h"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#ifdef ENABLE_OPENCV_CONTRIB
#include <opencv2/xfeatures2d.hpp>
#endif

#include <fstream>

#include <QDebug>

namespace MapCreator {

	class Feature
	{
	public:

		enum Type
		{
			kTypeUnknown = -1 ,     ///< Unknown
			kTypeORB     = 0 ,      ///< ORB
#ifdef ENABLE_OPENCV_CONTRIB
			kTypeFREAK ,            ///< FREAK
			kTypeSIFT ,             ///< SIFT
			kTypeSURF ,             ///< SURF
#endif
		};

		using KeyPoints = std::vector < cv::KeyPoint >;     ///< Keypoints
		using Descriptors = cv::Mat;                        ///< Keypoint descriptors

		Feature ( );

		Feature ( const cv::Mat_ < uchar > & image , Type type );

		~Feature ( );

		Type GetType ( ) const { return type_; }

		const KeyPoints & GetKeyPoints ( ) const { return key_points_; }

		const Descriptors & GetDescriptors ( ) const { return descriptors_; }

	private:

		Type        type_;
		KeyPoints   key_points_;        ///< Keypoints
		Descriptors descriptors_;       ///< Keypoint descriptors
                cv::Ptr<cv::FeatureDetector> detector_;

		template < class Detector , class Extractor >
		void Detect ( const cv::Mat_ < uchar > & image , KeyPoints * key_points , Descriptors * descriptors ) {

			if ( !image.empty ( ) ) {

				// detecting keypoints
				Detector detector;
				detector.detect ( image , * key_points );

				// computing descriptors
				Extractor extractor;
				extractor.compute ( image , * key_points , * descriptors );
			}
		}

		friend class boost::serialization::access;

		BOOST_SERIALIZATION_SPLIT_MEMBER ( );

		template < class Archive >
		void save ( Archive & ar , const unsigned int version ) const {
			const cv::Mat m = descriptors_;
			ar & type_;
			ar & key_points_;
			ar & m;
		}

		template < class Archive >
		void load ( Archive & ar , const unsigned int version ) {
			cv::Mat m;
			ar & type_;
			ar & key_points_;
			ar & m;
			descriptors_ = m;
		}

	};

	bool SaveFeature ( const std::string & name , const Feature & feature );
	bool LoadFeature ( const std::string & name , Feature & feature );

}


#endif //MAPCREATOR_FEATURE_H
