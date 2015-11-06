//
// Created by LinKun on 9/12/15.
//

#ifndef LK_SLAM_FEATURE_H
#define LK_SLAM_FEATURE_H

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <fstream>
#include <QDebug>
#include "Serialize.h"


namespace NiS {

	class Feature
	{
	public:

		enum Type
		{
			kTypeUnknown = -1 ,     ///< 不明（未初期化）
			kTypeORB     = 0 ,      ///< ORB
			kTypeFREAK ,            ///< FREAK
			kTypeSIFT ,             ///< SIFT
			kTypeSURF ,             ///< SURF
		};

		using KeyPoints = std::vector < cv::KeyPoint >;     ///< キーポイント
		using Descriptors = cv::Mat;                        ///< キーポイントディスクリプタ

		Feature ( );

		Feature ( const cv::Mat_ < uchar > & image , Type type );

		~Feature ( );

		Type GetType ( ) const { return type_; }

		const KeyPoints & GetKeyPoints ( ) const { return key_points_; }

		const Descriptors & GetDescriptors ( ) const { return descriptors_; }

	private:

		Type        type_;
		KeyPoints   key_points_;        // キーポイント
		Descriptors descriptors_;       // キーポイントディスクリプタ

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

	// Forward delacration of member template functions.
	// Need to be inplemented in .cpp file.
	// Have a look at the .cpp file for detail implementation.

	// This template impelemtation is just for reference
	template < >
	void Feature::Detect < cv::SiftFeatureDetector , cv::SiftDescriptorExtractor > ( const cv::Mat_ < uchar > & image ,
	                                                                                 KeyPoints * key_points ,
	                                                                                 Descriptors * descriptors );

	bool SaveFeature ( const std::string & name , const Feature & feature );
	bool LoadFeature ( const std::string & name , Feature & feature );

}


#endif //LK_SLAM_FEATURE_H
