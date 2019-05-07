//
// Created by LinKun on 10/6/15.
//

#ifndef MAPCREATOR_KEYFRAME_H
#define MAPCREATOR_KEYFRAME_H

#include "Core/Serialize.h"
#include "Core/Feature.h"

#include "SLAM/Calibrator.h"
#include "SLAM/SLAM.h"
#include "Core/Frame.h"

#include <QDir>
#include <QFileInfo>

namespace MapCreator {

	struct KeyFrame: public RGBDFrame
	{
	public:

		KeyFrame ( const std::string & name , const PointImage & pointimage , const ColorImage & colorimage ,
		           const Feature::Type & type = Feature::Type::kTypeORB)
		{
			name_ = name;
			this->pointimage = pointimage;
			this->colorimage = colorimage;
			this->type_ = type;
			this->is_used_ = false;
			CreateFeature ( );
		}

		KeyFrame ( ) = default;
		~KeyFrame ( ) = default;

		// Setters
		void SetId ( const int & id ) { id_ = id; }
		void SetName ( const std::string & name ) { name_ = name; }
		void SetColorImage ( const ColorImage & colorimage , const Feature::Type & type = Feature::Type::kTypeORB ) {

			type_        = type;
			this->colorimage = colorimage;
			CreateFeature ( );
		}
		void SetPointImage ( const PointImage & pointimage ) { this->pointimage = pointimage; }
		void SetAlignmentMatrix ( const glm::mat4 & mat ) { alignment_matrix_ = mat; }
		void SetAnswerAlignmentMatrix ( const glm::mat4 & mat ) { marker_alignment_matrix_ = mat; }
		void SetUsed ( bool is_used ) { is_used_ = is_used; }

		// Getters
		int GetId ( ) const { return id_; }
		const ColorImage & GetColorImage ( ) const { return colorimage; }
		const PointImage & GetPointImage ( ) const { return pointimage; }
		const std::string & GetName ( ) const { return name_; }
		const MapCreator::Feature & GetFeature ( ) const { return feature_; }
		const MapCreator::Feature::Type & GetFeatureType ( ) const { return type_; }
		const glm::mat4 & GetAlignmentMatrix ( ) const { return alignment_matrix_; }
		const glm::mat4 & GetAnswerAlignmentMatrix ( ) const { return marker_alignment_matrix_; }
		const bool IsUsed ( ) const { return is_used_; }

	private: // Private methods

		// Boost serialization methods
		friend class boost::serialization::access;

		BOOST_SERIALIZATION_SPLIT_MEMBER ( );
		template < class Archive > void save ( Archive & ar , const unsigned int version ) const {

			const cv::Mat color = colorimage;
			const cv::Mat point = pointimage;

			ar & color;
			ar & point;
			ar & name_;
			ar & feature_;

		}
		template < class Archive > void load ( Archive & ar , const unsigned int version ) {

			cv::Mat     color;
			cv::Mat     point;
			std::string name;
			Feature     feature;

			ar & color;
			ar & point;
			ar & name;
			ar & feature;

			colorimage = color;
			pointimage = point;
			name_        = name;
			feature_     = feature;
		}

		inline void CreateFeature ( ) {

			QFileInfo info ( QString::fromStdString ( name_ ) );

			auto path = info.absolutePath ( );
			auto name = info.completeBaseName ( );


			QString feature_prefix;
			switch ( type_ ) {
				case Feature::Type::kTypeORB:
					feature_prefix = "ORB/orb_";
					break;
#ifdef ENABLE_OPENCV_CONTRIB
				case Feature::Type::kTypeSIFT:
					feature_prefix = "SIFT/sift_";
					break;
				case Feature::Type::kTypeSURF:
					feature_prefix = "SURF/surf_";
					break;
				case Feature::Type::kTypeFREAK:
					feature_prefix = "FREAK/freak_";
					break;
#endif
				default:
					break;
			}

			QString feature_folder_path = path + "/Features/";

			QDir dir ( feature_folder_path );
			if ( !dir.exists ( ) ) dir.mkdir ( feature_folder_path );

			if ( !LoadFeature ( ( feature_folder_path + name + "." + "feature" ).toStdString ( ) , feature_ ) ) {
				assert( !colorimage.empty ( ) );
				cv::Mat cvt_color_image;
				cv::cvtColor ( colorimage , cvt_color_image , cv::COLOR_RGB2GRAY );
				feature_ = MapCreator::Feature ( cvt_color_image , type_ );
				SaveFeature ( QString ( feature_folder_path + name + ".feature" ).toStdString ( ) , feature_ );
			}
		}

	private: // Fields

		int                id_;
		bool               is_used_;
		glm::mat4          marker_alignment_matrix_;
		glm::mat4          alignment_matrix_;
		std::string        name_;
		MapCreator::Feature feature_;
		MapCreator::Feature::Type type_;

		PointImage pointimage;

	};

	using KeyFrames = std::vector < KeyFrame >;

}

#endif //MAPCREATOR_KEYFRAME_H
