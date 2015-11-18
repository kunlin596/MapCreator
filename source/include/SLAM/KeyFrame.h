//
// Created by LinKun on 10/6/15.
//

#ifndef NIS_KEYFRAME_H
#define NIS_KEYFRAME_H

#include <Core/Serialize.h>
#include <Core/Feature.h>

#include "SLAM/Calibrator.h"
#include "SLAM/CommonDefinitions.h"

#include <QDir>
#include <QFileInfo>

namespace NiS {

	class KeyFrame
	{
	public:

		KeyFrame ( const std::string & name , const PointImage & point_image , const ColorImage & color_image ,
		           const Feature::Type & type = Feature::Type::kTypeSIFT ) :
				name_ ( name ) ,
				point_image_ ( point_image ) ,
				color_image_ ( color_image ) ,
				type_ ( type ) ,
				is_used_ ( false ) {

			CreateFeature ( );
		}

		KeyFrame ( ) = default;
		~KeyFrame ( ) = default;

		// Setters
		void SetId ( const int & id ) { id_ = id; }
		void SetName ( const std::string & name ) { name_ = name; }
		void SetColorImage ( const ColorImage & color_image , const Feature::Type & type = Feature::Type::kTypeSIFT ) {

			type_        = type;
			color_image_ = color_image;
			CreateFeature ( );
		}
		void SetPointImage ( const PointImage & point_image ) { point_image_ = point_image; }
		void SetAlignmentMatrix ( const glm::mat4 & mat ) { alignment_matrix_ = mat; }
		void SetAnswerAlignmentMatrix ( const glm::mat4 & mat ) { marker_alignment_matrix_ = mat; }
		void SetUsed ( bool is_used ) { is_used_ = is_used; }

		// Getters
		int GetId ( ) const { return id_; }
		const ColorImage & GetColorImage ( ) const { return color_image_; }
		const PointImage & GetPointImage ( ) const { return point_image_; }
		const std::string & GetName ( ) const { return name_; }
		const NiS::Feature & GetFeature ( ) const { return feature_; }
		const NiS::Feature::Type & GetFeatureType ( ) const { return type_; }
		const glm::mat4 & GetAlignmentMatrix ( ) const { return alignment_matrix_; }
		const glm::mat4 & GetAnswerAlignmentMatrix ( ) const { return marker_alignment_matrix_; }
		const bool IsUsed ( ) const { return is_used_; }

	private: // Private methods

		// Boost serialization methods
		friend class boost::serialization::access;

		BOOST_SERIALIZATION_SPLIT_MEMBER ( );
		template < class Archive > void save ( Archive & ar , const unsigned int version ) const {

			const cv::Mat color = color_image_;
			const cv::Mat point = point_image_;

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

			color_image_ = color;
			point_image_ = point;
			name_        = name;
			feature_     = feature;
		}

		inline void CreateFeature ( ) {

			QFileInfo info ( QString::fromStdString ( name_ ) );

			auto path = info.absolutePath ( );
			auto name = info.completeBaseName ( );


			QString feature_prefix;
			switch ( type_ ) {
				case Feature::Type::kTypeSIFT:
					feature_prefix = "SIFT/sift_";
					break;
				case Feature::Type::kTypeSURF:
					feature_prefix = "SURF/surf_";
					break;
				case Feature::Type::kTypeFREAK:
					feature_prefix = "FREAK/freak_";
					break;
				case Feature::Type::kTypeORB:
					feature_prefix = "ORB/orb_";
					break;
				default:
					break;
			}

			QString feature_folder_path = path + "/Features/";

			QDir dir ( feature_folder_path );
			if ( !dir.exists ( ) ) dir.mkdir ( feature_folder_path );

			if ( !LoadFeature ( ( feature_folder_path + name + "." + "feature" ).toStdString ( ) , feature_ ) ) {
				assert( !color_image_.empty ( ) );
				cv::Mat cvt_color_image;
				cv::cvtColor ( color_image_ , cvt_color_image , cv::COLOR_RGB2GRAY );
				feature_ = NiS::Feature ( cvt_color_image , type_ );
				SaveFeature ( QString ( feature_folder_path + name + ".feature" ).toStdString ( ) , feature_ );
			}
		}

	private: // Fields

		int                id_;
		bool               is_used_;
		glm::mat4          marker_alignment_matrix_;
		glm::mat4          alignment_matrix_;
		std::string        name_;
		NiS::Feature       feature_;
		NiS::Feature::Type type_;
		PointImage         point_image_;
		ColorImage         color_image_;

	};

	using KeyFrames = std::vector < KeyFrame >;

}

#endif //NIS_KEYFRAME_H
