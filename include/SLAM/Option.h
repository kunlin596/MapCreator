//
// Created by LinKun on 10/5/15.
//

#ifndef NIS_OPTION_H
#define NIS_OPTION_H

#include <QString>

#include <Core/Serialize.h>
#include "SLAM/CommonDefinitions.h"

namespace MapCreator {

	struct Options
	{
		struct Options_OneByOne
		{
			int   num_ransac_iteration;
			float threshold_outlier;
			float threshold_inlier;

			inline Options_OneByOne ( ) :
					num_ransac_iteration ( 10000 ) ,
					threshold_outlier ( 0.035f ) ,
					threshold_inlier ( 0.035f ) { }

			inline Options_OneByOne ( int num_ransac_iteration ,
			                          float threshold_outlier ,
			                          float threshold_inlier ) :
					num_ransac_iteration ( num_ransac_iteration ) ,
					threshold_outlier ( threshold_outlier ) ,
					threshold_inlier ( threshold_inlier ) { }

			inline QString Output ( ) const {

				QString res;
				res.append ( QString ( "Parameters of SLAM :\n" ) );
				res.append ( QString ( "----------------------------------\n" ) );
				res.append ( QString ( "Number of RANSAC Iteration : %1\n" ).arg ( QString::number ( num_ransac_iteration ) ) );
				res.append ( QString ( "Threshold of Outlier       : %1\n" ).arg ( QString::number ( threshold_outlier ) ) );
				res.append ( QString ( "Threshold of Inlier        : %1\n" ).arg ( QString::number ( threshold_inlier ) ) );
				res.append ( QString ( "----------------------------------\n" ) );
				return res;
			}

			template < class Archive >
			void serialize ( Archive & ar , const unsigned int version ) {

				ar & num_ransac_iteration;
				ar & threshold_outlier;
				ar & threshold_inlier;

			}

		};

		struct Options_FixedFrameCount : public Options_OneByOne
		{
			int frame_count;

			Options_FixedFrameCount ( ) :
					Options_OneByOne ( ) ,
					frame_count ( 1 ) { }

			explicit Options_FixedFrameCount ( int frame_count ) :
					Options_OneByOne ( ) ,
					frame_count ( frame_count ) { }

			inline QString Output ( ) const {

				QString res;
				res.append ( QString ( "Parameters of Fixed Frame Count Tracking :\n" ) );
				res.append ( QString ( "----------------------------------\n" ) );
				res.append ( QString ( "Number of RANSAC Iteration : %1\n" ).arg ( QString::number ( num_ransac_iteration ) ) );
				res.append ( QString ( "Threshold of Outlier       : %1\n" ).arg ( QString::number ( threshold_outlier ) ) );
				res.append ( QString ( "Threshold of Inlier        : %1\n" ).arg ( QString::number ( threshold_inlier ) ) );
				res.append ( QString ( "Number of frames           : %1\n" ).arg ( QString::number ( frame_count ) ) );
				res.append ( QString ( "----------------------------------\n" ) );

				return res;
			}

			template < class Archive >
			void serialize ( Archive & ar , const unsigned int version ) {

				ar & boost::serialization::base_object < Options_OneByOne > ( * this );
				ar & frame_count;
			}

		};

		struct Options_PcaKeyFrame : public Options_OneByOne
		{
			int   num_inliers;
			float threshold_1st_component_contribution;
			float threshold_1st_component_variance;
			float threshold_2nd_component_variance;
			float threshold_3rd_component_variance;

			inline Options_PcaKeyFrame ( ) :
					Options_OneByOne ( ) ,
					num_inliers ( 3 ) ,
					threshold_1st_component_contribution ( 0.85f ) ,
					threshold_1st_component_variance ( 0.1f ) ,
					threshold_2nd_component_variance ( 0.05f ) ,
					threshold_3rd_component_variance ( 0.0f ) { }

			inline Options_PcaKeyFrame ( int num_inliers ,
			                             float threshold_1st_component_contribution ,
			                             float threshold_1st_component_variance ,
			                             float threshold_2nd_component_variance ,
			                             float threshold_3rd_component_variance ) :
					Options_OneByOne ( ) ,
					num_inliers ( num_inliers ) ,
					threshold_1st_component_contribution ( threshold_1st_component_contribution ) ,
					threshold_1st_component_variance ( threshold_1st_component_variance ) ,
					threshold_2nd_component_variance ( threshold_2nd_component_variance ) ,
					threshold_3rd_component_variance ( threshold_3rd_component_variance ) { }

			inline QString Output ( ) const {

				QString res;
				res.append ( QString ( "Parameters of PCA Key Frame Tracking :\n" ) );
				res.append ( QString ( "----------------------------------\n" ) );
				res.append ( QString ( "Number of RANSAC Iteration : %1\n" ).arg ( QString::number ( num_ransac_iteration ) ) );
				res.append ( QString ( "Threshold of Outlier       : %1\n" ).arg ( QString::number ( threshold_outlier ) ) );
				res.append ( QString ( "Threshold of Inlier        : %1\n\n" ).arg ( QString::number ( threshold_inlier ) ) );
				res.append ( QString ( "Number of inliers          : %1\n" ).arg ( QString::number ( num_inliers ) ) );
				res.append ( QString ( "1st PC contribution        : %1\n" ).arg (
						QString::number ( threshold_1st_component_contribution ) ) );
				res.append ( QString ( "1st PC eigenvalue          : %1\n" ).arg (
						QString::number ( threshold_1st_component_variance ) ) );
				res.append ( QString ( "2nd PC eigenvalue          : %1\n" ).arg (
						QString::number ( threshold_2nd_component_variance ) ) );
				res.append ( QString ( "3rd PC eigenvalue          : %1\n" ).arg (
						QString::number ( threshold_3rd_component_variance ) ) );
				res.append ( QString ( "----------------------------------\n" ) );

				return res;
			}

			template < class Archive >
			void serialize ( Archive & ar , const unsigned int version ) {

				ar & boost::serialization::base_object < Options_OneByOne > ( * this );
				ar & num_inliers;
				ar & threshold_1st_component_contribution;
				ar & threshold_1st_component_variance;
				ar & threshold_2nd_component_variance;
				ar & threshold_3rd_component_variance;
			}
		};

		Options ( ) : type_ ( TrackingType::Unknown ) { }

		Options ( const Options_OneByOne & options_one_by_one , const Options_FixedFrameCount & options_fixed_frame_count ,
		          const Options_PcaKeyFrame & options_pca_keyframe ) :
				type_ ( TrackingType::Unknown ) ,
				options_one_by_one ( options_one_by_one ) ,
				options_fixed_frame_count ( options_fixed_frame_count ) ,
				options_pca_keyframe ( options_pca_keyframe ) { }

		TrackingType            type_;
		bool                    use_bundle_adjustment_;
		Options_OneByOne        options_one_by_one;
		Options_FixedFrameCount options_fixed_frame_count;
		Options_PcaKeyFrame     options_pca_keyframe;

		TrackingType GetType ( ) const { return type_; }

		void UseBundleAdjustment ( bool use ) { use_bundle_adjustment_ = use; }

		bool IsUsingBundleAdjustment ( ) const { return use_bundle_adjustment_; };

		inline void SetOptionsType ( const TrackingType & type ) { type_ = type; }

		template < typename TrackingOptionsType >
		TrackingOptionsType GetOptions ( );

		template < typename Archive >
		void serialize ( Archive & ar , const unsigned int version ) {

			ar & type_;
			ar & use_bundle_adjustment_;
			ar & options_one_by_one;
			ar & options_fixed_frame_count;
			ar & options_pca_keyframe;
		}
	};

	template < >
	Options::Options_OneByOne           Options::GetOptions ( );

	template < >
	Options::Options_FixedFrameCount    Options::GetOptions ( );

	template < >
	Options::Options_PcaKeyFrame        Options::GetOptions ( );

}

#endif //NIS_OPTION_H
