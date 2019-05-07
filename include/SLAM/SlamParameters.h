//
// Created by LinKun on 10/5/15.
//

#ifndef MAPCREATOR_SLAMPARAMETERS_H
#define MAPCREATOR_SLAMPARAMETERS_H

#include <QString>

#include <Core/Serialize.h>

#include "SLAM/CommonDefinitions.h"

namespace MapCreator {

    struct TrackerParameters
    {
        struct Consecutive
        {
            int   num_ransac_iteration;
            float threshold_outlier;
            float threshold_inlier;

            inline Consecutive ( ) :
                    num_ransac_iteration ( 10000 ) ,
                    threshold_outlier ( 0.035f ) ,
                    threshold_inlier ( 0.035f ) { }

            inline Consecutive ( int num_ransac_iteration ,
                                      float threshold_outlier ,
                                      float threshold_inlier ) :
                    num_ransac_iteration ( num_ransac_iteration ) ,
                    threshold_outlier ( threshold_outlier ) ,
                    threshold_inlier ( threshold_inlier ) { }

            inline QString Output ( ) const {

                QString res;
                res.append ( QString ( "TrackerParameters of SLAM :\n" ) );
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

        struct FixedNumber : public Consecutive
        {
            int frame_count;

            FixedNumber ( ) :
                    Consecutive ( ) ,
                    frame_count ( 1 ) { }

            explicit FixedNumber ( int frame_count ) :
                    Consecutive ( ) ,
                    frame_count ( frame_count ) { }

            inline QString Output ( ) const {

                QString res;
                res.append ( QString ( "TrackerParameters of Fixed Frame Count Tracking :\n" ) );
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

                ar & boost::serialization::base_object < Consecutive > ( * this );
                ar & frame_count;
            }

        };

        struct KeyFrameOnly : public Consecutive
        {
            int   num_inliers;
            float threshold_1st_component_contribution;
            float threshold_1st_component_variance;
            float threshold_2nd_component_variance;
            float threshold_3rd_component_variance;

            inline KeyFrameOnly ( ) :
                    Consecutive ( ) ,
                    num_inliers ( 3 ) ,
                    threshold_1st_component_contribution ( 0.85f ) ,
                    threshold_1st_component_variance ( 0.1f ) ,
                    threshold_2nd_component_variance ( 0.05f ) ,
                    threshold_3rd_component_variance ( 0.0f ) { }

            inline KeyFrameOnly ( int num_inliers ,
                                         float threshold_1st_component_contribution ,
                                         float threshold_1st_component_variance ,
                                         float threshold_2nd_component_variance ,
                                         float threshold_3rd_component_variance ) :
                    Consecutive ( ) ,
                    num_inliers ( num_inliers ) ,
                    threshold_1st_component_contribution ( threshold_1st_component_contribution ) ,
                    threshold_1st_component_variance ( threshold_1st_component_variance ) ,
                    threshold_2nd_component_variance ( threshold_2nd_component_variance ) ,
                    threshold_3rd_component_variance ( threshold_3rd_component_variance ) { }

            inline QString Output ( ) const {

                QString res;
                res.append ( QString ( "TrackerParameters of PCA Key Frame Tracking :\n" ) );
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

                ar & boost::serialization::base_object < Consecutive > ( * this );
                ar & num_inliers;
                ar & threshold_1st_component_contribution;
                ar & threshold_1st_component_variance;
                ar & threshold_2nd_component_variance;
                ar & threshold_3rd_component_variance;
            }
        };

        TrackerParameters ( ) : type_ ( TrackingType::Unknown ) { }

        TrackerParameters ( const Consecutive & paramsConsectutive , const FixedNumber & paramsFixedNumber ,
                  const KeyFrameOnly & paramsKeyFramesOnly ) :
                type_ ( TrackingType::Unknown ) ,
                paramsConsectutive ( paramsConsectutive ) ,
                paramsFixedNumber ( paramsFixedNumber ) ,
                paramsKeyFramesOnly ( paramsKeyFramesOnly ) { }

        TrackingType            type_;
        bool                    use_bundle_adjustment_;
        Consecutive        paramsConsectutive;
        FixedNumber paramsFixedNumber;
        KeyFrameOnly     paramsKeyFramesOnly;

        TrackingType GetType ( ) const { return type_; }

        void UseBundleAdjustment ( bool use ) { use_bundle_adjustment_ = use; }

        bool IsUsingBundleAdjustment ( ) const { return use_bundle_adjustment_; };

        inline void SetOptionsType ( const TrackingType & type ) { type_ = type; }

        template < typename TrackingOptionsType >
        TrackingOptionsType GetParameters ( );

        template < typename Archive >
        void serialize ( Archive & ar , const unsigned int version ) {

            ar & type_;
            ar & use_bundle_adjustment_;
            ar & paramsConsectutive;
            ar & paramsFixedNumber;
            ar & paramsKeyFramesOnly;
        }
    };

    template < >
    TrackerParameters::Consecutive TrackerParameters::GetParameters ( );

    template < >
    TrackerParameters::FixedNumber TrackerParameters::GetParameters ( );

    template < >
    TrackerParameters::KeyFrameOnly TrackerParameters::GetParameters ( );
}

#endif //MAPCREATOR_SLAMPARAMETERS_H
