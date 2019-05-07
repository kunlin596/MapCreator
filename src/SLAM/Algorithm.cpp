//
// Created by LinKun on 9/13/15.
//

#include "Core/Logger.h"
#include "Core/Serialize.h"
#include "Core/Utility.h"
#include "Core/MyMath.h"

#include "SLAM/Algorithm.h"
#include "SLAM/Calibrator.h"
#include "SLAM/Transformation.h"
#include "SLAM/GlobalOptimization.h"
//#include "SLAM/ArucoMarkerUtils.h"

#include <limits>

#include <boost/tuple/tuple.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/format.hpp>

#include <QMessageBox>
#include <QMap>
#include <QFileDialog>
#include <QThread>
#include <QTimer>
#include <QTime>
#include <QWaitCondition>

LOGGER("SLAM.Algorithm");

namespace MapCreator {

    // SlamAlgorithm::SlamAlgorithm ( QObject * parent ) :
    //         running_flag_ ( true ) ,
    //         has_answer_ ( false ) ,
    //         is_computation_configured_ ( false ) ,
    //         is_data_initialized_ ( false ) {

    // }

    void SlamAlgorithm::SetDataDir ( const QDir & data_dir ) {

        data_dir_ = data_dir;
    }

    void SlamAlgorithm::StartCompute ( ) {

        running_flag_ = true;

        LOG_INFO(boost::str(boost::format("thread id: %1%, data size: %2%") % QThread::currentThreadId () % keyframes_.size ()));

        if ( keyframes_.size ( ) < 2 ) {
            emit SendData ( keyframes_ );
            return;
        }

        QTimer timer;
        timer.start ( );

        emit Message ( "Computation begins..." );

        switch ( params_.type_ ) {
            case TrackingType::Consecutive:
                ComputeHelper < TrackingType::Consecutive > ( );
                break;
            case TrackingType::FixedNumber:
                ComputeHelper < TrackingType::FixedNumber > ( );
                break;
            case TrackingType::KeyFrameOnly:
                ComputeHelper < TrackingType::KeyFrameOnly > ( );
                break;
            case TrackingType::Unknown:
                emit Message ( "Setup computation options at first." );
                return;
        }

        // emit Message ( QString ( "Done computing %1 frames. (used %2)" )
        //                         .arg ( keyframes_.size ( ) )
        //                         .arg ( ConvertTime ( timer.elapsed ( ) ) ) );

        emit SendData ( keyframes_ );

     // if ( has_answer_ ) WriteCache ( timer.elapsed ( ) , "WithAnswer" );
     // else WriteCache ( timer.elapsed ( ) );
    }

    void SlamAlgorithm::StartGenerateAnswer ( ) {

        if ( keyframes_.empty ( ) ) {
            return;
        }

        if ( keyframes_.size ( ) < 2 ) {
            emit SendData ( keyframes_ );
            return;
        }

//        aruco::MarkerDetector marker_detector;

        QTimer timer;
        timer.start ( );

        all_markers_points_pairs_.clear ( );

        for ( auto i = 1 ; i < keyframes_.size ( ) ; ++i ) {

            auto & keyframe1 = keyframes_[ i - 1 ];
            auto & keyframe2 = keyframes_[ i ];

//            Markers markers1;
//            Markers markers2;

//            marker_detector.detect ( keyframe1.GetColorImage ( ) , markers1 );
//            marker_detector.detect ( keyframe2.GetColorImage ( ) , markers2 );

            Points points1;
            Points points2;

//            boost::tie ( points1 , points2 ) = ArucoMarkerUtils::CreatePoints ( markers1 , markers2 , keyframe1 , keyframe2 );

//            all_markers_points_pairs_.push_back ( std::make_pair ( points1 , points2 ) );

//            auto matrix = ComputeTransformationMatrix ( points2 , points1 );

//            keyframe2.SetAnswerAlignmentMatrix ( std::move ( ConvertCVMatx44fToGLMmat4 ( matrix ) ) );
        }

        // emit Message ( QString ( "Done generating answers of  %1 frames. (used %2)" )
        //                         .arg ( keyframes_.size ( ) )
        //                         .arg ( ConvertTime ( timer.elapsed ( ) ) ) );

        emit SendData ( keyframes_ );

        has_answer_ = true;
    }

    bool SlamAlgorithm::WriteResult ( const std::pair < glm::vec3 , glm::vec3 > & point_pair ) {

        time_t time_stamp = time ( nullptr );

        QString result_name_prefix;

        std::cout << params_.type_ << std::endl;

        switch ( params_.type_ ) {
            case TrackingType::Consecutive:
                result_name_prefix = QString ( "%1_%2" ).arg ( time_stamp ).arg ( "Consecutive" );
                break;
            case TrackingType::KeyFrameOnly:
                result_name_prefix = QString ( "%1_%2" ).arg ( time_stamp ).arg ( "KeyFrameOnly" );
                break;
            case TrackingType::FixedNumber:
                result_name_prefix = QString ( "%1_%2" ).arg ( time_stamp ).arg ( "FixedNumber" );
                break;
            case TrackingType::Unknown:
                emit Message ( "No result to be written." );
                return false;

        }

        QString data_folder_path = data_dir_.absolutePath ( );
        QDir    dir ( data_folder_path + "/Result" );
        if ( !dir.exists ( ) ) dir.mkdir ( data_folder_path + "/Result" );

        std::string   result_file_name = QString ( data_folder_path + "/Result/" + result_name_prefix + ".txt" ).toStdString ( );
        std::ofstream out ( result_file_name );

        if ( out ) {

            out << "Current tracker type : " << static_cast<int>(params_.type_) << std::endl;

            switch ( params_.type_ ) {

                case TrackingType::Consecutive: {
                    out << params_.paramsConsectutive.Output ( ).toStdString ( ) << std::endl;
                    break;
                }
                case TrackingType::KeyFrameOnly: {
                    out << params_.paramsKeyFramesOnly.Output ( ).toStdString ( ) << std::endl;
                    break;
                }
                case TrackingType::FixedNumber : {
                    out << params_.paramsFixedNumber.Output ( ).toStdString ( ) << std::endl;
                    break;
                }
                default:
                    break;
            }

            out << "point1 " << glm::vec4 ( point_pair.first , 1.0f );
            out << "point2 " << glm::vec4 ( point_pair.second , 1.0f );

            out.close ( );

            return true;
        }

        return false;
    }

    bool SlamAlgorithm::WriteResult ( ) {

        time_t time_stamp = time ( nullptr );

        QString result_name_prefix;

        std::cout << params_.type_ << std::endl;

        switch ( params_.type_ ) {
            case TrackingType::Consecutive:
                result_name_prefix = QString ( "%1_%2" ).arg ( time_stamp ).arg ( "Consecutive" );
                break;
            case TrackingType::KeyFrameOnly:
                result_name_prefix = QString ( "%1_%2" ).arg ( time_stamp ).arg ( "KeyFrameOnly" );
                break;
            case TrackingType::FixedNumber:
                result_name_prefix = QString ( "%1_%2" ).arg ( time_stamp ).arg ( "FixedNumber" );
                break;
            case TrackingType::Unknown:
                emit Message ( "No result to be written." );
                return false;

        }

        QString data_folder_path = data_dir_.absolutePath ( );
        QDir    dir ( data_folder_path + "/Result" );
        if ( !dir.exists ( ) ) dir.mkdir ( data_folder_path + "/Result" );

        std::string   result_file_name = QString ( data_folder_path + "/Result/" + result_name_prefix + ".txt" ).toStdString ( );
        std::ofstream out ( result_file_name );

        if ( out ) {

            out << "Current tracker type : " << static_cast<int>(params_.type_) << std::endl;

            switch ( params_.type_ ) {

                case TrackingType::Consecutive: {
                    out << params_.paramsConsectutive.Output ( ).toStdString ( ) << std::endl;
                    break;
                }
                case TrackingType::KeyFrameOnly: {
                    out << params_.paramsKeyFramesOnly.Output ( ).toStdString ( ) << std::endl;
                    break;
                }
                case TrackingType::FixedNumber : {
                    out << params_.paramsFixedNumber.Output ( ).toStdString ( ) << std::endl;
                    break;
                }
                default:
                    break;
            }

//          out << "id x(estimation) y(estimation) z(estimation) "
//                  "x'(estimation) y'(estimation) z'(estimation) "
//                  "x(marker) y(marker) z(marker) "
//                  "x'(marker) y'(marker) z'(marker) "
//                  "position_error(estimation) "
//                  "position_error(marker) "
//                  "d_estimation_d_marker "
//                  "angle_estimation "
//                  "angle_marker" << std::endl;

//          std::cout << "all_markers_points_pairs_.size() : " << all_markers_points_pairs_.size ( ) << std::endl;
//
//          // estimation pair
//          glm::mat4 accumulated_matrix1_e;
//          glm::mat4 accumulated_matrix2_e;
//
//          // marker apir
//          glm::mat4 accumulated_matrix1_m;
//          glm::mat4 accumulated_matrix2_m;
            out << "CAUTION: The look at point has been translated to the origin." << std::endl;
            out << "Position X (estimation),Position Y (estimation),Position Z (estimation),"
                    "Position X (marker),Position Y (marker),Position Z (marker),"
                    "Translation Error,"
                    "LookatPoint X (estimation),LookatPoint Y (estimation),LookatPoint Z (estimation),"
                    "LookatPoint X (marker),LookatPoint Y (marker),LookatPoint Z (marker),"
                    "Rotation Error" << std::endl;

            auto position_estimation     = glm::vec3 ( );
            auto position_marker         = glm::vec3 ( );
            auto lookat_point_estimation = glm::vec3 ( 0.0f , 0.0f , 1.0f );
            auto lookat_point_marker     = glm::vec3 ( 0.0f , 0.0f , 1.0f );

            auto accumulated_matrix_estimation = glm::mat4 ( );
            auto accumulated_matrix_marker     = glm::mat4 ( );

            for ( const auto & keyframe : keyframes_ ) {

                accumulated_matrix_estimation *= keyframe.GetAlignmentMatrix ( );
                accumulated_matrix_marker *= keyframe.GetAnswerAlignmentMatrix ( );

                const auto _position_estimation = accumulated_matrix_estimation * glm::vec4 ( position_estimation , 1.0f );
                const auto _position_marker     = accumulated_matrix_marker * glm::vec4 ( position_marker , 1.0f );

                const auto _lookat_point_estimation = accumulated_matrix_estimation * glm::vec4 ( lookat_point_estimation , 1.0f );
                const auto _lookat_point_marker     = accumulated_matrix_marker * glm::vec4 ( lookat_point_marker , 1.0f );

                const auto __lookat_point_estimation = _lookat_point_estimation - _position_estimation;
                const auto __lookat_point_marker     = _lookat_point_marker - _position_marker;

                out << _position_estimation.x << "," << _position_estimation.y << "," << _position_estimation.z << ",";
                out << _position_marker.x << "," << _position_marker.y << "," << _position_marker.z << ",";

                if ( keyframe.IsUsed ( ) ) out << glm::length ( _position_estimation - _position_marker ) << ",";
                else out << 0 << ",";

                out << __lookat_point_estimation.x << "," << __lookat_point_estimation.y << "," << __lookat_point_estimation.z << ",";
                out << __lookat_point_marker.x << "," << __lookat_point_marker.y << "," << __lookat_point_marker.z << ",";

                // FIXME glm issue
                // if ( keyframe.IsUsed ( ) ) out << glm::angle ( __lookat_point_estimation , __lookat_point_marker ) << std::endl;
                // else out << 0 << std::endl;

            }

//          for ( auto id = 0 ; id < all_markers_points_pairs_.size ( ) ; ++id ) {
//
//              auto & markers_points_pair = all_markers_points_pairs_[ id ];
//              auto & points1             = markers_points_pair.first;
//              auto & points2             = markers_points_pair.second;
//
//              std::cout << id << " : points1.size() : " << points1.size ( ) << std::endl;
//
//              accumulated_matrix1_e *= keyframes_[ id ].GetAlignmentMatrix ( );               // estimation
//              accumulated_matrix2_e *= keyframes_[ id + 1 ].GetAlignmentMatrix ( );           // estimation
//
//              accumulated_matrix1_m *= keyframes_[ id ].GetAnswerAlignmentMatrix ( );         // marker
//              accumulated_matrix2_m *= keyframes_[ id + 1 ].GetAnswerAlignmentMatrix ( );     // marker
//
//              for ( auto i = 0 ; i < points1.size ( ) ; ++i ) {
//
//                  const auto & point1 = points1[ i ];
//                  const auto & point2 = points2[ i ];
//
//                  const auto glm_point1 = glm::vec4 ( point1.x , point1.y , point1.z , 1.0f );
//                  const auto glm_point2 = glm::vec4 ( point2.x , point2.y , point2.z , 1.0f );
//
//                  // estimation pair
//                  const auto glm_point1_e_vec3 = glm::vec3 ( accumulated_matrix1_e * glm_point1 );
//                  const auto glm_point2_e_vec3 = glm::vec3 ( accumulated_matrix2_e * glm_point2 );
//
//                  // marker pair
//                  const auto glm_point1_m_vec3 = glm::vec3 ( accumulated_matrix1_m * glm_point1 );
//                  const auto glm_point2_m_vec3 = glm::vec3 ( accumulated_matrix2_m * glm_point2 );
//
//                  const auto angle_estimation = glm::angle ( glm_point1_e_vec3 , glm_point2_e_vec3 );
//                  const auto angle_marker     = glm::angle ( glm_point1_m_vec3 , glm_point2_m_vec3 );
//
//                  out << id << " ";
//
//                  // estimation pair
//                  out << glm_point1_e_vec3.x << " " << glm_point1_e_vec3.y << " " << glm_point1_e_vec3.z << " ";
//                  out << glm_point2_e_vec3.x << " " << glm_point2_e_vec3.y << " " << glm_point2_e_vec3.z << " ";
//
//                  // marker pair
//                  out << glm_point1_m_vec3.x << " " << glm_point1_m_vec3.y << " " << glm_point1_m_vec3.z << " ";
//                  out << glm_point2_m_vec3.x << " " << glm_point2_m_vec3.y << " " << glm_point2_m_vec3.z << " ";
//
//                  out << angle_estimation << " ";
//                  out << angle_marker << std::endl;
//              }
//          }

            out.close ( );
            return true;
        }

        return false;

    }

    bool SlamAlgorithm::CheckPreviousResult ( ) {

        std::cout << "Checking privious result." << std::endl;

        result_cache_path_ = data_dir_.absolutePath ( ) + "/Cache";

        QDir dir ( result_cache_path_ );
        if ( !dir.exists ( ) ) {
            dir.mkdir ( result_cache_path_ );
            return false;
        }

        QStringList filter_list;
        filter_list.push_back ( QString ( "*.cache" ) );

        return !dir.entryInfoList ( filter_list ).empty ( );
    }

    void SlamAlgorithm::UsePreviousResult ( const QString & result_cache_name ) {

        if ( keyframes_.empty ( ) ) {
            emit Message ( "No data found, cannot apply matrices." );
        }

//      result_keyframes_.clear ( );

        QTimer timer;
        timer.start ( );

        ComputationResultCache cache;

        bool load_succeeded = LoadComputationResultCache ( result_cache_name.toStdString ( ) , cache );
        if ( !load_succeeded ) { return; }

        auto data_set_name    = cache.data_set_name;
        auto computation_time = cache.computation_time;

        params_ = cache.options;

        for ( auto i = 0 ; i < cache.indices.size ( ) ; ++i ) {

            auto id                = cache.indices[ i ];
            auto is_used           = cache.used_status[ i ];
            auto estimation_matrix = cache.estimation_matrices[ i ];
            auto marker_matrix     = cache.marker_matrices[ i ];

            auto & kf = keyframes_[ id ];

            kf.SetId ( id );
            kf.SetUsed ( is_used );
            kf.SetAlignmentMatrix ( estimation_matrix );
            kf.SetAnswerAlignmentMatrix ( marker_matrix );

//          result_keyframes_.push_back ( kf );
        }

        emit SendData ( keyframes_ );
        // emit Message ( QString ( "Done loading %1 frames' results. (used %2)" )
        //                         .arg ( keyframes_.size ( ) )
        //                         .arg ( ConvertTime ( timer.elapsed ( ) ) ) );
    }

    void SlamAlgorithm::StopCompute ( ) {

        running_flag_ = false;
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    bool SaveMatricesInfo ( const std::string & file_name , const MatricesInfo & info ) {

        std::ofstream out ( file_name , std::ios::binary );

        if ( out ) {
            namespace bio = ::boost::iostreams;

            bio::filtering_ostream f;
            f.push ( bio::gzip_compressor ( ) );
            f.push ( out );

            boost::archive::binary_oarchive ar ( out );
            ar << info;

            return true;
        }

        return false;
    }

    bool LoadMatricesInfo ( const std::string & file_name , MatricesInfo & info ) {

        std::ifstream in ( file_name , std::ios::binary );

        if ( in ) {
            namespace bio = ::boost::iostreams;

            bio::filtering_istream f;
            f.push ( bio::gzip_decompressor ( ) );
            f.push ( in );

            boost::archive::binary_iarchive ar ( in );
            ar >> info;

            return true;
        }

        return false;
    }

}

