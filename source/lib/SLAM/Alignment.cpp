//
// Created by LinKun on 9/13/15.
//
#include "SLAM/Alignment.h"
#include "SLAM/Calibrator.h"
#include "SLAM/Transformation.h"
#include "SLAM/GlobalOptimization.h"
#include "SLAM/ArucoMarkerUtils.h"

#include <limits>

#include <Core/Serialize.h>
#include <Core/Utility.h>
#include <Core/MyMath.h>

#include <boost/tuple/tuple.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <QMessageBox>
#include <QMap>
#include <QFileDialog>
#include <QThread>
#include <QTime>
#include <QWaitCondition>

namespace NiS {

	SlamComputer::SlamComputer ( QObject * parent ) :
			running_flag_ ( true ) ,
			has_answer_ ( false ) ,
			is_computation_configured_ ( false ) ,
			is_data_initialized_ ( false ) {

	}

	void SlamComputer::SetDataDir ( const QDir & data_dir ) {

		data_dir_ = data_dir;
	}

	void SlamComputer::StartCompute ( ) {

		running_flag_ = true;

		std::cout << "SlamComputer thread : " << QThread::currentThreadId ( ) << std::endl;
		std::cout << "SlamComputer thread - data size : " << keyframes_.size ( ) << std::endl;

		if ( keyframes_.size ( ) < 2 ) {
			emit SendData ( keyframes_ );
			return;
		}

		QTime timer;
		timer.start ( );

		emit Message ( "Computation begins..." );

		switch ( options_.type_ ) {
			case TrackingType::OneByOne:
				ComputeHelper < TrackingType::OneByOne > ( );
				break;
			case TrackingType::FixedFrameCount:
				ComputeHelper < TrackingType::FixedFrameCount > ( );
				break;
			case TrackingType::PcaKeyFrame:
				ComputeHelper < TrackingType::PcaKeyFrame > ( );
				break;
			case TrackingType::Unknown:
				emit Message ( "Setup computation options at first." );
				return;
		}

		emit Message ( QString ( "Done computing %1 frames. (used %2)" )
				               .arg ( keyframes_.size ( ) )
				               .arg ( ConvertTime ( timer.elapsed ( ) ) ) );

		emit SendData ( keyframes_ );

		if ( has_answer_ ) WriteCache ( timer.elapsed ( ) , "WithAnswer" );
		else WriteCache ( timer.elapsed ( ) );

	}

	void SlamComputer::StartGenerateAnswer ( ) {

		if ( keyframes_.empty ( ) ) {
			return;
		}

		if ( keyframes_.size ( ) < 2 ) {
			emit SendData ( keyframes_ );
			return;
		}

		aruco::MarkerDetector marker_detector;

		QTime timer;
		timer.start ( );

		all_markers_points_pairs_.clear ( );

		for ( auto i = 1 ; i < keyframes_.size ( ) ; ++i ) {

			auto & keyframe1 = keyframes_[ i - 1 ];
			auto & keyframe2 = keyframes_[ i ];

			Markers markers1;
			Markers markers2;

			marker_detector.detect ( keyframe1.GetColorImage ( ) , markers1 );
			marker_detector.detect ( keyframe2.GetColorImage ( ) , markers2 );

			Points points1;
			Points points2;

			boost::tie ( points1 , points2 ) = ArucoMarkerUtils::CreatePoints ( markers1 , markers2 , keyframe1 , keyframe2 );

			all_markers_points_pairs_.push_back ( std::make_pair ( points1 , points2 ) );

			auto matrix = ComputeTransformationMatrix ( points2 , points1 );

			keyframe2.SetAnswerAlignmentMatrix ( std::move ( Convert_OpenCV_Matx44f_To_GLM_mat4 ( matrix ) ) );
		}

		emit Message ( QString ( "Done generating answers of  %1 frames. (used %2)" )
				               .arg ( keyframes_.size ( ) )
				               .arg ( ConvertTime ( timer.elapsed ( ) ) ) );

		emit SendData ( keyframes_ );

		has_answer_ = true;
	}

	bool SlamComputer::WriteResult ( const std::pair < glm::vec3 , glm::vec3 > & point_pair ) {

		time_t time_stamp = time ( nullptr );

		QString result_name_prefix;

		std::cout << options_.type_ << std::endl;

		switch ( options_.type_ ) {
			case TrackingType::OneByOne:
				result_name_prefix = QString ( "%1_%2" ).arg ( time_stamp ).arg ( "OneByOne" );
				break;
			case TrackingType::PcaKeyFrame:
				result_name_prefix = QString ( "%1_%2" ).arg ( time_stamp ).arg ( "PcaKeyFrame" );
				break;
			case TrackingType::FixedFrameCount:
				result_name_prefix = QString ( "%1_%2" ).arg ( time_stamp ).arg ( "FixedFrameCount" );
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

			out << "Current tracker type : " << static_cast<int>(options_.type_) << std::endl;

			switch ( options_.type_ ) {

				case TrackingType::OneByOne: {
					out << options_.options_one_by_one.Output ( ).toStdString ( ) << std::endl;
					break;
				}
				case TrackingType::PcaKeyFrame: {
					out << options_.options_pca_keyframe.Output ( ).toStdString ( ) << std::endl;
					break;
				}
				case TrackingType::FixedFrameCount : {
					out << options_.options_fixed_frame_count.Output ( ).toStdString ( ) << std::endl;
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

	bool SlamComputer::WriteResult ( ) {

		if ( all_markers_points_pairs_.empty ( ) ) {
			return false;
		}

		time_t time_stamp = time ( nullptr );

		QString result_name_prefix;

		std::cout << options_.type_ << std::endl;

		switch ( options_.type_ ) {
			case TrackingType::OneByOne:
				result_name_prefix = QString ( "%1_%2" ).arg ( time_stamp ).arg ( "OneByOne" );
				break;
			case TrackingType::PcaKeyFrame:
				result_name_prefix = QString ( "%1_%2" ).arg ( time_stamp ).arg ( "PcaKeyFrame" );
				break;
			case TrackingType::FixedFrameCount:
				result_name_prefix = QString ( "%1_%2" ).arg ( time_stamp ).arg ( "FixedFrameCount" );
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

			out << "Current tracker type : " << static_cast<int>(options_.type_) << std::endl;

			switch ( options_.type_ ) {

				case TrackingType::OneByOne: {
					out << options_.options_one_by_one.Output ( ).toStdString ( ) << std::endl;
					break;
				}
				case TrackingType::PcaKeyFrame: {
					out << options_.options_pca_keyframe.Output ( ).toStdString ( ) << std::endl;
					break;
				}
				case TrackingType::FixedFrameCount : {
					out << options_.options_fixed_frame_count.Output ( ).toStdString ( ) << std::endl;
					break;
				}
				default:
					break;
			}

			glm::mat4 accumulated_matrix1;
			glm::mat4 accumulated_matrix2;

			out <<
			"id x y z x'(estimation) y'(estimation) z'(estimation) x'(marker) y'(marker) z'(marker) position_error(estimation) position_error(marker)" <<
			std::endl;

			std::cout << "all_markers_points_pairs_.size() : " << all_markers_points_pairs_.size ( ) << std::endl;

			for ( auto id = 0 ; id < all_markers_points_pairs_.size ( ) ; ++id ) {

				auto & markers_points_pair = all_markers_points_pairs_[ id ];
				auto & points1             = markers_points_pair.first;
				auto & points2             = markers_points_pair.second;

				std::cout << id << " : points1.size() : " << points1.size ( ) << std::endl;
				for ( auto i = 0 ; i < points1.size ( ) ; ++i ) {

					const auto & point1 = points1[ i ];
					const auto & point2 = points2[ i ];

					out << id << " ";

					accumulated_matrix1 *= keyframes_[ id + 1 ].GetAlignmentMatrix ( );         // estimation
					accumulated_matrix2 *= keyframes_[ id + 1 ].GetAnswerAlignmentMatrix ( );   // marker

					const auto glm_point2_1 = glm::vec4 ( point2.x , point2.y , point2.z , 1.0f );
					const auto glm_point2_2 = glm::vec4 ( point2.x , point2.y , point2.z , 1.0f );

					const auto glm_point2_1_vec3 = glm::vec3 ( accumulated_matrix1 * glm_point2_1 );
					const auto glm_point2_2_vec3 = glm::vec3 ( accumulated_matrix2 * glm_point2_2 );

					out << point1.x << " " << point1.y << " " << point1.z << " ";
					out << glm_point2_1_vec3.x << " " << glm_point2_1_vec3.y << " " << glm_point2_1_vec3.z << " ";
					out << glm_point2_2_vec3.x << " " << glm_point2_2_vec3.y << " " << glm_point2_2_vec3.z << " ";

					out << std::sqrt ( ( point1.x - glm_point2_1_vec3.x ) * ( point1.x - glm_point2_1_vec3.x ) +
					                   ( point1.y - glm_point2_1_vec3.y ) * ( point1.y - glm_point2_1_vec3.y ) +
					                   ( point1.z - glm_point2_1_vec3.z ) * ( point1.z - glm_point2_1_vec3.z ) ) << " ";

					out << std::sqrt ( ( point1.x - glm_point2_2_vec3.x ) * ( point1.x - glm_point2_2_vec3.x ) +
					                   ( point1.y - glm_point2_2_vec3.y ) * ( point1.y - glm_point2_2_vec3.y ) +
					                   ( point1.z - glm_point2_2_vec3.z ) * ( point1.z - glm_point2_2_vec3.z ) ) << std::endl;
				}

			}

			out.close ( );
			return true;
		}
		return false;

	}


	bool SlamComputer::CheckPreviousResult ( ) {

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

	void SlamComputer::UsePreviousResult ( const QString & result_cache_name ) {

		if ( keyframes_.empty ( ) ) {
			emit Message ( "No data found, cannot apply matrices." );
		}

//		result_keyframes_.clear ( );

		QTime timer;
		timer.start ( );

		ComputationResultCache cache;

		bool load_succeeded = LoadComputationResultCache ( result_cache_name.toStdString ( ) , cache );
		if ( !load_succeeded ) { return; }

		auto data_set_name    = cache.data_set_name;
		auto computation_time = cache.computation_time;

		options_ = cache.options;

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

//			result_keyframes_.push_back ( kf );
		}

		emit SendData ( keyframes_ );
		emit Message ( QString ( "Done loading %1 frames' results. (used %2)" )
				               .arg ( keyframes_.size ( ) )
				               .arg ( ConvertTime ( timer.elapsed ( ) ) ) );
	}

	void SlamComputer::StopCompute ( ) {

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

