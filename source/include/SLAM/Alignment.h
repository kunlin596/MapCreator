//
// Created by LinKun on 9/13/15.
//

#ifndef NIS_MAPCREATOR_H
#define NIS_MAPCREATOR_H

#include <Core/Feature.h>

#include "SLAM/Option.h"
#include "SLAM/KeyFrame.h"
#include "SLAM/Matcher.h"
#include "SLAM/Tracker.h"
#include "SLAM/ComputationResultCache.h"
#include "SLAM/CoordinateConverter.h"

#include <limits>

namespace {

	using MatricesInfo = std::pair < std::vector < size_t > , std::vector < glm::mat4 > >;

};

namespace NiS {


	class SlamComputer : public QObject
	{

	Q_OBJECT

	public:

		SlamComputer ( QObject * parent = 0 );

		void SetDataDir ( const QDir & data_dir );
		void SetFeatureType ( Feature::Type type );
		bool WriteResult ( const std::pair < glm::vec3 , glm::vec3 > & point_pair );
		bool CheckPreviousResult ( );
		void UsePreviousResult ( const QString & result_cache_name );
		Options GetOptions ( ) const { return options_; }
		void SetRunningFLag ( bool running_flag ) { running_flag_ = running_flag; };

	public slots:

		void SetOptions ( const Options & options ) { options_ = options; };
		void SetFrameData ( const KeyFrames & keyframes ) {

			keyframes_           = keyframes;
			is_data_initialized_ = true;
		}
		void SetCoordinateConverter ( const XtionCoordinateConverter & converter ) {

			xtion_converter_  = converter;
			converter_choice_ = 0;
		}
		void SetCoordinateConverter ( const AistCoordinateConverter & converter ) {

			aist_converter_   = converter;
			converter_choice_ = 1;
		}

		void StartCompute ( );
		void StopCompute ( );

		void StartGenerateAnswer();

	public:

		inline const KeyFrames & GetResultKeyFrames ( ) const { return result_keyframes_; }
		inline const KeyFrames & GetKeyFrames ( ) const { return keyframes_; }

		inline bool IsComputationConfigured ( ) const { return is_computation_configured_; }
		inline bool IsDataInitialized ( ) const { return is_data_initialized_; }

	signals:

		void SendData ( KeyFrames );
		void Message ( QString );

	private:

		template < TrackingType type > void ComputeHelper ( ) {

			std::cout << "Computation begins" << std::endl;

			switch ( converter_choice_ ) {
				case 0: {
					Tracker < type > tracker1 ( keyframes_ , options_ , xtion_converter_ );
					do {
						result_keyframes_.push_back ( tracker1.ComputeNext ( ) );
						emit Message ( tracker1.GetMessage ( ) );
					}
					while ( tracker1.Update ( ) );
					break;
				}
				case 1: {
					Tracker < type > tracker2 ( keyframes_ , options_ , aist_converter_ );
					do {
						result_keyframes_.push_back ( tracker2.ComputeNext ( ) );
						emit Message ( tracker2.GetMessage ( ) );
					}
					while ( tracker2.Update ( ) );
					break;
				}
				default:
					break;
			}
		}
		template < TrackingType type > void WriteCache ( ) { }

		bool is_computation_configured_;
		bool is_data_initialized_;

		QDir data_dir_;

		QString result_cache_path_;

		Feature::Type feature_type_;

		Options options_;

		KeyFrames keyframes_;
		KeyFrames result_keyframes_;

		int                      converter_choice_;
		XtionCoordinateConverter xtion_converter_;
		AistCoordinateConverter  aist_converter_;

		bool running_flag_;

	};

	template < > void SlamComputer::WriteCache < TrackingType::OneByOne > ( );
	template < > void SlamComputer::WriteCache < TrackingType::FixedFrameCount > ( );
	template < > void SlamComputer::WriteCache < TrackingType::PcaKeyFrame > ( );

	// Serialize
	bool LoadMatricesInfo ( const std::string & file_name , MatricesInfo & info );
	bool SaveMatricesInfo ( const std::string & file_name , const MatricesInfo & info );

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

#endif //NIS_MAPCREATOR_H
