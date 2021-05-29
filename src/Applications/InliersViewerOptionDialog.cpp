//
// Created by LinKun on 10/13/15.
//

#include "ui_InliersViewerOptionDialog.h"
#include "Engine/InliersViewerOptionDialog.h"
#include <opencv2/core/core_c.h>
#include <SLAM/Transformation.h>
#include <SLAM/Tracker.h>
#include <QFileDialog>

namespace MapCreator {

	InliersViewerOptionDialog::InliersViewerOptionDialog ( QWidget * parent ) :
			ui_(new Ui::InliersViewerOptionDialog),
			has_frame1_ ( false ) ,
			has_frame2_ ( false ) {

		ui_->setupUi ( this );
		setWindowFlags ( Qt::WindowStaysOnTopHint );
		setFixedSize ( sizeHint ( ) );

		// connect ( ui_->PushButton_OpenDataFileFolder , & QPushButton::clicked , this , & InliersViewerOptionDialog::OpenDataFileFolder );

		keyframes_for_inliers_.resize ( 2 );

	}

	InliersViewerOptionDialog::~InliersViewerOptionDialog ( ) { }

	void InliersViewerOptionDialog::OpenDataFileFolder ( ) {

		QString data_folder_path = QFileDialog::getExistingDirectory ( this , "Open.." , "." );

		QDir dir ( data_folder_path , "*.dat" );

		QFileInfoList file_list = dir.entryInfoList ( );
	}

	void InliersViewerOptionDialog::SetKeyFrames ( const KeyFrames & keyframes ) {

		std::cout << "inliers dialog received : " << keyframes.size ( ) << " frames." << std::endl;

		if ( not keyframes.empty ( ) ) {

			keyframes_ = keyframes;

			// since every time you change the row index of the list widget, the signal will be sent.
			// At this time, if you clear the list widget, will cause the current row to be null finally,
			// and this will cause your program to crash.
			// The solution is that to disconnect the connection at first, and after you set up the new data,
			// re-connect the connection again. This should work fine.

//			disconnect ( ui_->SpinBox_IndexFrame1 , SIGNAL( valueChanged ( int ) ) , ui_->HorizontalSlider_IndexFrame1 , SLOT( setValue ( int ) ) );
//			disconnect ( ui_->SpinBox_IndexFrame2 , SIGNAL( valueChanged ( int ) ) , ui_->HorizontalSlider_IndexFrame2 , SLOT( setValue ( int ) ) );
//			disconnect ( ui_->HorizontalSlider_IndexFrame1 , SIGNAL( valueChanged ( int ) ) , ui_->SpinBox_IndexFrame1 , SLOT( setValue ( int ) ) );
//			disconnect ( ui_->HorizontalSlider_IndexFrame1 , SIGNAL( valueChanged ( int ) ) , ui_->SpinBox_IndexFrame2 , SLOT( setValue ( int ) ) );
//
//			disconnect ( ui_->HorizontalSlider_IndexFrame1 , SIGNAL( valueChanged ( int ) ) , this , SLOT( onSetNewFrame1 ( int ) ) );
//			disconnect ( ui_->HorizontalSlider_IndexFrame2 , SIGNAL( valueChanged ( int ) ) , this , SLOT( onSetNewFrame2 ( int ) ) );
//

			ui_->LineEdit_TotalFrameCount->setText ( QString::number ( keyframes.size ( ) ) );

			ui_->HorizontalSlider_IndexFrame1->setRange ( 0 , static_cast<int> (keyframes_.size ( ) ) - 1 );
			ui_->HorizontalSlider_IndexFrame2->setRange ( 0 , static_cast<int> (keyframes_.size ( ) ) - 1 );
			ui_->SpinBox_IndexFrame1->setRange ( 0 , static_cast<int> (keyframes_.size ( ) ) - 1 );
			ui_->SpinBox_IndexFrame2->setRange ( 0 , static_cast<int> (keyframes_.size ( ) ) - 1 );


			connect ( ui_->SpinBox_IndexFrame1 , SIGNAL( valueChanged ( int ) ) , ui_->HorizontalSlider_IndexFrame1 , SLOT( setValue ( int ) ) );
			connect ( ui_->SpinBox_IndexFrame2 , SIGNAL( valueChanged ( int ) ) , ui_->HorizontalSlider_IndexFrame2 , SLOT( setValue ( int ) ) );
			connect ( ui_->HorizontalSlider_IndexFrame1 , SIGNAL( valueChanged ( int ) ) , ui_->SpinBox_IndexFrame1 , SLOT( setValue ( int ) ) );
			connect ( ui_->HorizontalSlider_IndexFrame2 , SIGNAL( valueChanged ( int ) ) , ui_->SpinBox_IndexFrame2 , SLOT( setValue ( int ) ) );

			connect ( ui_->PushButton_Show , SIGNAL( clicked ( ) ) , this , SLOT( onShowButtonClicked ( ) ) );

//			connect ( ui_->HorizontalSlider_IndexFrame1 , SIGNAL( valueChanged ( int ) ) , this , SLOT( onSetNewFrame1 ( int ) ) );
//			connect ( ui_->HorizontalSlider_IndexFrame2 , SIGNAL( valueChanged ( int ) ) , this , SLOT( onSetNewFrame2 ( int ) ) );

			ui_->HorizontalSlider_IndexFrame1->setValue ( 0 );
			ui_->HorizontalSlider_IndexFrame2->setValue ( 0 );

//			emit SendData ( keyframes_for_inliers_ );
		}

	}

	void InliersViewerOptionDialog::onSetNewFrame1 ( int index ) {

		if ( 0 <= index and index < keyframes_.size ( ) ) {
			SetFrame1 ( keyframes_[ index ] );
			if ( ui_->SpinBox_IndexFrame1->value ( ) != ui_->SpinBox_IndexFrame2->value ( ) )
				ComputeCorrespondingPoints ( );
		}

	}

	void InliersViewerOptionDialog::onSetNewFrame2 ( int index ) {

		if ( 0 <= index and index < keyframes_.size ( ) ) {
			SetFrame2 ( keyframes_[ index ] );
			if ( ui_->SpinBox_IndexFrame1->value ( ) != ui_->SpinBox_IndexFrame2->value ( ) )
				ComputeCorrespondingPoints ( );
		}
	}

	void InliersViewerOptionDialog::ComputeCorrespondingPoints ( ) {

//		assert( !keyframes_for_inliers_.empty ( ) );

		if ( keyframes_for_inliers_[ 0 ].GetColorImage ( ).empty ( ) or keyframes_for_inliers_[ 1 ].GetColorImage ( ).empty ( ) )
			return;

		const auto & frame1 = keyframes_for_inliers_[ 0 ];
		const auto & frame2 = keyframes_for_inliers_[ 1 ];

		QString _message;

		CorrespondingPointsPair corresponding_points_pair = CreateCorrespondingPointsPair ( frame1 ,
		                                                                                    frame2 );


		const auto & points1 = corresponding_points_pair.first;

		cv::Mat points_mat ( ( int ) points1.size ( ) , 3 , CV_32FC1 , ( void * ) points1.data ( ) );
		cv::PCA pca1 ( points_mat , cv::Mat ( ) , CV_PCA_DATA_AS_ROW );

		float _1st_principal_component_variance1     = pca1.eigenvalues.at < float > ( 0 , 0 );
		float _2nd_principal_component_variance1     = pca1.eigenvalues.at < float > ( 1 , 0 );
		float _3rd_principal_component_variance1     = pca1.eigenvalues.at < float > ( 2 , 0 );
		float _1st_principal_component_contribution1 = _1st_principal_component_variance1 /
		                                               ( _1st_principal_component_variance1 +
		                                                 _2nd_principal_component_variance1 +
		                                                 _3rd_principal_component_variance1 );


		_message = QString ( "3D corresponding points PCA results : \n"
				                     "1st PCA eigenvalue   : %1\n"
				                     "2nd PCA eigenvalue   : %2\n"
				                     "3rd PCA eigenvalue   : %3\n"
				                     "1st PCA contribution : %4\n" )
				.arg ( _1st_principal_component_variance1 )
				.arg ( _2nd_principal_component_variance1 )
				.arg ( _3rd_principal_component_variance1 )
				.arg ( _1st_principal_component_contribution1 );

		emit Message ( _message );

		std::cout << "Threshold : " <<
		params_.paramsConsectutive.num_ransac_iteration << ", " <<
		params_.paramsConsectutive.threshold_outlier << ", " <<
		params_.paramsConsectutive.threshold_inlier << std::endl;

//		CorrespondingPointsPair inliers_pair = ComputeInliersWithFlow ( corresponding_points_pair.second ,
//		                                                                corresponding_points_pair.first ,
//		                                                                params_.paramsConsectutive.num_ransac_iteration ,
//		                                                                params_.paramsConsectutive.threshold_outlier ,
//		                                                                params_.paramsConsectutive.threshold_inlier );

		CorrespondingPointsPair inliers_pair = ComputeInliers ( corresponding_points_pair.second ,
		                                                        corresponding_points_pair.first ,
		                                                        params_.paramsConsectutive.num_ransac_iteration ,
		                                                        params_.paramsConsectutive.threshold_outlier ,
		                                                        params_.paramsConsectutive.threshold_inlier );

		const auto & inliers1 = inliers_pair.first;

		cv::Mat inliers_mat ( ( int ) inliers1.size ( ) , 3 , CV_32FC1 , ( void * ) inliers1.data ( ) );
		cv::PCA pca2 ( inliers_mat , cv::Mat ( ) , CV_PCA_DATA_AS_ROW );

		float _1st_principal_component_variance2     = pca2.eigenvalues.at < float > ( 0 , 0 );
		float _2nd_principal_component_variance2     = pca2.eigenvalues.at < float > ( 1 , 0 );
		float _3rd_principal_component_variance2     = pca2.eigenvalues.at < float > ( 2 , 0 );
		float _1st_principal_component_contribution2 = _1st_principal_component_variance2 /
		                                               ( _1st_principal_component_variance2 +
		                                                 _2nd_principal_component_variance2 +
		                                                 _3rd_principal_component_variance2 );


		_message = QString ( "3D corresponding inliers PCA results : \n"
				                     "1st PCA eigenvalue   : %1\n"
				                     "2nd PCA eigenvalue   : %2\n"
				                     "3rd PCA eigenvalue   : %3\n"
				                     "1st PCA contribution : %4\n" )
				.arg ( _1st_principal_component_variance2 )
				.arg ( _2nd_principal_component_variance2 )
				.arg ( _3rd_principal_component_variance2 )
				.arg ( _1st_principal_component_contribution2 );

		emit Message ( _message );


		ui_->LineEdit_NumberOfInitialCorrespondingPoints->setText ( QString::number ( static_cast<int>(corresponding_points_pair.first.size ( )) ) );
		ui_->LineEdit_NumberOfInliers->setText ( QString::number ( static_cast<int>(inliers_pair.first.size ( )) ) );

		assert( keyframes_for_inliers_.size ( ) == 2 );

		emit SendData ( keyframes_for_inliers_ );
		emit SendCorrespondingPoints ( corresponding_points_pair );
		emit SendInliers ( inliers_pair );
	}


	void InliersViewerOptionDialog::onShowButtonClicked ( ) {

		onSetNewFrame1 ( ui_->SpinBox_IndexFrame1->value ( ) );
		onSetNewFrame2 ( ui_->SpinBox_IndexFrame2->value ( ) );

	}

};