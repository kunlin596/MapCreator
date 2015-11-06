//
// Created by LinKun on 10/13/15.
//

#include"MapCreator/InliersViewerOptionDialog.h"
#include <SLAM/Transformation.h>
#include <SLAM/Tracker.h>
#include <QFileDialog>

namespace NiS {

	InliersViewerOptionDialog::InliersViewerOptionDialog ( QWidget * parent ) :
			has_frame1_ ( false ) ,
			has_frame2_ ( false ) {

		ui_.setupUi ( this );
		setWindowFlags ( Qt::WindowTitleHint | Qt::CustomizeWindowHint );

		// connect ( ui_.PushButton_OpenDataFileFolder , & QPushButton::clicked , this , & InliersViewerOptionDialog::OpenDataFileFolder );

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

		if ( keyframes.size ( ) > 1 ) {

			keyframes_ = keyframes;

			// since every time you change the row index of the list widget, the signal will be sent.
			// At this time, if you clear the list widget, will cause the current row to be null finally,
			// and this will cause your program to crash.
			// The solution is that to disconnect the connection at first, and after you set up the new data,
			// re-connect the connection again. This should work fine.
			disconnect ( ui_.HorizontalSlider_IndexFrame1 , SIGNAL( valueChanged ( int ) ) , this , SLOT( onSetNewFrame1 ( int ) ) );
			disconnect ( ui_.HorizontalSlider_IndexFrame2 , SIGNAL( valueChanged ( int ) ) , this , SLOT( onSetNewFrame2 ( int ) ) );

			ui_.LineEdit_TotalFrameCount->setText ( QString::number ( keyframes.size ( ) ) );

			ui_.HorizontalSlider_IndexFrame1->setMaximum ( static_cast<int> (keyframes_.size ( ) ) - 1 );
			ui_.HorizontalSlider_IndexFrame2->setMaximum ( static_cast<int> (keyframes_.size ( ) ) - 1 );

			ui_.LineEdit_CurrentIndexFrame1->setText ( QString::number ( ui_.HorizontalSlider_IndexFrame1->value ( ) ) );
			ui_.LineEdit_CurrentIndexFrame2->setText ( QString::number ( ui_.HorizontalSlider_IndexFrame2->value ( ) ) );

			connect ( ui_.HorizontalSlider_IndexFrame1 , SIGNAL( valueChanged ( int ) ) , this , SLOT( onSetNewFrame1 ( int ) ) );
			connect ( ui_.HorizontalSlider_IndexFrame2 , SIGNAL( valueChanged ( int ) ) , this , SLOT( onSetNewFrame2 ( int ) ) );

			SetFrame1 ( keyframes_[ 0 ] );
			SetFrame2 ( keyframes_[ 0 ] );

			emit SendData ( keyframes_for_inliers_ );
		}

	}

	void InliersViewerOptionDialog::onSetNewFrame1 ( int index ) {

		SetFrame1 ( keyframes_[ index ] );
		has_frame1_ = true;
		ui_.LineEdit_CurrentIndexFrame1->setText ( QString::number ( index ) );
		if ( has_frame2_ )ComputeCorrespondingPoints ( );

	}

	void InliersViewerOptionDialog::onSetNewFrame2 ( int index ) {

		SetFrame2 ( keyframes_[ index ] );
		ui_.LineEdit_CurrentIndexFrame2->setText ( QString::number ( index ) );
		has_frame2_ = true;
		if ( has_frame1_ )ComputeCorrespondingPoints ( );
	}

	void InliersViewerOptionDialog::ComputeCorrespondingPoints ( ) {

		assert( !keyframes_for_inliers_.empty ( ) );

		const KeyFrame & frame1 = keyframes_for_inliers_[ 0 ];
		const KeyFrame & frame2 = keyframes_for_inliers_[ 1 ];

		CorrespondingPointsPair corresponding_points_pair = CreateCorrespondingPointsPair ( frame1 ,
		                                                                                    frame2 );

		std::cout << "Threshold : " <<
		options_.options_one_by_one.num_ransac_iteration << ", " <<
		options_.options_one_by_one.threshold_outlier << ", " <<
		options_.options_one_by_one.threshold_inlier << std::endl;

		CorrespondingPointsPair inliers_pair = ComputeInliers ( corresponding_points_pair.second ,
		                                                        corresponding_points_pair.first ,
		                                                        options_.options_one_by_one.num_ransac_iteration ,
		                                                        options_.options_one_by_one.threshold_outlier ,
		                                                        options_.options_one_by_one.threshold_inlier );

		ui_.LineEdit_NumberOfInitialCorrespondingPoints->setText ( QString::number ( static_cast<int>(corresponding_points_pair.first.size ( )) ) );
		ui_.LineEdit_NumberOfInliers->setText ( QString::number ( static_cast<int>(inliers_pair.first.size ( )) ) );

		assert( keyframes_for_inliers_.size ( ) == 2 );

		emit SendData ( keyframes_for_inliers_ );
		emit SendCorrespondingPoints ( corresponding_points_pair );
		emit SendInliers ( inliers_pair );
	}


};