//
// Created by LinKun on 10/5/15.
//

#include "MapCreator/MainWindow.h"
#include "MapCreator/CoordinateConverterDialog.h"

#include <SLAM/Tracker.h>
#include <SLAM/CoordinateConverter.h>

#include <QMessageBox>
#include <QMouseEvent>
#include <QFileDialog>
#include <QDir>
#include <QThread>
#include <QProgressDialog>

namespace NiS {

	MainWindow::MainWindow ( QWidget * parent ) :
			computation_configured_ ( false ) ,
			computation_done_ ( false ) ,
			play_timer_ ( new QTimer ( this ) ) ,
			watcher_ ( new QFutureWatcher < void > ( this ) ) ,
			log_panel_dialog_ ( new LogPanelDialog ( this ) ) ,
			control_panel_dialog_ ( new ControlPanelDialog ( this ) ) ,
			cloud_play_control_dialog_ ( new CloudPlayControlDialog ( this ) ) {

		// Register object for Qt
		qRegisterMetaType < KeyFrames > ( "KeyFrames" );
		qRegisterMetaType < CorrespondingPointsPair > ( "CorrespondingPointsPair" );
		qRegisterMetaType < CorrespondingPointsPair > ( "PointPair" );

		ui_.setupUi ( this );

		keyframes_ptr_ = std::make_shared < KeyFrames > ( );

		inliers_viewer_option_dialog_ = new InliersViewerOptionDialog ( this );

		ui_.actionOpenInliersViewMode->setChecked ( false );
		ui_.actionStartSlamComputation->setEnabled ( false );
//		ui_.actionInternalCalibration->setEnabled ( false );
		ui_.actionOpenInliersViewMode->setEnabled ( false );
		ui_.actionCaptureModelMage->setEnabled ( false );
		ui_.actionUsePreviousResult->setEnabled ( false );
		ui_.actionConfigureSlamComputation->setEnabled ( false );

		ConnectCloudPlayControlDialog ( );
		ConnectControlPanelDialog ( );

		control_panel_dialog_->show ( );
		cloud_play_control_dialog_->show ( );

		connect ( ui_.actionConfigureSlamComputation , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionConfigureSlamComputation ( ) ) );
		connect ( ui_.actionStartSlamComputation , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionStartSlamComputation ( ) ) );
		connect ( ui_.actionOpenDataFiles , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionOpenDataFiles ( ) ) );
		connect ( ui_.actionShowControlPanel , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionShowControlPanel ( ) ) );
		connect ( ui_.actionOpenLogPanel , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionOpenLogPanel ( ) ) );
		connect ( ui_.actionInternalCalibration , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionInternalCalibration ( ) ) );
		connect ( ui_.actionOpenInliersViewMode , SIGNAL ( triggered ( bool ) ) , this , SLOT ( onActionOpenInliersViewerMode ( bool ) ) );
		connect ( ui_.actionCaptureModelMage , SIGNAL ( triggered ( bool ) ) , this , SLOT ( onActionCaptureModelImage ( ) ) );
		connect ( ui_.actionUsePreviousResult , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionUsePreviousResult ( ) ) );
		connect ( ui_.actionOutputResult , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionOutputResult ( ) ) );

		connect ( ui_.actionGenerateAnswer , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionGenerateAnswer ( ) ) );

		connect ( this , SIGNAL ( ConfigurationDone ( ) ) , this , SLOT ( PrepareComputation ( ) ) );
		connect ( this , SIGNAL ( ChangeViewerMode ( int ) ) , ui_.BasicViewer , SLOT ( SetViewerMode ( int ) ) );

		// UI dialogs initialization
		computation_configure_dialog_ = new ComputationConfigureDialog ( this );
		computation_configure_dialog_->setFixedSize ( computation_configure_dialog_->sizeHint ( ) );

		computer_ = new SlamComputer ( keyframes_ptr_ , this );
		connect ( computer_ , SIGNAL ( Message ( QString ) ) , log_panel_dialog_ , SLOT ( AppendMessage ( QString ) ) );
//		connect ( computer_ , SIGNAL ( SendData ( std::shared_ptr < KeyFrames > keyframes ) ) , ui_.BasicViewer , SLOT ( SetKeyFrames ( std::shared_ptr < KeyFrames >keyframes ) ) );

		connect ( ui_.BasicViewer , SIGNAL ( Message ( QString ) ) , log_panel_dialog_ , SLOT ( AppendMessage ( QString ) ) );
		connect ( ui_.actionStopSlamComputation , SIGNAL ( triggered ( ) ) , computer_ , SLOT ( StopCompute ( ) ) );

		// Marker Image Viewer
		marker_viewer_dialog_ = new MarkerViewerDialog ( this );
		connect ( marker_viewer_dialog_ , SIGNAL ( SendEstimationPointPair ( PointPair ) ) , ui_.BasicViewer ,
		          SLOT ( SetEstimationPointPair ( PointPair ) ) );
		connect ( marker_viewer_dialog_ , SIGNAL ( SendMarkerPointPair ( PointPair ) ) , ui_.BasicViewer ,
		          SLOT ( SetMarkerPointPair ( PointPair ) ) );

		connect ( ui_.actionIShowMarkerViewer , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionShowMarkerViewer ( ) ) );

	}

	void MainWindow::ConnectCloudPlayControlDialog ( ) {

		auto dialog_singal_ptr1 = static_cast<void ( CloudPlayControlDialog::* ) ( int )> (& CloudPlayControlDialog::SetBeginFrame);
		auto dialog_signal_ptr2 = static_cast<void ( CloudPlayControlDialog::* ) ( int )> (& CloudPlayControlDialog::SetEndFrame);

		auto viewer_slot_ptr1 = static_cast<void ( BasicViewer::* ) ( int )> ( & BasicViewer::SetBeginFrame );
		auto viewer_slot_ptr2 = static_cast<void ( BasicViewer::* ) ( int )> ( & BasicViewer::SetEndFrame );

		connect ( cloud_play_control_dialog_ , dialog_singal_ptr1 , ui_.BasicViewer , viewer_slot_ptr1 );
		connect ( cloud_play_control_dialog_ , dialog_signal_ptr2 , ui_.BasicViewer , viewer_slot_ptr2 );
	}

	void MainWindow::ConnectControlPanelDialog ( ) {

		connect ( control_panel_dialog_ , SIGNAL ( ResetCamera ( ) ) , ui_.BasicViewer , SLOT ( onResetCamera ( ) ) );
		connect ( control_panel_dialog_ , SIGNAL ( ResetModel ( ) ) , ui_.BasicViewer , SLOT ( onResetModel ( ) ) );
		connect ( control_panel_dialog_ , SIGNAL ( ChangePointCloudDensity ( int ) ) , ui_.BasicViewer , SLOT ( onChangeDensity ( int ) ) );
		connect ( control_panel_dialog_ , SIGNAL ( SetTopView ( int ) ) , ui_.BasicViewer , SLOT ( onTopView ( int ) ) );
		connect ( control_panel_dialog_ , SIGNAL ( SetSpinModel ( int ) ) , ui_.BasicViewer , SLOT ( onSpin ( int ) ) );
		connect ( control_panel_dialog_ , SIGNAL ( SetShowGrid ( int ) ) , ui_.BasicViewer , SLOT ( onRenderGrid ( int ) ) );
		connect ( control_panel_dialog_ , SIGNAL ( SetShowPointCloud ( int ) ) , ui_.BasicViewer , SLOT ( onRenderPointCloud ( int ) ) );
		connect ( control_panel_dialog_ , SIGNAL ( SetShowTrajectory ( int ) ) , ui_.BasicViewer , SLOT ( onRenderTrajectory ( int ) ) );
		connect ( control_panel_dialog_ , SIGNAL ( SetShowAnswer ( int ) ) , ui_.BasicViewer , SLOT ( onRenderAnswer ( int ) ) );
	}

	MainWindow::~MainWindow ( ) {

		delete watcher_;
		delete inliers_viewer_option_dialog_;
	}

	void MainWindow::PrepareComputation ( ) {

		computation_configured_ = true;

		ui_.actionStartSlamComputation->setEnabled ( true );

	}

	void MainWindow::EnableDensitySlider ( ) {

//		ui_.HorizontalSlider_PointCloudDensity->setEnabled ( true );
	}

	void MainWindow::OnReadingFinished ( ) {

//		ui_.actionInternalCalibration->setEnabled ( true );
	}

	void MainWindow::OnConversionFinished ( ) {

		watcher_->disconnect ( this );

		assert( not keyframes_ptr_->empty ( ) );

		std::cout << "OnConversionFinished - data size : " << keyframes_ptr_->size ( ) << std::endl;

		ui_.actionStartSlamComputation->setEnabled ( true );
		ui_.actionOpenInliersViewMode->setEnabled ( true );
		ui_.actionCaptureModelMage->setEnabled ( true );
		ui_.actionUsePreviousResult->setEnabled ( true );
		ui_.actionConfigureSlamComputation->setEnabled ( true );

		computer_->SetDataDir ( data_dir_ );
//        computer_->SetFrameData(keyframes_);
	}

	void MainWindow::WriteResult ( const std::pair < glm::vec3 , glm::vec3 > & marker_points_pair ) {

		computer_->WriteResult ( marker_points_pair );

	}

	void MainWindow::onActionOpenDataFiles ( ) {

		// ResetWatcher ( );

		watcher_->disconnect ( this );

		ui_.actionStartSlamComputation->setEnabled ( false );

		std::cerr << "Main thread : " << QThread::currentThreadId ( ) << std::endl;

		QString dir_path = QFileDialog::getExistingDirectory ( this , "Open Data File Dir ..." , QDir::homePath ( ) );

		if ( not dir_path.isEmpty ( ) ) {

			data_dir_          = QDir ( dir_path );

			QProgressDialog dialog;
			dialog.setModal ( true );
			dialog.setAutoClose ( true );
			dialog.setLabelText (
					QString ( "Reading data files ... (using %1 thread(s).)" ).arg ( QThread::idealThreadCount ( ) ) );
			dialog.setFixedSize ( dialog.sizeHint ( ) );

			QStringList   name_filers ( QString ( "*.dat" ) );
			QFileInfoList list = data_dir_.entryInfoList ( name_filers );

			handler_ = new ImageHandler2 ( list , keyframes_ptr_ , this );

			connect ( watcher_ , SIGNAL ( finished ( ) ) , this , SLOT ( OnReadingFinished ( ) ) );
			connect ( handler_ , SIGNAL ( SetProgressValue ( int ) ) , & dialog , SLOT ( setValue ( int ) ) );
			connect ( handler_ , SIGNAL ( SetProgressRange ( int , int ) ) , & dialog , SLOT ( setRange ( int , int ) ) );
			connect ( & dialog , SIGNAL ( canceled ( ) ) , watcher_ , SLOT ( cancel ( ) ) );

			watcher_->setFuture ( QtConcurrent::run ( this->handler_ , & ImageHandler2::StartReading ) );
			dialog.exec ( );
		}
	}

	void MainWindow::onActionOpenInternalCalibrationFile ( ) {

		QString path = QFileDialog::getOpenFileName ( this ,
		                                              "Open Internal Calibration File" ,
		                                              QDir::homePath ( ) ,
		                                              "*.cal" );


		if ( !path.isEmpty ( ) ) {

//			handler_->SetInternalCalibrator ( std::move ( Calibrator ( path.toStdString ( ) ) ) );

			handler_->SetAistCoordinateConverter ( std::move ( AistCoordinateConverter ( path.toStdString ( ) ) ) );

			ui_.actionInternalCalibration->setEnabled ( true );

			log_panel_dialog_->AppendMessage ( QString ( "Internal calibration file loaded :\n%1" ).arg ( path ) );
		}

	}

	void MainWindow::onActionInternalCalibration ( ) {

		watcher_->disconnect ( this );

		ui_.actionStartSlamComputation->setEnabled ( false );

		std::cerr << "Main thread : " << QThread::currentThreadId ( ) << std::endl;

		CoordinateConverterDialog dialog;
		dialog.setModal ( true );

		if ( dialog.exec ( ) != QDialog::Accepted ) {
			return;
		}

		auto choice = dialog.GetChoice ( );

		std::cout << "Choice is : " << choice << std::endl;

		handler_->SetKeyFrames ( keyframes_ptr_ );

		switch ( choice ) {
			case 0: {
				// This is for bundle adjustment, since BA needs to know the how the coordinate is converted from
				// screen to world in order to convert it back wards
				computer_->SetCoordinateConverter ( handler_->GetXtionCoordinateConverter ( ) );
				break;
			}
			case 1: {
				onActionOpenInternalCalibrationFile ( );
				// Likewise.
				computer_->SetCoordinateConverter ( handler_->GetAistCoordinateConverter ( ) );
				break;
			}
		}

		QProgressDialog dialog_;
		dialog_.setModal ( true );
		dialog_.setAutoClose ( true );
		dialog_.setLabelText (
				QString ( "Converting depth image to point image\nDetecting keypoints in color image.\n ... (using %1 thread(s).)" ).arg (
						QThread::idealThreadCount ( ) ) );
		dialog_.setFixedSize ( dialog_.sizeHint ( ) );

		connect ( watcher_ , SIGNAL ( finished ( ) ) , this , SLOT ( OnConversionFinished ( ) ) );
		connect ( handler_ , SIGNAL ( SetProgressValue ( int ) ) , & dialog_ , SLOT ( setValue ( int ) ) );
		connect ( handler_ , SIGNAL ( SetProgressRange ( int , int ) ) , & dialog_ , SLOT ( setRange ( int , int ) ) );
		connect ( & dialog_ , SIGNAL ( canceled ( ) ) , watcher_ , SLOT ( cancel ( ) ) );
		watcher_->setFuture ( QtConcurrent::run ( this->handler_ , & ImageHandler2::ConvertToPointImages , choice ) );
		dialog_.exec ( );
	}

	void MainWindow::onActionShowControlPanel ( ) {

	}

	void MainWindow::onActionOpenLogPanel ( ) {

		if ( log_panel_dialog_->isHidden ( ) ) {
			log_panel_dialog_->show ( );
		}
		else {
			log_panel_dialog_->hide ( );
		}
	}

	void MainWindow::onActionStartSlamComputation ( ) {

		watcher_->disconnect ( this );

		if ( !computation_configured_ ) {
			ui_.statusbar->showMessage ( "SLAM computation not configured, configure at first." );
			QMessageBox::warning ( this , "Warning" , "SLAM computation not configured." );
			return;
		}

		assert ( not keyframes_ptr_->empty ( ) );

		std::cout << "MainWindow::onActionStartSlamComputation - data size: " << keyframes_.size ( ) << std::endl;

		QProgressDialog dialog;
		dialog.setModal ( true );
		dialog.setAutoClose ( true );
		dialog.setLabelText ( QString ( "Computing ... (using %1 thread(s).)" ).arg ( QThread::idealThreadCount ( ) ) );
		dialog.setFixedSize ( dialog.sizeHint ( ) );

		connect ( watcher_ , SIGNAL ( finished ( ) ) , this , SLOT ( OnSlamComputationCompleted ( ) ) );
		connect ( computer_ , SIGNAL ( SetProgressValue ( int ) ) , & dialog , SLOT ( setValue ( int ) ) );
		connect ( computer_ , SIGNAL ( SetProgressRange ( int , int ) ) , & dialog , SLOT ( setRange ( int , int ) ) );
		connect ( & dialog , SIGNAL ( canceled ( ) ) , watcher_ , SLOT ( cancel ( ) ) );

		watcher_->setFuture ( QtConcurrent::run ( computer_ , & SlamComputer::StartCompute ) );

		dialog.exec ( );

	}

	void MainWindow::OnSlamComputationCompleted ( ) {

		if ( keyframes_ptr_->empty ( ) ) {
			return;
		}

		const auto size = static_cast<int>(keyframes_ptr_->size ( ));

		// Since the change of the slider's value will trigger a signal to set the viewer's current begin and end frame,
		// set keyframes must be called at first otherwise the setting will be a fault, bacause the KeyframesGL has not been created.

		// Must important part
		ui_.BasicViewer->SetKeyFrames ( keyframes_ptr_ );
		cloud_play_control_dialog_->SetupUiForNewFrames ( keyframes_ptr_ );

	}

	void MainWindow::onActionOpenInliersViewerMode ( bool checked ) {

		if ( checked ) {

			emit ChangeViewerMode ( 1 );

			connect ( inliers_viewer_option_dialog_ , SIGNAL ( Message ( QString ) ) , log_panel_dialog_ ,
			          SLOT ( AppendMessage ( QString ) ) );

			connect ( inliers_viewer_option_dialog_ , SIGNAL ( SendData ( KeyFrames ) ) , ui_.BasicViewer ,
			          SLOT ( SetKeyFramesForInliers ( KeyFrames ) ) );
			connect ( inliers_viewer_option_dialog_ , SIGNAL ( SendInliers ( CorrespondingPointsPair ) ) , ui_.BasicViewer ,
			          SLOT ( SetInliers ( CorrespondingPointsPair ) ) );
			connect ( inliers_viewer_option_dialog_ , SIGNAL ( SendCorrespondingPoints ( CorrespondingPointsPair ) ) ,
			          ui_.BasicViewer ,
			          SLOT ( SetCorrespondingPoints ( CorrespondingPointsPair ) ) );

			inliers_viewer_option_dialog_->SetKeyFrames ( keyframes_ );
			inliers_viewer_option_dialog_->SetOptions ( computer_->GetOptions ( ) );

			inliers_viewer_option_dialog_->show ( );

		}
		else {
			emit ChangeViewerMode ( 0 );
			inliers_viewer_option_dialog_->hide ( );
		}
	}

	void MainWindow::onActionConfigureSlamComputation ( ) {

		if ( computation_configure_dialog_->exec ( ) == QDialog::Accepted ) {

			computer_->SetOptions ( computation_configure_dialog_->GetOptions ( ) );

			std::cout << computer_->GetOptions ( ).options_one_by_one.Output ( ).toStdString ( ) << std::endl;
			std::cout << computer_->GetOptions ( ).options_pca_keyframe.Output ( ).toStdString ( ) << std::endl;
			std::cout << computer_->GetOptions ( ).options_fixed_frame_count.Output ( ).toStdString ( ) << std::endl;

			emit ConfigurationDone ( );
		}

		else {

			QMessageBox::warning ( this , "Warning" , "Configuration Not Done." );
		}

	}

	void MainWindow::onActionCaptureModelImage ( ) {

		QDir dir ( data_dir_.absolutePath ( ) + "/ScreenShots" );
		if ( !dir.exists ( ) ) dir.mkdir ( dir.absolutePath ( ) );

		ui_.BasicViewer->grabFramebuffer ( ).save ( QString ( "%1/%2.png" )
				                                            .arg ( dir.absolutePath ( ) )
				                                            .arg ( time ( nullptr ) ) );


	}

	void MainWindow::onActionUsePreviousResult ( ) {

		if ( computer_->CheckPreviousResult ( ) ) {

			QString result_cache_name = QFileDialog::getOpenFileName ( this , "Open Cache File" , data_dir_.absolutePath ( ) ,
			                                                           "*.cache" );
			if ( !result_cache_name.isEmpty ( ) ) {
				computer_->UsePreviousResult ( result_cache_name );
				OnSlamComputationCompleted ( );
			}
		}
		else {
			QMessageBox::information ( this , "Privious result not found" , QString ( "Privious cached result not found." ) ,
			                           QMessageBox::Ok );
		}
	}

	void MainWindow::onActionStopSlamComputation ( ) {

	}

	void MainWindow::keyPressEvent ( QKeyEvent * e ) {

	}

	void MainWindow::onActionShowMarkerViewer ( ) {

		if ( !keyframes_.empty ( ) ) {
			marker_viewer_dialog_->show ( );
			marker_viewer_dialog_->SetKeyFrames ( keyframes_ );
		}

		else {

			QMessageBox::information ( this , "Warning" , "No data is loaded.\nLoad at first." , QMessageBox::Ok );

		}
	}

	void MainWindow::onActionOutputResult ( ) {

		if ( keyframes_ptr_->empty ( ) ) {

			QMessageBox::information ( this , "Warning" , "No computation results found." , QMessageBox::Ok );

		}

		else {

			computer_->WriteResult ( );
			QMessageBox::information ( this , "Succeeded" , "Reuslt cache file has been written to \n\"./Caches\"." );

			// marker_viewer_dialog_->SetKeyFrames ( keyframes_ );
			// int result = marker_viewer_dialog_->exec ( );

//			if ( result == QDialog::Accepted ) {
//
//				// std::pair < glm::vec3 , glm::vec3 > marker_points_pair = marker_viewer_dialog_->GetPointPair ( );
//				// WriteResult ( marker_points_pair );
//			}
//			else {
//				QMessageBox::information ( this , "Warning" , "You need to selected 2 marker points." , QMessageBox::Ok );
//			}
		}

	}

	void MainWindow::onActionGenerateAnswer ( ) {

		if ( keyframes_ptr_->empty ( ) ) {
			return;
		}

		std::cout << "MainWindow::onActionStartSlamAnswerComputation - data size: " << keyframes_ptr_->size ( ) <<
		std::endl;

		QProgressDialog dialog;
		dialog.setModal ( true );
		dialog.setAutoClose ( true );
		dialog.setLabelText (
				QString ( "Generating answers ... (using %1 thread(s).)" ).arg ( QThread::idealThreadCount ( ) ) );
		dialog.setFixedSize ( dialog.sizeHint ( ) );


		connect ( watcher_ , SIGNAL ( finished ( ) ) , this , SLOT ( OnSlamComputationCompleted ( ) ) );
		connect ( computer_ , SIGNAL ( SetProgressValue ( int ) ) , & dialog , SLOT ( setValue ( int ) ) );
		connect ( computer_ , SIGNAL ( SetProgressRange ( int , int ) ) , & dialog , SLOT ( setRange ( int , int ) ) );
		connect ( & dialog , SIGNAL ( canceled ( ) ) , watcher_ , SLOT ( cancel ( ) ) );

		watcher_->setFuture ( QtConcurrent::run ( computer_ , & SlamComputer::StartGenerateAnswer ) );

		dialog.exec ( );
	}
}
