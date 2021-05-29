//
// Created by LinKun on 10/5/15.
//

#include "Engine/MainWindow.h"
#include "Engine/CoordinateConverterDialog.h"
#include "ui_MainWindow.h"

#include <SLAM/Tracker.h>
#include <SLAM/CoordinateConverter.h>

#include <QMessageBox>
#include <QMouseEvent>
#include <QFileDialog>
#include <QDir>
#include <QThread>

namespace MapCreator {

	MainWindow::MainWindow ( QWidget * parent ) :
			ui_(new Ui::MainWindow),
			computation_configured_ ( false ) ,
			computation_done_ ( false ) ,
			play_timer_ ( new QTimer ( this ) ) ,
			watcher_ ( new QFutureWatcher < void > ( this ) ) {

		// Register object for Qt
		qRegisterMetaType < KeyFrames > ( "KeyFrames" );
		qRegisterMetaType < CorrespondingPointsPair > ( "CorrespondingPointsPair" );
		qRegisterMetaType < CorrespondingPointsPair > ( "PointPair" );

		ui_->setupUi ( this );

		log_panel_dialog_ = new LogPanelDialog ( this );
		log_panel_dialog_->setWindowFlags ( Qt::WindowStaysOnTopHint );

		inliers_viewer_option_dialog_ = new InliersViewerOptionDialog ( this );
		inliers_viewer_option_dialog_->setModal ( false );

		ui_->actionOpenInliersViewMode->setChecked ( false );
		ui_->Frame_ControlPanel->hide ( );
		ui_->actionStartSlamComputation->setEnabled ( false );
		ui_->actionInternalCalibration->setEnabled ( false );
		ui_->HorizontalSlider_PointCloudDensity->setEnabled ( false );
		ui_->actionOpenInliersViewMode->setEnabled ( false );
		ui_->actionCaptureModelMage->setEnabled ( false );
		ui_->actionUsePreviousResult->setEnabled ( false );
		ui_->actionConfigureSlamComputation->setEnabled ( false );

		connect ( ui_->PushButton_ResetCamera , SIGNAL ( pressed ( ) ) , ui_->BasicViewer , SLOT ( onResetCamera ( ) ) );
		connect ( ui_->PushButton_ResetModel , SIGNAL ( pressed ( ) ) , ui_->BasicViewer , SLOT ( onResetModel ( ) ) );
		connect ( ui_->HorizontalSlider_PointCloudDensity , SIGNAL ( valueChanged ( int ) ) , ui_->BasicViewer ,
		          SLOT ( onChangeDensity ( int ) ) );
		connect ( ui_->CheckBox_ViewDirection_Top , SIGNAL ( stateChanged ( int ) ) , ui_->BasicViewer , SLOT ( onTopView ( int ) ) );
		connect ( ui_->CheckBox_SpinModel , SIGNAL ( stateChanged ( int ) ) , ui_->BasicViewer , SLOT ( onSpin ( int ) ) );

		connect ( ui_->CheckBox_ShowGrid , SIGNAL ( stateChanged ( int ) ) , ui_->BasicViewer , SLOT ( onRenderGrid ( int ) ) );
		connect ( ui_->CheckBox_ShowPointCloud , SIGNAL ( stateChanged ( int ) ) , ui_->BasicViewer , SLOT ( onRenderPointCloud ( int ) ) );
		connect ( ui_->CheckBox_ShowTrajectory , SIGNAL ( stateChanged ( int ) ) , ui_->BasicViewer , SLOT ( onRenderTrajectory ( int ) ) );
		connect ( ui_->CheckBox_ShowAnswer , SIGNAL ( stateChanged ( int ) ) , ui_->BasicViewer , SLOT ( onRenderAnswer ( int ) ) );

		connect ( ui_->actionConfigureSlamComputation , SIGNAL ( triggered ( ) ) , this ,
		          SLOT ( onActionConfigureSlamComputation ( ) ) );
		connect ( ui_->actionStartSlamComputation , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionStartSlamComputation ( ) ) );
		connect ( ui_->actionOpenDataFiles , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionOpenDataFiles ( ) ) );
		connect ( ui_->actionShowControlPanel , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionShowControlPanel ( ) ) );
		connect ( ui_->actionOpenLogPanel , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionOpenLogPanel ( ) ) );
		connect ( ui_->actionInternalCalibration , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionInternalCalibration ( ) ) );
		connect ( ui_->actionOpenInliersViewMode , SIGNAL ( triggered ( bool ) ) , this ,
		          SLOT ( onActionOpenInliersViewerMode ( bool ) ) );
		connect ( ui_->actionCaptureModelMage , SIGNAL ( triggered ( bool ) ) , this , SLOT ( onActionCaptureModelImage ( ) ) );
		connect ( ui_->actionUsePreviousResult , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionUsePreviousResult ( ) ) );
		connect ( ui_->actionOutputResult , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionOutputResult ( ) ) );

		connect ( ui_->actionGenerateAnswer , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionGenerateAnswer ( ) ) );

		connect ( this , SIGNAL ( ConfigurationDone ( ) ) , this , SLOT ( PrepareComputation ( ) ) );
		connect ( this , SIGNAL ( ChangeViewerMode ( int ) ) , ui_->BasicViewer , SLOT ( SetViewerMode ( int ) ) );

		// UI dialogs initialization
		computation_configure_dialog_ = new ComputationConfigureDialog ( this );
		computation_configure_dialog_->setFixedSize ( computation_configure_dialog_->sizeHint ( ) );

		computer_ = new SlamAlgorithm ( this );
		connect ( computer_ , SIGNAL ( Message ( QString ) ) , log_panel_dialog_ , SLOT ( AppendMessage ( QString ) ) );
		connect ( computer_ , SIGNAL ( SendData ( KeyFrames ) ) , ui_->BasicViewer , SLOT ( SetKeyFrames ( KeyFrames ) ) );

		connect ( ui_->BasicViewer , SIGNAL ( Message ( QString ) ) , log_panel_dialog_ , SLOT ( AppendMessage ( QString ) ) );
		connect ( ui_->actionStopSlamComputation , SIGNAL ( triggered ( ) ) , computer_ , SLOT ( StopCompute ( ) ) );

		// Marker Image Viewer
		marker_viewer_dialog_ = new MarkerViewerDialog ( this );
		connect ( marker_viewer_dialog_ , SIGNAL ( SendEstimationPointPair ( PointPair ) ) , ui_->BasicViewer ,
		          SLOT ( SetEstimationPointPair ( PointPair ) ) );
		connect ( marker_viewer_dialog_ , SIGNAL ( SendMarkerPointPair ( PointPair ) ) , ui_->BasicViewer ,
		          SLOT ( SetMarkerPointPair ( PointPair ) ) );

		connect ( ui_->actionIShowMarkerViewer , SIGNAL ( triggered ( ) ) , this , SLOT ( onActionShowMarkerViewer ( ) ) );


		connect ( ui_->SpinBox_BeginFrame , SIGNAL ( valueChanged ( int ) ) , ui_->HorizontalSlider_BeginFrame , SLOT( setValue ( int ) ) );
		connect ( ui_->SpinBox_BeginFrame , SIGNAL( valueChanged ( int ) ) , this , SLOT( onBeginFrameIsBiggerThanEndFrame ( int ) ) );
		connect ( ui_->SpinBox_EndFrame , SIGNAL ( valueChanged ( int ) ) , ui_->HorizontalSlider_EndFrame , SLOT( setValue ( int ) ) );
		connect ( ui_->SpinBox_EndFrame , SIGNAL ( valueChanged ( int ) ) , this , SLOT( onEndFrameIsSmallerThanBeginFrame ( int ) ) );
		connect ( ui_->HorizontalSlider_BeginFrame , SIGNAL ( valueChanged ( int ) ) , ui_->SpinBox_BeginFrame , SLOT( setValue ( int ) ) );
		connect ( ui_->HorizontalSlider_BeginFrame , SIGNAL( valueChanged ( int ) ) , this , SLOT( onBeginFrameIsBiggerThanEndFrame ( int ) ) );
		connect ( ui_->HorizontalSlider_EndFrame , SIGNAL ( valueChanged ( int ) ) , ui_->SpinBox_EndFrame , SLOT( setValue ( int ) ) );
		connect ( ui_->HorizontalSlider_EndFrame , SIGNAL ( valueChanged ( int ) ) , this , SLOT( onEndFrameIsSmallerThanBeginFrame ( int ) ) );

		connect ( ui_->SpinBox_BeginFrame , SIGNAL( valueChanged ( int ) ) , ui_->BasicViewer , SLOT( SetBeginFrame ( int ) ) );
		connect ( ui_->SpinBox_EndFrame , SIGNAL( valueChanged ( int ) ) , ui_->BasicViewer , SLOT( SetEndFrame ( int ) ) );


		connect ( ui_->PushButton_Play , SIGNAL( clicked ( ) ) , this , SLOT( onPlayButtonClicked ( ) ) );
		connect ( ui_->PushButton_Stop , SIGNAL( clicked ( ) ) , this , SLOT( onStopButtonClicked ( ) ) );
		connect ( play_timer_ , SIGNAL( timeout ( ) ) , this , SLOT( onRewindCloud ( ) ) );

	}

	MainWindow::~MainWindow ( ) {

		delete watcher_;
		delete inliers_viewer_option_dialog_;
	}

	void MainWindow::PrepareComputation ( ) {

		computation_configured_ = true;

		ui_->actionStartSlamComputation->setEnabled ( true );

	}

	void MainWindow::EnableDensitySlider ( ) {

		ui_->HorizontalSlider_PointCloudDensity->setEnabled ( true );
	}

	void MainWindow::OnReadingFinished ( ) {

		ui_->actionInternalCalibration->setEnabled ( true );

	}

	void MainWindow::OnConversionFinished ( ) {

		watcher_->disconnect ( this );

		keyframes_ = handler_->GetKeyFrames ( );

		assert ( !keyframes_.empty ( ) );

		std::cout << "OnConversionFinished - data size : " << keyframes_.size ( ) << std::endl;

		ui_->actionStartSlamComputation->setEnabled ( true );
		ui_->actionOpenInliersViewMode->setEnabled ( true );
		ui_->actionCaptureModelMage->setEnabled ( true );
		ui_->actionUsePreviousResult->setEnabled ( true );
		ui_->actionConfigureSlamComputation->setEnabled ( true );

		computer_->SetDataDir ( data_dir_ );
		computer_->SetFrameData ( keyframes_ );
	}

	void MainWindow::WriteResult ( const std::pair < glm::vec3 , glm::vec3 > & marker_points_pair ) {

		computer_->WriteResult ( marker_points_pair );

	}

	void MainWindow::onActionOpenDataFiles ( ) {

		// ResetWatcher ( );

		watcher_->disconnect ( this );

		ui_->actionStartSlamComputation->setEnabled ( false );

		std::cerr << "Main thread : " << QThread::currentThreadId ( ) << std::endl;

		QString dir_path = QFileDialog::getExistingDirectory ( this , "Open Data File Dir ..." , QDir::homePath ( ) );

		if ( !dir_path.isEmpty ( ) ) {

			data_dir_          = QDir ( dir_path );

			QStringList   name_filers ( QString ( "*.dat" ) );
			QFileInfoList list = data_dir_.entryInfoList ( name_filers );

			handler_ = new ImageHandler2 ( list , this );

			connect ( watcher_ , SIGNAL ( finished ( ) ) , this , SLOT ( OnReadingFinished ( ) ) );
			connect ( handler_ , SIGNAL ( Message ( QString ) ) , log_panel_dialog_ , SLOT ( AppendMessage ( QString ) ) );
			connect ( handler_ , SIGNAL ( Message ( QString ) ) , ui_->statusbar , SLOT ( showMessage ( QString ) ) );

			QFuture < void > reading_result = QtConcurrent::run ( this->handler_ , & ImageHandler2::StartReading );
			watcher_->setFuture ( reading_result );

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

			ui_->actionInternalCalibration->setEnabled ( true );

			log_panel_dialog_->AppendMessage ( QString ( "Internal calibration file loaded :\n%1" ).arg ( path ) );
		}

	}

	void MainWindow::onActionInternalCalibration ( ) {

		// ResetWatcher ( );

		ui_->actionStartSlamComputation->setEnabled ( false );

		std::cerr << "Main thread : " << QThread::currentThreadId ( ) << std::endl;

		assert ( !handler_->GetRawDataFrames ( ).empty ( ) );

		connect ( watcher_ , SIGNAL ( finished ( ) ) , this , SLOT ( OnConversionFinished ( ) ) );

		CoordinateConverterDialog dialog;

		dialog.exec ( );

		int choice = dialog.GetChoice ( );

		std::cout << "Choice is : " << choice << std::endl;

		switch ( choice ) {
			case 0: {
				QFuture < void > reading_result = QtConcurrent::run ( this->handler_ ,
				                                                      & ImageHandler2::ConvertToPointImages , choice );
				computer_->SetCoordinateConverter ( handler_->GetXtionCoordinateConverter ( ) );
				watcher_->setFuture ( reading_result );
				break;
			}
			case 1: {
				onActionOpenInternalCalibrationFile ( );
				QFuture < void > reading_result = QtConcurrent::run ( this->handler_ ,
				                                                      & ImageHandler2::ConvertToPointImages , choice );
				computer_->SetCoordinateConverter ( handler_->GetAistCoordinateConverter ( ) );
				watcher_->setFuture ( reading_result );
				break;
			}

			default:
				return;
		}

//		if ( b == QMessageBox::Yes ) {
//			onActionOpenInternalCalibrationFile ( );
//			QFuture < void > reading_result = QtConcurrent::run ( this->handler_ ,
//			                                                      & ImageHandler2::ConvertToPointImages , 1 );
//			watcher_->setFuture ( reading_result );
//		}
//		else if ( b == QMessageBox::No ) {
//			QFuture < void > reading_result = QtConcurrent::run ( this->handler_ ,
//			                                                      & ImageHandler2::ConvertToPointImages , 0 );
//			watcher_->setFuture ( reading_result );
//		}
	}

	void MainWindow::onActionShowControlPanel ( ) {

		if ( ui_->Frame_ControlPanel->isHidden ( ) ) {
			ui_->Frame_ControlPanel->show ( );
		}
		else {
			ui_->Frame_ControlPanel->hide ( );
		}
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

		// ResetWatcher ( );

		watcher_->disconnect ( this );

		if ( !computation_configured_ ) {
			ui_->statusbar->showMessage ( "SLAM computation not configured, configure at first." );
			QMessageBox::warning ( this , "Warning" , "SLAM computation not configured." );
			return;
		}

		assert ( !keyframes_.empty ( ) );

		std::cout << "MainWindow::onActionStartSlamComputation - data size: " << keyframes_.size ( ) << std::endl;

		assert ( !computer_->GetKeyFrames ( ).empty ( ) );

		connect ( watcher_ , SIGNAL ( finished ( ) ) , this , SLOT ( OnSlamComputationCompleted ( ) ) );
		QFuture < void > compute_result = QtConcurrent::run ( computer_ , & SlamAlgorithm::StartCompute );

		watcher_->setFuture ( compute_result );

		ui_->CheckBox_ShowAnswer->setChecked ( false );

	}

	void MainWindow::OnSlamComputationCompleted ( ) {

		ui_->HorizontalSlider_PointCloudDensity->setEnabled ( true );

		keyframes_ = computer_->GetKeyFrames ( );
		assert ( !keyframes_.empty ( ) );

		ui_->HorizontalSlider_BeginFrame->setRange ( 0 , static_cast<int>(keyframes_.size ( ) - 1 ) );
		ui_->HorizontalSlider_EndFrame->setRange ( 0 , static_cast<int>(keyframes_.size ( ) - 1 ) );

		ui_->SpinBox_BeginFrame->setRange ( 0 , static_cast<int>(keyframes_.size ( ) - 1 ) );
		ui_->SpinBox_EndFrame->setRange ( 0 , static_cast<int>(keyframes_.size ( ) - 1 ) );

		ui_->HorizontalSlider_BeginFrame->setValue ( 0 );
		ui_->HorizontalSlider_EndFrame->setValue ( static_cast<int>(keyframes_.size ( ) - 1 ) );

		ui_->HorizontalSlider_PointCloudDensity->setValue ( 5 );
	}

	void MainWindow::onActionOpenInliersViewerMode ( bool checked ) {

		if ( checked ) {

			emit ChangeViewerMode ( 1 );

			connect ( inliers_viewer_option_dialog_ , SIGNAL( Message ( QString ) ) , log_panel_dialog_ , SLOT( AppendMessage ( QString ) ) );

			connect ( inliers_viewer_option_dialog_ , SIGNAL ( SendData ( KeyFrames ) ) , ui_->BasicViewer ,
			          SLOT ( SetKeyFramesForInliers ( KeyFrames ) ) );
			connect ( inliers_viewer_option_dialog_ , SIGNAL ( SendInliers ( CorrespondingPointsPair ) ) , ui_->BasicViewer ,
			          SLOT ( SetInliers ( CorrespondingPointsPair ) ) );
			connect ( inliers_viewer_option_dialog_ , SIGNAL ( SendCorrespondingPoints ( CorrespondingPointsPair ) ) ,
			          ui_->BasicViewer ,
			          SLOT ( SetCorrespondingPoints ( CorrespondingPointsPair ) ) );

			inliers_viewer_option_dialog_->SetKeyFrames ( keyframes_ );
			inliers_viewer_option_dialog_->SetOptions ( computer_->GetParameters ( ) );

			inliers_viewer_option_dialog_->show ( );
			ui_->HorizontalSlider_PointCloudDensity->setEnabled ( true );

		}
		else {
			emit ChangeViewerMode ( 0 );
			inliers_viewer_option_dialog_->hide ( );
		}
	}

	void MainWindow::onActionConfigureSlamComputation ( ) {

		if ( computation_configure_dialog_->exec ( ) == QDialog::Accepted ) {

			computer_->SetOptions ( computation_configure_dialog_->GetParameters ( ) );

			std::cout << computer_->GetParameters ( ).paramsConsectutive.Output ( ).toStdString ( ) << std::endl;
			std::cout << computer_->GetParameters ( ).paramsKeyFramesOnly.Output ( ).toStdString ( ) << std::endl;
			std::cout << computer_->GetParameters ( ).paramsFixedNumber.Output ( ).toStdString ( ) << std::endl;

			emit ConfigurationDone ( );
		}

		else {

			QMessageBox::warning ( this , "Warning" , "Configuration Not Done." );
		}

	}

	void MainWindow::onActionCaptureModelImage ( ) {

		QDir dir ( data_dir_.absolutePath ( ) + "/ScreenShots" );
		if ( !dir.exists ( ) ) dir.mkdir ( dir.absolutePath ( ) );

		ui_->BasicViewer->grabFramebuffer ( ).save ( QString ( "%1/%2.png" )
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

		if ( keyframes_.empty ( ) ) {

			QMessageBox::information ( this , "Warning" , "No computation results found." , QMessageBox::Ok );

		}

		else {

			computer_->WriteResult ( );

			marker_viewer_dialog_->SetKeyFrames ( keyframes_ );
			int result = marker_viewer_dialog_->exec ( );

			if ( result == QDialog::Accepted ) {

				std::pair < glm::vec3 , glm::vec3 > marker_points_pair = marker_viewer_dialog_->GetPointPair ( );

				WriteResult ( marker_points_pair );
				QMessageBox::information ( this , "Succeeded" , "Reuslt file has been written to \nDATA_DIR/Results." );
			}
			else {
				QMessageBox::information ( this , "Warning" , "You need to selected 2 marker points." , QMessageBox::Ok );
			}
		}

	}

	void MainWindow::onActionGenerateAnswer ( ) {

		// ResetWatcher ( );

		assert ( !keyframes_.empty ( ) );

		std::cout << "MainWindow::onActionStartSlamAnswerComputation - data size: " << keyframes_.size ( ) << std::endl;

		assert ( !computer_->GetKeyFrames ( ).empty ( ) );

		connect ( watcher_ , SIGNAL ( finished ( ) ) , this , SLOT ( OnSlamComputationCompleted ( ) ) );
		QFuture < void > compute_result = QtConcurrent::run ( computer_ , & SlamAlgorithm::StartGenerateAnswer );

		watcher_->setFuture ( compute_result );
		ui_->CheckBox_ShowAnswer->setChecked ( true );

	}

	void MainWindow::ResetWatcher ( ) {

	}

	void MainWindow::onBeginFrameIsBiggerThanEndFrame ( int val ) {

		if ( val > ui_->SpinBox_EndFrame->value ( ) ) {
			ui_->SpinBox_EndFrame->setValue ( val );
		}
	}

	void MainWindow::onEndFrameIsSmallerThanBeginFrame ( int val ) {

		if ( val < ui_->SpinBox_BeginFrame->value ( ) ) {
			ui_->SpinBox_BeginFrame->setValue ( val );
		}
	}


	void MainWindow::onPlayButtonClicked ( ) {

		play_timer_->start ( 33 );

	}

	void MainWindow::onStopButtonClicked ( ) {

		play_timer_->stop ( );

	}


	void MainWindow::onRewindCloud ( ) {

		const auto begin_frame = ui_->SpinBox_BeginFrame->value ( );
		const auto end_frame   = ui_->SpinBox_EndFrame->value ( );

		static bool backwards = true;

		if ( not backwards and end_frame != keyframes_.size ( ) - 1 ) {
			ui_->SpinBox_EndFrame->setValue ( end_frame + 1 );
		} else if ( not backwards and end_frame == keyframes_.size ( ) - 1 ) {
			backwards = true;
		} else if ( backwards and ui_->SpinBox_EndFrame->value ( ) != begin_frame ) {
			ui_->SpinBox_EndFrame->setValue ( end_frame - 1 );
		} else if ( backwards and ui_->SpinBox_EndFrame->value ( ) == begin_frame ) {
			backwards = false;
		}
	}

}
