//
// Created by LinKun on 10/5/15.
//

#ifndef MAPCREATOR_MAINWINDOW_H
#define MAPCREATOR_MAINWINDOW_H

#include <unique_ptr>
#include <QMainWindow>
#include <QtConcurrent>
#include <QThread>

#include "SLAM/Option.h"
#include "SLAM/KeyFrame.h"
#include "Handler/ImageDataHandler.h"
#include "Engine/InliersViewerOptionDialog.h"
#include "Engine/LogPanelDialog.h"
#include "Engine/MarkerViewerDialog.h"
#include "Engine/UiDialogs.h"

Q_DECLARE_METATYPE ( MapCreator::KeyFrames )

namespace Ui { class MainWindow; }

namespace MapCreator {

	class MainWindow : public QMainWindow
	{

	Q_OBJECT

	public:

		MainWindow ( QWidget * parent = 0 );
		~MainWindow ( );

	signals:

		void ConfigurationDone ( );
		void ChangeViewerMode ( int );

	public slots:

		void EnableDensitySlider ( );

	private slots:

		void onActionConfigureSlamComputation ( );
		void onActionStartSlamComputation ( );
		void onActionOpenDataFiles ( );
		void onActionOpenInternalCalibrationFile ( );
		void onActionInternalCalibration ( );
		void onActionOpenInliersViewerMode ( bool );
		void onActionCaptureModelImage ( );
		void onActionUsePreviousResult ( );
		void onActionStopSlamComputation ( );
		void onActionShowControlPanel ( );
		void onActionOpenLogPanel ( );
		void onActionShowMarkerViewer ( );
		void onActionOutputResult ( );
		void onActionGenerateAnswer ( );
		void onBeginFrameIsBiggerThanEndFrame ( int );
		void onEndFrameIsSmallerThanBeginFrame ( int );

		void OnReadingFinished ( );
		void OnConversionFinished ( );
		void OnSlamComputationCompleted ( );

		void onPlayButtonClicked ( );
		void onStopButtonClicked ( );
		void onRewindCloud ( );


		void PrepareComputation ( );

	protected:

		void keyPressEvent ( QKeyEvent * e ) override;

	private:

		std::unique_ptr<Ui::MainWindow> ui_;

		void WriteResult ( const std::pair < glm::vec3 , glm::vec3 > & marker_points_pair );
		void ResetWatcher ( );

		bool computation_configured_;
		bool computation_done_;

		TrackingType type_;

		KeyFrames keyframes_;

		ImageHandler2 * handler_;
		SlamAlgorithm  * computer_;

		QTimer                  * play_timer_;
		QFutureWatcher < void > * watcher_;

		Calibrator calibrator_;

		InliersViewerOptionDialog * inliers_viewer_option_dialog_;

		QDir data_dir_;

		// SLAM configuration dialogs
		ComputationConfigureDialog * computation_configure_dialog_;
		LogPanelDialog             * log_panel_dialog_;

		//
		MarkerViewerDialog * marker_viewer_dialog_;
	};
}

#endif //MAPCREATOR_MAINWINDOW_H
