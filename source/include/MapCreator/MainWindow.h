//
// Created by LinKun on 10/5/15.
//

#ifndef NIS_MAINWINDOW_H
#define NIS_MAINWINDOW_H


#include <QMainWindow>
#include <QtConcurrent>
#include <QThread>

#include "../../../bin/lib/MapCreator/ui_MainWindow.h"

#include <SLAM/Option.h>
#include <SLAM/KeyFrame.h>
#include <Handler/ImageDataHandler.h>
#include "MapCreator/InliersViewerOptionDialog.h"
#include "MapCreator/LogPanelDialog.h"
#include "MapCreator/MarkerViewerDialog.h"
#include "MapCreator/UiDialogs.h"

Q_DECLARE_METATYPE ( NiS::KeyFrames )

namespace Ui { class MainWindow; }

namespace NiS {

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

		void OnReadingFinished ( );
		void OnConversionFinished ( );
		void OnSlamComputationCompleted ( );

		void PrepareComputation ( );

	protected:

		void keyPressEvent ( QKeyEvent * e ) override;

	private:

		Ui::MainWindow ui_;

		void WriteResult ( const std::pair < glm::vec3 , glm::vec3 > & marker_points_pair );

		bool computation_configured_;
		bool computation_done_;

		TrackingType type_;

		KeyFrames keyframes_;
		KeyFrames result_keyframes_;

		ImageHandler2 * handler_;
		SlamComputer  * computer_;

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

#endif //NIS_MAINWINDOW_H
