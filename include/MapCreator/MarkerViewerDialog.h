//
// Created by LinKun on 10/22/15.
//

#ifndef NIS_MARKERVIEWERDIALOG_H
#define NIS_MARKERVIEWERDIALOG_H


#include <QDialog>

#include "../../../bin/lib/MapCreator/ui_MarkerViewerDialog.h"

#include <SLAM/KeyFrame.h>

#include <MapCreator/MarkerSelectorDialog.h>
#include <glm/glm.hpp>

namespace Ui {
	class MarkerViewerDialog;
}

namespace MapCreator {

	class MarkerViewerDialog : public QDialog
	{
	Q_OBJECT

	public:

		MarkerViewerDialog ( const KeyFrames & keyframes , QWidget * parent = 0 );
		MarkerViewerDialog ( QWidget * parent = 0 );

		void SetKeyFrames ( const KeyFrames & keyframes );

		inline PointPair GetPointPair ( ) const { return std::make_pair ( point1_ , point2_ ); }

	signals:

		void SendEstimationPointPair ( PointPair );
		void SendMarkerPointPair ( PointPair );

	private slots:

		void SetImage1 ( int index );
		void SetImage2 ( int index );

		void onFetchPoint1Done ( );
		void onFetchPoint2Done ( );

		void onResultButtonsClicked ( QAbstractButton * button );

	protected:

		void mouseDoubleClickEvent ( QMouseEvent * e );

	private:

		glm::vec3 point1_;
		glm::vec3 point2_;

		void InitializeConnections ( );

		KeyFrames keyframes_;

		Ui::MarkerViewerDialog ui_;

		MarkerSelectorDialog * dialog1_;
		MarkerSelectorDialog * dialog2_;

	};

}


#endif //NIS_MARKERVIEWERDIALOG_H
