//
// Created by LinKun on 10/13/15.
//

#ifndef NIS_INLIERSVIEWEROPTIONDIALOG_H
#define NIS_INLIERSVIEWEROPTIONDIALOG_H

#include <QDialog>

#include "../../../bin/lib/MapCreator/ui_InliersViewerOptionDialog.h"

#include <SLAM/KeyFrame.h>
#include <SLAM/Option.h>

namespace Ui {
	class InliersViewerOptionDialog;
}

namespace NiS {

	class InliersViewerOptionDialog : public QDialog
	{

	Q_OBJECT

	signals:

		void SendData ( KeyFrames );
		void SendCorrespondingPoints ( CorrespondingPointsPair );
		void SendInliers ( CorrespondingPointsPair );

	public slots:

		void SetKeyFrames ( const KeyFrames & );
		inline void SetOptions ( const Options & options ) { options_ = options; }

	private slots:

		void OpenDataFileFolder ( );
		inline void SetFrame1 ( const KeyFrame & frame1 ) { keyframes_for_inliers_[ 0 ] = frame1; };
		inline void SetFrame2 ( const KeyFrame & frame2 ) { keyframes_for_inliers_[ 1 ] = frame2; };

		void onSetNewFrame1 ( int );
		void onSetNewFrame2 ( int );


	public:

		InliersViewerOptionDialog ( QWidget * parent = 0 );
		~InliersViewerOptionDialog ( );

	private:

		void ComputeCorrespondingPoints ( );

		Ui::InliersViewerOptionDialog ui_;

		KeyFrames keyframes_;

		KeyFrames keyframes_for_inliers_;

		Options options_;

		bool has_frame1_;
		bool has_frame2_;

	};
}

#endif //NIS_INLIERSVIEWEROPTIONDIALOG_H
