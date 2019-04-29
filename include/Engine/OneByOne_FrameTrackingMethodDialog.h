//
// Created by LinKun on 10/20/15.
//

#ifndef MAPCREATOR_ONEBYONE_FRAMETRACKINGMETHODDIALOG_H
#define MAPCREATOR_ONEBYONE_FRAMETRACKINGMETHODDIALOG_H

#include <QDialog>

#include "ui_OneByOne_FrameTrackingMethodDialog.h"

#include <SLAM/Option.h>

namespace Ui {

	class OneByOne_FrameTrackingMethodDialog;
}

namespace MapCreator {

	class OneByOne_FrameTrackingMethodDialog : public QDialog
	{

	Q_OBJECT
	public:

		OneByOne_FrameTrackingMethodDialog ( QWidget * parent = 0 );

		inline Options::Options_OneByOne GetOptions ( ) { return options_; }

	private slots:

		void onResultButtonBoxClicked ( QAbstractButton * button );

	private:

		bool IsValidInput ( );
		void AcceptDialog ( );
		void RejectDialog ( );
		void RestoreDefaultSettings ( );

	private:

		Ui::OneByOne_FrameTrackingMethodDialog ui_;

		Options::Options_OneByOne options_;

	};
}

#endif //MAPCREATOR_ONEBYONE_FRAMETRACKINGMETHODDIALOG_H
