//
// Created by LinKun on 10/20/15.
//

#ifndef NIS_PCAKEYFRAME_FRAMETRACKINGMETHODDIALOG_H
#define NIS_PCAKEYFRAME_FRAMETRACKINGMETHODDIALOG_H

#include <QDialog>

#include "../../../bin/lib/MapCreator/ui_PcaKeyFrame_FrameTrackingMethodDialog.h"

#include <SLAM/Option.h>

namespace Ui {

	class PcaKeyFrame_FrameTrackingMethodDialog;
}


namespace NiS {

	class PcaKeyFrame_FrameTrackingMethodDialog : public QDialog
	{

	Q_OBJECT

	public:

		PcaKeyFrame_FrameTrackingMethodDialog ( QWidget * parent = 0 );

		inline Options::Options_PcaKeyFrame GetOptions ( ) { return options_; }

	private slots:

		void onResultButtonBoxClicked ( QAbstractButton * button );

	private:

		bool IsValidInput ( );
		void AcceptDialog ( );
		void RejectDialog ( );
		void RestoreDefaultSettings ( );

		Ui::PcaKeyFrame_FrameTrackingMethodDialog ui_;

		Options::Options_PcaKeyFrame options_;

	};
}


#endif //NIS_PCAKEYFRAME_FRAMETRACKINGMETHODDIALOG_H
