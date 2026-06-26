//
// Created by LinKun on 10/20/15.
//

#ifndef MAPCREATOR_PCAKEYFRAME_FRAMETRACKINGMETHODDIALOG_H
#define MAPCREATOR_PCAKEYFRAME_FRAMETRACKINGMETHODDIALOG_H

#include <QDialog>
#include <QAbstractButton>

#include "Core/SlamParameters.h"

namespace Ui {
	class PcaKeyFrame_FrameTrackingMethodDialog;
}


namespace MapCreator {

	class PcaKeyFrame_FrameTrackingMethodDialog : public QDialog
	{

	Q_OBJECT

	public:

		PcaKeyFrame_FrameTrackingMethodDialog ( QWidget * parent = 0 );

		inline KeyFrameOnlyTrackerParameters GetParameters ( ) { return params_; }

	private slots:

		void onResultButtonBoxClicked ( QAbstractButton * button );

	private:

		bool IsValidInput ( );
		void AcceptDialog ( );
		void RejectDialog ( );
		void RestoreDefaultSettings ( );

		Ui::PcaKeyFrame_FrameTrackingMethodDialog* ui_;

		KeyFrameOnlyTrackerParameters params_;

	};
}


#endif //MAPCREATOR_PCAKEYFRAME_FRAMETRACKINGMETHODDIALOG_H
