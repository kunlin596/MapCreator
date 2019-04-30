//
// Created by LinKun on 10/20/15.
//

#include "ui_FixedFrameCount_FrameTrackingMethodDialog.h"
#include "Engine/FixedFrameCount_FrameTrackingMethodDialog.h"

namespace MapCreator {

	FixedFrameCount_FrameTrackingMethodDialog::FixedFrameCount_FrameTrackingMethodDialog ( QWidget * parent ):
		ui_(new Ui::FixedFrameCount_FrameTrackingMethodDialog) {

		ui_->setupUi ( this );

		connect ( ui_->ButtonBox_ResultButtons , SIGNAL ( clicked ( QAbstractButton * ) ) , this ,
		          SLOT( onResultButtonBoxClicked ( QAbstractButton * ) ) );

	}

	void FixedFrameCount_FrameTrackingMethodDialog::onResultButtonBoxClicked ( QAbstractButton * button ) {

		QPushButton * _button = ( QPushButton * ) ( button );

		if ( _button == ( ui_->ButtonBox_ResultButtons->button ( QDialogButtonBox::RestoreDefaults ) ) ) {
			RestoreDefaultSettings ( );
		}

		else if ( _button == ( ui_->ButtonBox_ResultButtons->button ( QDialogButtonBox::Apply ) ) ) {
			AcceptDialog ( );
		}

		else if ( _button == ( ui_->ButtonBox_ResultButtons->button ( QDialogButtonBox::Cancel ) ) ) {
			RejectDialog ( );
		}
	}

	void FixedFrameCount_FrameTrackingMethodDialog::AcceptDialog ( ) {

		if ( IsValidInput ( ) ) {
			QDialog::accept ( );
		}
	}

	void FixedFrameCount_FrameTrackingMethodDialog::RejectDialog ( ) {

		QDialog::reject ( );
	}

	void FixedFrameCount_FrameTrackingMethodDialog::RestoreDefaultSettings ( ) {

		ui_->LineEdit_NumRansacIteration->setText ( QString::number ( 10000 ) );
		ui_->LineEdit_OutlierThreshold->setText ( QString::number ( 0.035 ) );
		ui_->LineEdit_InlierThreshold->setText ( QString::number ( 0.035 ) );
		ui_->LineEdit_FrameCount->setText ( QString::number ( 1 ) );
	}

	bool FixedFrameCount_FrameTrackingMethodDialog::IsValidInput ( ) {

		bool conversion_succeeded;

		int val1 = ui_->LineEdit_NumRansacIteration->text ( ).toInt ( & conversion_succeeded , 10 );
		if ( !conversion_succeeded ) return false;

		float val2 = ui_->LineEdit_OutlierThreshold->text ( ).toFloat ( & conversion_succeeded );

		if ( !conversion_succeeded ) return false;

		float val3 = ui_->LineEdit_InlierThreshold->text ( ).toFloat ( & conversion_succeeded );
		if ( !conversion_succeeded ) return false;

		int val4 = ui_->LineEdit_FrameCount->text ( ).toInt ( & conversion_succeeded , 10 );
		if ( !conversion_succeeded ) return false;

		if ( val1 < 0 or val2 < 0 or val3 < 0 or val4 < 0 )
			return false;

		options_.num_ransac_iteration = val1;
		options_.threshold_outlier    = val2;
		options_.threshold_inlier     = val3;
		options_.frame_count          = val4;

		return true;
	}
}