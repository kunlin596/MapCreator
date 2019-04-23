//
// Created by LinKun on 10/20/15.
//

#include "MapCreator/OneByOne_FrameTrackingMethodDialog.h"

#include <iostream>

namespace MapCreator {

	OneByOne_FrameTrackingMethodDialog::OneByOne_FrameTrackingMethodDialog ( QWidget * parent ) {

		ui_.setupUi ( this );

		connect ( ui_.ButtonBox_ResultButtons , SIGNAL ( clicked ( QAbstractButton * ) ) , this ,
		          SLOT( onResultButtonBoxClicked ( QAbstractButton * ) ) );


	}

	void OneByOne_FrameTrackingMethodDialog::AcceptDialog ( ) {

		if ( IsValidInput ( ) ) {
			QDialog::accept ( );
		}
	}

	void OneByOne_FrameTrackingMethodDialog::RejectDialog ( ) {

		QDialog::reject ( );
	}

	void OneByOne_FrameTrackingMethodDialog::RestoreDefaultSettings ( ) {

		ui_.LineEdit_NumRansacIteration->setText ( QString::number ( 10000 ) );
		ui_.LineEdit_OutlierThreshold->setText ( QString::number ( 0.035 ) );
		ui_.LineEdit_InlierThreshold->setText ( QString::number ( 0.035 ) );
	}

	bool OneByOne_FrameTrackingMethodDialog::IsValidInput ( ) {

		bool conversion_succeeded;

		int val1 = ui_.LineEdit_NumRansacIteration->text ( ).toInt ( & conversion_succeeded , 10 );
		if ( !conversion_succeeded ) return false;

		float val2 = ui_.LineEdit_OutlierThreshold->text ( ).toFloat ( & conversion_succeeded );
		if ( !conversion_succeeded ) return false;

		float val3 = ui_.LineEdit_InlierThreshold->text ( ).toFloat ( & conversion_succeeded );
		if ( !conversion_succeeded ) return false;

		if ( val1 < 0 or val2 < 0 or val3 < 0 )
			return false;

		options_.num_ransac_iteration = val1;
		options_.threshold_outlier    = val2;
		options_.threshold_inlier     = val3;

		return true;
	}

	void OneByOne_FrameTrackingMethodDialog::onResultButtonBoxClicked ( QAbstractButton * button ) {

		QPushButton * _button = ( QPushButton * ) ( button );

		if ( _button == ( ui_.ButtonBox_ResultButtons->button ( QDialogButtonBox::RestoreDefaults ) ) ) {

			RestoreDefaultSettings ( );
		}
		else if ( _button == ( ui_.ButtonBox_ResultButtons->button ( QDialogButtonBox::Apply ) ) ) {

			AcceptDialog ( );
		}

		else if ( _button == ( ui_.ButtonBox_ResultButtons->button ( QDialogButtonBox::Cancel ) ) ) {

			RejectDialog ( );
		}
	}

}