//
// Created by LinKun on 10/20/15.
//

#include "MapCreator/PcaKeyFrame_FrameTrackingMethodDialog.h"

namespace NiS {


	PcaKeyFrame_FrameTrackingMethodDialog::PcaKeyFrame_FrameTrackingMethodDialog ( QWidget * parent ) {

		ui_.setupUi ( this );

		connect ( ui_.ButtonBox_ResultButtons , SIGNAL ( clicked ( QAbstractButton * ) ) , this ,
		          SLOT( onResultButtonBoxClicked ( QAbstractButton * ) ) );


	}

	bool PcaKeyFrame_FrameTrackingMethodDialog::IsValidInput ( ) {

		bool conversion_succeeded;

		int val1 = ui_.LineEdit_NumRansacIteration->text ( ).toInt ( & conversion_succeeded , 10 );
		if ( !conversion_succeeded ) return false;

		float val2 = ui_.LineEdit_OutlierThreshold->text ( ).toFloat ( & conversion_succeeded );

		if ( !conversion_succeeded ) return false;

		float val3 = ui_.LineEdit_InlierThreshold->text ( ).toFloat ( & conversion_succeeded );
		if ( !conversion_succeeded ) return false;

		float val4 = ui_.LineEdit_Threshold_1st_PC_Contribution->text ( ).toFloat ( & conversion_succeeded );
		if ( !conversion_succeeded ) return false;

		float val5 = ui_.LineEdit_Threshold_1st_PC_Variance->text ( ).toFloat ( & conversion_succeeded );
		if ( !conversion_succeeded ) return false;

		float val6 = ui_.LineEdit_Threshold_2nd_PC_Variance->text ( ).toFloat ( & conversion_succeeded );
		if ( !conversion_succeeded ) return false;

		float val7 = ui_.LineEdit_Threshold_3rd_PC_Variance->text ( ).toFloat ( & conversion_succeeded );
		if ( !conversion_succeeded ) return false;

		int val8 = ui_.LineEdit_Threshold_InliersNumber->text ( ).toInt ( & conversion_succeeded , 10 );
		if ( !conversion_succeeded ) return false;

		if ( val1 < 0 or val2 < 0 or val3 < 0 or val4 < 0 or val5 < 0 or val6 < 0 or val7 < 0 or val8 < 0 )
			return false;

		options_.num_ransac_iteration                 = val1;
		options_.threshold_outlier                    = val2;
		options_.threshold_inlier                     = val3;
		options_.threshold_1st_component_contribution = val4;
		options_.threshold_1st_component_variance     = val5;
		options_.threshold_2nd_component_variance     = val6;
		options_.threshold_3rd_component_variance     = val7;
		options_.num_inliers                          = val8;

		return true;
	}


	void PcaKeyFrame_FrameTrackingMethodDialog::onResultButtonBoxClicked ( QAbstractButton * button ) {

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


	void PcaKeyFrame_FrameTrackingMethodDialog::AcceptDialog ( ) {

		if ( IsValidInput ( ) ) {
			QDialog::accept ( );
		}
	}

	void PcaKeyFrame_FrameTrackingMethodDialog::RejectDialog ( ) {

		QDialog::reject ( );
	}

	void PcaKeyFrame_FrameTrackingMethodDialog::RestoreDefaultSettings ( ) {

		ui_.LineEdit_NumRansacIteration->setText ( QString::number ( 10000 ) );
		ui_.LineEdit_OutlierThreshold->setText ( QString::number ( 0.035 ) );
		ui_.LineEdit_InlierThreshold->setText ( QString::number ( 0.035 ) );

		ui_.LineEdit_Threshold_1st_PC_Contribution->setText ( QString::number ( 0.85 ) );
		ui_.LineEdit_Threshold_1st_PC_Variance->setText ( QString::number ( 0.1 ) );
		ui_.LineEdit_Threshold_2nd_PC_Variance->setText ( QString::number ( 0.05 ) );
		ui_.LineEdit_Threshold_3rd_PC_Variance->setText ( QString::number ( 0.0 ) );
		ui_.LineEdit_Threshold_InliersNumber->setText ( QString::number ( 3 ) );
	}


}