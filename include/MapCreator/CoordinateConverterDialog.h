//
// Created by LinKun on 11/4/15.
//

#ifndef NIS_COORDINATECONVERTERDIALOG_H
#define NIS_COORDINATECONVERTERDIALOG_H

#include <QDialog>

#include "../../../bin/lib/MapCreator/ui_CoordinateConverterDialog.h"

namespace Ui {
	class CoordinateConverterDialog;
}

namespace MapCreator {

	class CoordinateConverterDialog : public QDialog
	{

	Q_OBJECT

	public:

		CoordinateConverterDialog ( QWidget * parent = 0 ) : choice_ ( -1 ) {

			ui_.setupUi ( this );

			connect ( ui_.ButtonBox_ResultButtons , SIGNAL( clicked ( QAbstractButton * ) ) , this ,
			          SLOT( onResultButtonBoxClicked ( QAbstractButton * ) ) );;

		}

		~CoordinateConverterDialog ( ) = default;

		int GetChoice ( ) const {

			if ( ui_.RadioButton_XtionCoordinateConverter->isChecked ( ) )
				return 0;

			if ( ui_.RadioButton_AistCoordinateConverter->isChecked ( ) )
				return 1;

			return -1;
		}

	private slots:

		void onResultButtonBoxClicked ( QAbstractButton * button ) {

			QPushButton * _button = ( QPushButton * ) ( button );

			if ( _button == ( ui_.ButtonBox_ResultButtons->button ( QDialogButtonBox::Ok ) ) ) {

				QDialog::accept ( );
			}

			else if ( _button == ( ui_.ButtonBox_ResultButtons->button ( QDialogButtonBox::Cancel ) ) ) {
				QDialog::reject ( );
			}
		};

	private:

		int choice_;

		Ui::CoordinateConverterDialog ui_;

	};
}


#endif //NIS_COORDINATECONVERTERDIALOG_H
