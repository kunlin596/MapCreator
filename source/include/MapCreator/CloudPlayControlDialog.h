//
// Created by LinKun on 12/2/15.
//

#ifndef NIS_CLOUDPLAYCONTROLDIALOG_H
#define NIS_CLOUDPLAYCONTROLDIALOG_H

#include <QDialog>

#include "../../../bin/lib/MapCreator/ui_CloudPlayControlDialog.h"

namespace Ui {

	class CloudPlayControlDialog;

}

namespace NiS {

	class CloudPlayControlDialog : public QDialog
	{
		Q_OBJECT

	public:

		CloudPlayControlDialog ( QWidget * parent = 0 ) : ui_ ( new Ui::CloudPlayControlDialog ) {

			ui_->setupUi ( this );
			setFixedSize ( sizeHint ( ) );

		}

	private:

		Ui::CloudPlayControlDialog * ui_;

	};

}


#endif //NIS_CLOUDPLAYCONTROLDIALOG_H
