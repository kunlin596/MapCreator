//
// Created by LinKun on 10/21/15.
//

#include "MapCreator/LogPanelDialog.h"

namespace NiS {

	LogPanelDialog::LogPanelDialog ( QWidget * parent ) {

		ui_.setupUi ( this );
	}

	void LogPanelDialog::AppendMessage ( QString message ) {

		ui_.PlainTextEdit_Log->appendPlainText ( message );
	}

}