//
// Created by LinKun on 10/21/15.
//

#include "Engine/LogPanelDialog.h"

namespace MapCreator {

	LogPanelDialog::LogPanelDialog ( QWidget * parent ) {

		ui_.setupUi ( this );
	}

	void LogPanelDialog::AppendMessage ( QString message ) {

		ui_.PlainTextEdit_Log->appendPlainText ( message );
	}

}