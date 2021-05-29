//
// Created by LinKun on 10/21/15.
//

#include "ui_LogPanelDialog.h"
#include "Engine/LogPanelDialog.h"

namespace MapCreator {

	LogPanelDialog::LogPanelDialog ( QWidget * parent ):
        ui_(new Ui::LogPanelDialog) {

		ui_->setupUi ( this );
	}

	void LogPanelDialog::AppendMessage ( QString message ) {

		ui_->PlainTextEdit_Log->appendPlainText ( message );
	}

}