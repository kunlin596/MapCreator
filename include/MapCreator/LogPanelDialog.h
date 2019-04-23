//
// Created by LinKun on 10/21/15.
//

#ifndef NIS_LOGPANELDIALOG_H
#define NIS_LOGPANELDIALOG_H

#include <QDialog>

#include "../../../bin/lib/MapCreator/ui_LogPanelDialog.h"

namespace Ui { class LogPanelDialog; }

namespace MapCreator {

	class LogPanelDialog : public QDialog
	{

	Q_OBJECT

	public:

		LogPanelDialog ( QWidget * parent = 0 );

	public slots:

		void AppendMessage ( QString );

	private:

		Ui::LogPanelDialog ui_;

	};
}


#endif //NIS_LOGPANELDIALOG_H
