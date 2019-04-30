//
// Created by LinKun on 10/21/15.
//

#ifndef MAPCREATOR_LOGPANELDIALOG_H
#define MAPCREATOR_LOGPANELDIALOG_H

#include <QDialog>

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

		Ui::LogPanelDialog* ui_;

	};
}


#endif //MAPCREATOR_LOGPANELDIALOG_H
