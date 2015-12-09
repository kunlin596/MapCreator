//
// Created by LinKun on 12/2/15.
//

#ifndef NIS_CONTROLPANELDIALOG_H
#define NIS_CONTROLPANELDIALOG_H

#include <QDialog>

#include "../../../bin/lib/MapCreator/ui_ControlPanelDialog.h"


namespace Ui {

	class ControlPanelDialog;

}

namespace NiS {

	class ControlPanelDialog : public QDialog
	{
	Q_OBJECT

	signals:

		void ResetCamera ( );
		void ResetModel ( );
		void ChangePointCloudDensity ( int );
		void SetTopView ( int );
		void SetSpinModel ( int );
		void SetShowPointCloud ( int );
		void SetShowTrajectory ( int );
		void SetShowAnswer ( int );
		void SetShowGrid ( int );

	public:

		ControlPanelDialog ( QWidget * parent = 0 ) : ui_ ( new Ui::ControlPanelDialog ) {

			ui_->setupUi ( this );
			setModal ( false );
			setFixedSize ( sizeHint ( ) );

			connect ( ui_->PushButton_ResetCamera , SIGNAL ( pressed ( ) ) , SIGNAL( ResetCamera ( ) ) );
			connect ( ui_->PushButton_ResetModel , SIGNAL ( pressed ( ) ) , SIGNAL( ResetModel ( ) ) );
			connect ( ui_->HorizontalSlider_PointCloudDensity , SIGNAL( valueChanged ( int ) ) , SIGNAL( ChangePointCloudDensity ( int ) ) );
			connect ( ui_->CheckBox_ViewDirection_Top , SIGNAL( stateChanged ( int ) ) , SIGNAL( ChangePointCloudDensity ( int ) ) );
			connect ( ui_->CheckBox_SpinModel , SIGNAL( stateChanged ( int ) ) , SIGNAL( SetSpinModel ( int ) ) );
			connect ( ui_->CheckBox_ShowPointCloud , SIGNAL( stateChanged ( int ) ) , SIGNAL( SetShowPointCloud ( int ) ) );
			connect ( ui_->CheckBox_ShowTrajectory , SIGNAL( stateChanged ( int ) ) , SIGNAL( SetShowTrajectory ( int ) ) );
			connect ( ui_->CheckBox_ShowAnswer , SIGNAL( stateChanged ( int ) ) , SIGNAL( SetShowAnswer ( int ) ) );
			connect ( ui_->CheckBox_ShowGrid , SIGNAL( stateChanged ( int ) ) , SIGNAL( SetShowGrid ( int ) ) );
		}

	private:

		Ui::ControlPanelDialog * ui_;

	};
}


#endif //NIS_CONTROLPANELDIALOG_H
