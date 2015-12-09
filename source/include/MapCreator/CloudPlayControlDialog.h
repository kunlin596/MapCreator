//
// Created by LinKun on 12/2/15.
//

#ifndef NIS_CLOUDPLAYCONTROLDIALOG_H
#define NIS_CLOUDPLAYCONTROLDIALOG_H

#include <QDialog>

#include <memory>
#include <SLAM/KeyFrame.h>

#include "../../../bin/lib/MapCreator/ui_CloudPlayControlDialog.h"

namespace Ui {

	class CloudPlayControlDialog;

}

namespace NiS {

	class CloudPlayControlDialog : public QDialog
	{
	Q_OBJECT

	signals:
		void SetFirstPersonView ( int );
		void StartPlay ( );
		void PausePlay ( );
		void SetBeginFrame ( int );
		void SetEndFrame ( int );

	public:

		CloudPlayControlDialog ( QWidget * parent = 0 ) : ui_ ( new Ui::CloudPlayControlDialog ) {

			ui_->setupUi ( this );
			setFixedSize ( sizeHint ( ) );
			setModal ( false );

			connect ( ui_->CheckBox_FirstPersonView , SIGNAL ( stateChanged ( int ) ) , SIGNAL( SetFirstPersonView ( int ) ) );

			connect ( ui_->HorizontalSlider_Frame1 , SIGNAL ( valueChanged ( int ) ) , SIGNAL( SetBeginFrame ( int ) ) );
			connect ( ui_->HorizontalSlider_Frame2 , SIGNAL ( valueChanged ( int ) ) , SIGNAL( SetEndFrame ( int ) ) );

			connect ( ui_->SpinBox_Frame1 , SIGNAL ( valueChanged ( int ) ) , SIGNAL ( SetBeginFrame ( int ) ) );
			connect ( ui_->SpinBox_Frame2 , SIGNAL ( valueChanged ( int ) ) , SIGNAL ( SetEndFrame ( int ) ) );

			// Change one other through ont other
			connect ( ui_->SpinBox_Frame1 , SIGNAL ( valueChanged ( int ) ) , ui_->HorizontalSlider_Frame1 , SLOT ( setValue ( int ) ) );
			connect ( ui_->SpinBox_Frame2 , SIGNAL ( valueChanged ( int ) ) , ui_->HorizontalSlider_Frame2 , SLOT ( setValue ( int ) ) );
			connect ( ui_->HorizontalSlider_Frame1 , SIGNAL ( valueChanged ( int ) ) , ui_->SpinBox_Frame1 , SLOT( setValue ( int ) ) );
			connect ( ui_->HorizontalSlider_Frame2 , SIGNAL ( valueChanged ( int ) ) , ui_->SpinBox_Frame2 , SLOT( setValue ( int ) ) );

			auto onBeginFrameIsBiggerThanEndFrame  = [ this ] ( int val ) {
				if ( val > ui_->SpinBox_Frame2->value ( ) ) {
					ui_->SpinBox_Frame2->setValue ( val );
				}
			};
			auto onEndFrameIsSmallerThanBeginFrame = [ this ] ( int val ) {
				if ( val < ui_->SpinBox_Frame1->value ( ) ) {
					ui_->SpinBox_Frame1->setValue ( val );
				}
			};

			connect ( ui_->SpinBox_Frame1 , static_cast< void ( QSpinBox::* ) ( int ) >(& QSpinBox::valueChanged) , onBeginFrameIsBiggerThanEndFrame );
			connect ( ui_->SpinBox_Frame2 , static_cast< void ( QSpinBox::* ) ( int ) >(& QSpinBox::valueChanged) , onEndFrameIsSmallerThanBeginFrame );

			connect ( ui_->PushButton_Play , & QPushButton::clicked , this , & CloudPlayControlDialog::StartPlay );
			connect ( ui_->PushButton_Pause , & QPushButton::clicked , this , & CloudPlayControlDialog::PausePlay );


		}

		void SetupUiForNewFrames ( std::weak_ptr < KeyFrames > keyframes ) {

			if ( auto _keyframes = keyframes.lock ( ) ) {
				if ( not _keyframes->empty ( ) ) {
					const auto size = static_cast<int>(_keyframes->size ( ));
					ui_->HorizontalSlider_Frame1->setRange ( 0 , size - 1 );
					ui_->HorizontalSlider_Frame2->setRange ( 0 , size - 1 );
					ui_->SpinBox_Frame1->setRange ( 0 , size - 1 );
					ui_->SpinBox_Frame2->setRange ( 0 , size - 1 );
					ui_->HorizontalSlider_Frame1->setValue ( 0 );
					ui_->HorizontalSlider_Frame2->setValue ( size - 1 );
				}

			}
		};


	private:

		Ui::CloudPlayControlDialog * ui_;

	};

}


#endif //NIS_CLOUDPLAYCONTROLDIALOG_H
