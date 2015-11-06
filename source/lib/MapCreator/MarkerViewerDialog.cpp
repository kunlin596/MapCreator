//
// Created by LinKun on 10/22/15.
//

#include "MapCreator/MarkerViewerDialog.h"

#include <QMouseEvent>
#include <QMessageBox>


namespace NiS {

	MarkerViewerDialog::MarkerViewerDialog ( QWidget * parent ) {

		ui_.setupUi ( this );

		setMouseTracking ( true );

		dialog1_ = new MarkerSelectorDialog ( this );
		dialog2_ = new MarkerSelectorDialog ( this );

		setFixedSize ( sizeHint ( ) );

		connect ( ui_.ButtonBox_ResultButtons , SIGNAL ( clicked ( QAbstractButton * ) ) , this ,
		          SLOT ( onResultButtonsClicked ( QAbstractButton * ) ) );

	}

	MarkerViewerDialog::MarkerViewerDialog ( const KeyFrames & keyframes , QWidget * parent ) :
			keyframes_ ( keyframes ) {

		ui_.setupUi ( this );

		SetKeyFrames ( keyframes );
		setMouseTracking ( true );

		dialog1_ = new MarkerSelectorDialog ( this );
		dialog2_ = new MarkerSelectorDialog ( this );

		setFixedSize ( sizeHint ( ) );

	}

	void MarkerViewerDialog::SetKeyFrames ( const KeyFrames & keyframes ) {

		keyframes_ = keyframes;

		ui_.HorizontalSlider_MarkerImage1->setRange ( 0 , keyframes_.size ( ) - 1 );
		ui_.HorizontalSlider_MarkerImage2->setRange ( 0 , keyframes_.size ( ) - 1 );

		ui_.HorizontalSlider_MarkerImage1->setValue ( 0 );
		ui_.HorizontalSlider_MarkerImage2->setValue ( 0 );

		ui_.SpinBox_MarkerImage1->setRange ( 0 , keyframes_.size ( ) - 1 );
		ui_.SpinBox_MarkerImage2->setRange ( 0 , keyframes_.size ( ) - 1 );

		ui_.SpinBox_MarkerImage1->setValue ( 0 );
		ui_.SpinBox_MarkerImage2->setValue ( 0 );

		ui_.Label_MinImage1->setText ( QString::number ( 0 ) );
		ui_.Label_MinImage2->setText ( QString::number ( 0 ) );

		ui_.Label_MaxImage1->setText ( QString::number ( keyframes_.size ( ) - 1 ) );
		ui_.Label_MaxImage2->setText ( QString::number ( keyframes_.size ( ) - 1 ) );

		SetImage1 ( 0 );
		SetImage2 ( 0 );

		InitializeConnections ( );
	}


	void MarkerViewerDialog::SetImage1 ( int index ) {

		ColorImage color = keyframes_[ index ].GetColorImage ( );
		PointImage point = keyframes_[ index ].GetPointImage ( );

		QImage display_image = QImage ( color.data , color.cols , color.rows , QImage::Format::Format_RGB888 );

		ui_.Label_MarkerImage1->setPixmap ( QPixmap::fromImage ( display_image ).scaled ( ui_.Label_MarkerImage1->width ( ) ,
		                                                                                  ui_.Label_MarkerImage1->height ( ) ,
		                                                                                  Qt::KeepAspectRatio ) );
		ui_.CheckBox_HasPoint1->setChecked ( false );
	}

	void MarkerViewerDialog::SetImage2 ( int index ) {

		ColorImage color = keyframes_[ index ].GetColorImage ( );
		PointImage point = keyframes_[ index ].GetPointImage ( );

		QImage display_image = QImage ( color.data , color.cols , color.rows , QImage::Format::Format_RGB888 );

		ui_.Label_MarkerImage2->setPixmap ( QPixmap::fromImage ( display_image ).scaled ( ui_.Label_MarkerImage2->width ( ) ,
		                                                                                  ui_.Label_MarkerImage2->height ( ) ,
		                                                                                  Qt::KeepAspectRatio ) );
		ui_.CheckBox_HasPoint2->setChecked ( false );
	}

	void MarkerViewerDialog::InitializeConnections ( ) {

		connect ( ui_.HorizontalSlider_MarkerImage1 , SIGNAL( valueChanged ( int ) ) , this , SLOT( SetImage1 ( int ) ) );
		connect ( ui_.HorizontalSlider_MarkerImage2 , SIGNAL( valueChanged ( int ) ) , this , SLOT( SetImage2 ( int ) ) );

		connect ( ui_.SpinBox_MarkerImage1 , SIGNAL( valueChanged ( int ) ) , ui_.HorizontalSlider_MarkerImage1 , SLOT( setValue ( int ) ) );
		connect ( ui_.SpinBox_MarkerImage2 , SIGNAL( valueChanged ( int ) ) , ui_.HorizontalSlider_MarkerImage2 , SLOT( setValue ( int ) ) );

		connect ( ui_.HorizontalSlider_MarkerImage1 , SIGNAL( valueChanged ( int ) ) , ui_.SpinBox_MarkerImage1 , SLOT( setValue ( int ) ) );
		connect ( ui_.HorizontalSlider_MarkerImage2 , SIGNAL( valueChanged ( int ) ) , ui_.SpinBox_MarkerImage2 , SLOT( setValue ( int ) ) );

		connect ( dialog1_ , SIGNAL( FetchPointDone ( ) ) , this , SLOT( onFetchPoint1Done ( ) ) );
		connect ( dialog2_ , SIGNAL( FetchPointDone ( ) ) , this , SLOT( onFetchPoint2Done ( ) ) );
	}


	void MarkerViewerDialog::onFetchPoint1Done ( ) {

		point1_ = dialog1_->GetPoint ( );
		ui_.LineEdit_MarkerImage1_Point->setText ( QString ( "%1, %2, %3" ).arg ( point1_.x ).arg ( point1_.y ).arg ( point1_.z ) );
		ui_.CheckBox_HasPoint1->setChecked ( true );
	}

	void MarkerViewerDialog::onFetchPoint2Done ( ) {

		point2_ = dialog2_->GetPoint ( );
		ui_.LineEdit_MarkerImage2_Point->setText ( QString ( "%1, %2, %3" ).arg ( point2_.x ).arg ( point2_.y ).arg ( point2_.z ) );
		ui_.CheckBox_HasPoint2->setChecked ( true );
	}

	void MarkerViewerDialog::mouseDoubleClickEvent ( QMouseEvent * e ) {

		if ( ui_.Label_MarkerImage1->geometry ( ).contains ( e->pos ( ) ) ) {
			int index = ui_.HorizontalSlider_MarkerImage1->value ( );

			dialog1_->SetKeyFrame ( keyframes_[ index ] );
			dialog1_->exec ( );
			point1_ = dialog1_->GetPoint ( );

		}

		else if ( ui_.Label_MarkerImage2->geometry ( ).contains ( e->pos ( ) ) ) {
			int index = ui_.HorizontalSlider_MarkerImage2->value ( );

			dialog2_->SetKeyFrame ( keyframes_[ index ] );
			dialog2_->exec ( );
			point2_ = dialog2_->GetPoint ( );
		}

	}

	void MarkerViewerDialog::onResultButtonsClicked ( QAbstractButton * button ) {

		QPushButton * _button = ( QPushButton * ) ( button );

		if ( _button == ( ui_.ButtonBox_ResultButtons->button ( QDialogButtonBox::Ok ) ) ) {
			if ( ui_.CheckBox_HasPoint1->isChecked ( ) and ui_.CheckBox_HasPoint2->isChecked ( ) ) {

				QDialog::accept ( );

				glm::mat4 accumulated_matrix;
				for_each ( keyframes_.begin ( ) , keyframes_.end ( ) ,
				           [ & ] ( const KeyFrame & keyframe ) {
					           accumulated_matrix *= keyframe.GetAlignmentMatrix ( );
				           } );

				point2_ = glm::vec3 ( accumulated_matrix * glm::vec4 ( point2_ , 1.0f ) );

				emit SendPointPair ( std::make_pair ( point1_ , point2_ ) );

			} else {
				QMessageBox::information ( this , "Warning" , "A pair of point must be set." );
			}
		}

		else if ( _button == ( ui_.ButtonBox_ResultButtons->button ( QDialogButtonBox::Cancel ) ) ) {
			QDialog::reject ( );
		}
	}
}