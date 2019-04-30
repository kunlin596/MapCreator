//
// Created by LinKun on 11/15/15.
//

#include "BasicViewer/FrameViewer.h"
#include "Handler/ImageDataHandler.h"

#include <iostream>

#include <QFileDialog>
#include <QImage>

// #include <aruco/aruco.h>
// #include <aruco/cvdrawingutils.h>
#include <qtimer.h>

#include "ui_FrameViewer.h"

namespace {
	using namespace cv;
}


namespace MapCreator {

	FrameViewer::FrameViewer ( QWidget * parent )
		: ui_(new Ui::FrameViewer) {

		ui_->setupUi ( this );

		connect ( ui_->PushButton_AddFile , & QPushButton::clicked , this , & FrameViewer::onAddFileButtonPushed );
		connect ( ui_->PushButton_AddFiles , & QPushButton::clicked , this , & FrameViewer::onAddFilesButtonPushed );
		connect ( ui_->PushButton_Clear , & QPushButton::clicked , this , & FrameViewer::onClearButtonPushed );
		// FIXME
		// connect ( ui_->PushButton_TryDetection , & QPushButton::clicked , this , & FrameViewer::onTryDetection );
		connect ( ui_->ListWidget_FileList , SIGNAL( currentRowChanged ( int ) ) , this , SLOT( onFileListCurrentItemChanged ( int ) ) );
	}

	void FrameViewer::onAddFileButtonPushed ( ) {

		std::cout << "onAddFisleButtonPushed" << std::endl;

		QString file_name = QFileDialog::getOpenFileName ( this , "Add File" , "." , "*.dat" );
		ui_->ListWidget_FileList->addItem ( file_name );

	}

	void FrameViewer::onAddFilesButtonPushed ( ) {

		std::cout << "onAddFilesButtonPushed" << std::endl;

		QString dir_path = QFileDialog::getExistingDirectory ( this , "Add Files" , "." );

		QDir dir ( dir_path );

		QStringList filters;
		filters.push_back ( QString ( "*.dat" ) );
		QFileInfoList file_info_list = dir.entryInfoList ( filters );

		for ( int i = 0 ; i < file_info_list.size ( ) ; ++i ) {

			auto & file_info = file_info_list[ i ];

			QListWidgetItem * item = new QListWidgetItem ( QString ( "%1 : (%2)" )
					                                               .arg ( i + 1 )
					                                               .arg ( file_info.baseName ( ) ) ,
			                                               ui_->ListWidget_FileList );
			item->setData ( Qt::UserRole , file_info.absoluteFilePath ( ) );
			ui_->ListWidget_FileList->addItem ( item );
		}
	}

	void FrameViewer::onClearButtonPushed ( ) {

		std::cout << "onCLearButtonPushed" << std::endl;

		ui_->ListWidget_FileList->clear ( );
	}

	void FrameViewer::onFileListCurrentItemChanged ( int row ) {

		if ( row < 0 ) {
			return;
		}

		std::cout << ui_->ListWidget_FileList->item ( row )->data ( Qt::UserRole ).toString ( ).toStdString ( ) << std::endl;

		MapCreator::RawDataFrame frame = MapCreator::ImageHandler2::ReadFrame ( ui_->ListWidget_FileList->item ( row )->data ( Qt::UserRole ).toString ( ) );

		color_image_ = frame.color_image;
		depth_image_ = frame.depth_image;

		onDetectMarkerButtonPushed ( );

		// UpdateDisplayImage ( );
	}

	void FrameViewer::onDetectMarkerButtonPushed ( ) {

		// using namespace aruco;

		if ( not color_image_.empty ( ) and not depth_image_.empty ( ) ) {

			cv::Mat depth_image_rgb;
			depth_image_.convertTo ( depth_image_rgb , CV_8UC1 , 255.0f / 10000.0f );
			cv::cvtColor ( depth_image_rgb , depth_image_rgb , CV_GRAY2RGB );

			depth_image_rgb_ = depth_image_rgb;

			// MarkerDetector    marker_detector;
			// vector < Marker > markers;

			// marker_detector.detect ( color_image_ , markers );

			// has_marker_ = ( not markers.empty ( ) );

			// for ( auto const & marker : markers ) {
			// 	marker.draw ( color_image_ , Scalar ( 0 , 0 , 255 ) , 2 );

			// 	bool valid_marker = true;
			// 	for ( const auto & vertex : marker ) {

			// 		const auto row = cvRound ( vertex.y );
			// 		const auto col = cvRound ( vertex.x );

			// 		std::cout << depth_image_.at < ushort > ( row , col ) << std::endl;

			// 		if ( depth_image_.at < ushort > ( row , col ) == 0 ) {

			// 			std::cout << marker.id << " : [" << vertex.y << ", " << vertex.x << "], depth == 0\n";

			// 			valid_marker = false;
			// 		}
			// 	}

			// 	if ( valid_marker ) {
			// 		marker.draw ( depth_image_rgb_ , Scalar ( 0 , 0 , 255 ) , 2 );
			// 	}
			// }

			// UpdateDisplayImage ( );
		}
	}

	// TODO:
	// void FrameViewer::resizeEvent ( QResizeEvent * e ) {

	// 	UpdateDisplayImage ( );
	// }

	// TODO:
	// void FrameViewer::UpdateDisplayImage ( ) {

	// 	if ( not color_image_.empty ( ) ) {

	// 		QImage display_color_image ( color_image_.data ,
	// 		                             color_image_.cols ,
	// 		                             color_image_.rows ,
	// 		                             QImage::Format::Format_RGB888 );

	// 		ui_->Label_ColorImage->setPixmap ( QPixmap::fromImage ( display_color_image ).scaled ( ui_->Label_ColorImage->size ( ) ,
	// 		                                                                                      Qt::KeepAspectRatio ,
	// 		                                                                                      Qt::SmoothTransformation ) );
	// 	}

	// 	if ( not depth_image_rgb_.empty ( ) ) {

	// 		// cv::Mat_ < uchar > depth_image_8bit;
	// 		// depth_image_.convertTo ( depth_image_8bit , CV_8UC1 , 255.0f / 10000.0f );

	// 		// QImage display_depth_image ( depth_image_8bit.data ,
	// 		//                              depth_image_8bit.cols ,
	// 		//                              depth_image_8bit.rows ,
	// 		//                              QImage::Format::Format_Grayscale8 );

	// 		QImage display_depth_image ( depth_image_rgb_.data ,
	// 		                             depth_image_rgb_.cols ,
	// 		                             depth_image_rgb_.rows ,
	// 		                             QImage::Format::Format_RGB888 );

	// 		ui_->Label_DepthImage->setPixmap ( QPixmap::fromImage ( display_depth_image ).scaled ( ui_->Label_DepthImage->size ( ) ,
	// 		                                                                                      Qt::KeepAspectRatio ,
	// 		                                                                                      Qt::SmoothTransformation ) );

	// 	}
	// }

	// TODO
	// void FrameViewer::onTryDetection ( ) {

	// 	for ( auto i = 0 ; i < ui_->ListWidget_FileList->count ( ) ; ++i ) {

	// 		ui_->ListWidget_FileList->setCurrentRow ( i );
	// 		onFileListCurrentItemChanged ( i );

	// 		if ( has_marker_ ) {
	// 			ui_->ListWidget_FileList->currentItem ( )->setBackgroundColor ( QColor::fromRgb ( qRgb ( 100 , 100 , 100 ) ) );
	// 			ui_->ListWidget_FileList->currentItem ( )->setTextColor ( QColor::fromRgb ( qRgb ( 255 , 100 , 100 ) ) );
	// 		}
	// 	}
	// }

}