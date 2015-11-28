//
// Created by LinKun on 11/15/15.
//

#include "BasicViewer/FrameViewer.h"

#include <iostream>

#include <QFileDialog>
#include <QImage>


#include <Handler/ImageDataHandler.h>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <qtimer.h>


namespace NiS {

	FrameViewer::FrameViewer ( QWidget * parent ) {

		ui_.setupUi ( this );

		connect ( ui_.PushButton_Open , & QPushButton::clicked , this , & FrameViewer::onOpenButtonClicked );
//		connect ( ui_.ListWidget_FileList , SIGNAL( currentRowChanged ( int ) ) , this , SLOT( onFileListCurrentItemChanged ( int ) ) );

		dir_model_  = new QFileSystemModel ( this );
		file_model_ = new QFileSystemModel ( this );

		dir_model_->setReadOnly ( true );
		dir_model_->setReadOnly ( true );

		connect ( ui_.TreeView_DataFolder , & QTreeView::clicked , this , & FrameViewer::onTreeViewItemClicked );
		connect ( ui_.ListView_DataFileList , & QListView::clicked , this , & FrameViewer::onListViewItemClicked );

	}

//	void FrameViewer::onAddFileButtonPushed ( ) {
//
//		std::cout << "onAddFisleButtonPushed" << std::endl;
//
//		QString file_name = QFileDialog::getOpenFileName ( this , "Add File" , "." , "*.dat" );
//		ui_.ListWidget_FileList->addItem ( file_name );
//
//	}
//
//	void FrameViewer::onAddFilesButtonPushed ( ) {
//
//		std::cout << "onAddFilesButtonPushed" << std::endl;
//
//
//		QDir dir ( dir_path );
//
//		QStringList filters;
//		filters.push_back ( QString ( "*.dat" ) );
//		QFileInfoList file_info_list = dir.entryInfoList ( filters );
//
//		for ( int i = 0 ; i < file_info_list.size ( ) ; ++i ) {
//
//			auto & file_info = file_info_list[ i ];
//
//			QListWidgetItem * item = new QListWidgetItem ( QString ( "%1 : (%2)" )
//					                                               .arg ( i + 1 )
//					                                               .arg ( file_info.baseName ( ) ) ,
//			                                               ui_.ListWidget_FileList );
//			item->setData ( Qt::UserRole , file_info.absoluteFilePath ( ) );
//			ui_.ListWidget_FileList->addItem ( item );
//		}
//	}
//
//	void FrameViewer::onClearButtonPushed ( ) {
//
//		std::cout << "onCLearButtonPushed" << std::endl;
//
//		ui_.ListWidget_FileList->clear ( );
//	}

//	void FrameViewer::onFileListCurrentItemChanged ( int row ) {
//
//		if ( row < 0 ) {
//			return;
//		}
//
//		std::cout << ui_.ListWidget_FileList->item ( row )->data ( Qt::UserRole ).toString ( ).toStdString ( ) << std::endl;
//
//		NiS::RawDataFrame frame = NiS::ImageHandler2::ReadFrame ( ui_.ListWidget_FileList->item ( row )->data ( Qt::UserRole ).toString ( ) );
//
//		color_image_ = frame.color_image;
//		depth_image_ = frame.depth_image;
//
//		onDetectMarkerButtonPushed ( );
//
////		UpdateDisplayImage ( );
//	}

	void FrameViewer::onDetectMarkerButtonPushed ( ) {

		using namespace aruco;
		using namespace cv;

		if ( not color_image_.empty ( ) and not depth_image_.empty ( ) ) {

			cv::Mat depth_image_rgb;
			depth_image_.convertTo ( depth_image_rgb , CV_8UC1 , 255.0f / 10000.0f );
			cv::cvtColor ( depth_image_rgb , depth_image_rgb , CV_GRAY2RGB );

			depth_image_rgb_ = depth_image_rgb;

			MarkerDetector    marker_detector;
			vector < Marker > markers;

			marker_detector.detect ( color_image_ , markers );

			has_marker_ = ( not markers.empty ( ) );

			for ( auto const & marker : markers ) {
				marker.draw ( color_image_ , Scalar ( 0 , 0 , 255 ) , 2 );

				bool valid_marker = true;
				for ( const auto & vertex : marker ) {

					const auto row = cvRound ( vertex.y );
					const auto col = cvRound ( vertex.x );

					std::cout << depth_image_.at < ushort > ( row , col ) << std::endl;

					if ( depth_image_.at < ushort > ( row , col ) == 0 ) {

						std::cout << marker.id << " : [" << vertex.y << ", " << vertex.x << "], depth == 0\n";

						valid_marker = false;
					}
				}

				if ( valid_marker ) {
					marker.draw ( depth_image_rgb_ , Scalar ( 0 , 0 , 255 ) , 2 );
				}
			}

			UpdateDisplayImage ( );
		}
	}

	void FrameViewer::resizeEvent ( QResizeEvent * e ) {

		UpdateDisplayImage ( );
	}

	void FrameViewer::UpdateDisplayImage ( ) {

		if ( not color_image_.empty ( ) ) {

			QImage display_color_image ( color_image_.data ,
			                             color_image_.cols ,
			                             color_image_.rows ,
			                             QImage::Format::Format_RGB888 );

			ui_.Label_ColorImage->setPixmap ( QPixmap::fromImage ( display_color_image ).scaled ( ui_.Label_ColorImage->size ( ) ,
			                                                                                      Qt::KeepAspectRatio ,
			                                                                                      Qt::SmoothTransformation ) );
		}

		if ( not depth_image_rgb_.empty ( ) ) {

//			cv::Mat_ < uchar > depth_image_8bit;
//			depth_image_.convertTo ( depth_image_8bit , CV_8UC1 , 255.0f / 10000.0f );

//			QImage display_depth_image ( depth_image_8bit.data ,
//			                             depth_image_8bit.cols ,
//			                             depth_image_8bit.rows ,
//			                             QImage::Format::Format_Grayscale8 );

			QImage display_depth_image ( depth_image_rgb_.data ,
			                             depth_image_rgb_.cols ,
			                             depth_image_rgb_.rows ,
			                             QImage::Format::Format_RGB888 );

			ui_.Label_DepthImage->setPixmap ( QPixmap::fromImage ( display_depth_image ).scaled ( ui_.Label_DepthImage->size ( ) ,
			                                                                                      Qt::KeepAspectRatio ,
			                                                                                      Qt::SmoothTransformation ) );

		}
	}


	void FrameViewer::onTryDetection ( ) {

//		for ( auto i = 0 ; i < ui_.ListWidget_FileList->count ( ) ; ++i ) {
//
//			ui_.ListWidget_FileList->setCurrentRow ( i );
//			onFileListCurrentItemChanged ( i );
//
//			if ( has_marker_ ) {
//				ui_.ListWidget_FileList->currentItem ( )->setBackgroundColor ( QColor::fromRgb ( qRgb ( 100 , 100 , 100 ) ) );
//				ui_.ListWidget_FileList->currentItem ( )->setTextColor ( QColor::fromRgb ( qRgb ( 255 , 100 , 100 ) ) );
//			}
//		}
	}

	void FrameViewer::onOpenButtonClicked ( ) {

		QString dir_path = QFileDialog::getExistingDirectory ( this , "Open Data Root Dir" , "." );

		std::cout << dir_path.toStdString ( ) << std::endl;

		dir_model_->setRootPath ( dir_path );
		dir_model_->setFilter ( QDir::NoDotAndDotDot | QDir::AllDirs );
		dir_model_->setNameFilterDisables ( false );

		file_model_->setRootPath ( dir_path );
		file_model_->setFilter ( QDir::NoDotAndDotDot | QDir::Files );
		file_model_->setNameFilters ( QStringList ( ) << "*.dat" );
		file_model_->setNameFilterDisables ( false );

		ui_.TreeView_DataFolder->setModel ( dir_model_ );
		ui_.ListView_DataFileList->setModel ( file_model_ );

		QModelIndex index = dir_model_->index ( dir_path );
		ui_.TreeView_DataFolder->expand ( index );
		ui_.TreeView_DataFolder->scrollTo ( index );
		ui_.TreeView_DataFolder->setCurrentIndex ( index );

		ui_.TreeView_DataFolder->resizeColumnToContents ( 0 );
		ui_.TreeView_DataFolder->hideColumn ( 1 );
		ui_.TreeView_DataFolder->hideColumn ( 2 );
		ui_.TreeView_DataFolder->hideColumn ( 3 );

	}

	void FrameViewer::onTreeViewItemClicked ( QModelIndex index ) {

		QString path = dir_model_->fileInfo ( index ).absoluteFilePath ( );
		ui_.ListView_DataFileList->setRootIndex ( file_model_->setRootPath ( path ) );
	}

	void FrameViewer::onListViewItemClicked ( QModelIndex index ) {

		QString path = file_model_->fileInfo ( index ).absoluteFilePath ( );

		NiS::RawDataFrame frame = NiS::ImageHandler2::ReadFrame ( path );
//
		color_image_ = frame.color_image;
		depth_image_ = frame.depth_image;
//
		onDetectMarkerButtonPushed ( );
	}
}