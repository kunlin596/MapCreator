//
// Created by LinKun on 11/15/15.
//

#ifndef NIS_FRAMEVIEWER_H
#define NIS_FRAMEVIEWER_H

#include <SLAM/CommonDefinitions.h>
#include <QWidget>
#include "../../../bin/lib/BasicViewer/ui_FrameViewer.h"

#include <QStringListModel>
#include <QDirModel>
#include <QFileSystemModel>
#include <QModelIndex>

namespace Ui {
	class FrameViewer;
}

namespace NiS {

	class FrameViewer : public QWidget
	{

	Q_OBJECT

	public:

		FrameViewer ( QWidget * parent = 0 );

	signals:

		void GetNextFrame ( int );

	protected:

		void resizeEvent ( QResizeEvent * e ) override;

	private slots:

		void onOpenButtonClicked ( );
//		void onFileListCurrentItemChanged ( int );
		void onDetectMarkerButtonPushed ( );
		void onTryDetection ( );

		void onTreeViewItemClicked ( QModelIndex index );
		void onListViewItemClicked ( QModelIndex index );

	private:

		void UpdateDisplayImage ( );

		Ui::FrameViewer ui_;

		ColorImage color_image_;
		ColorImage depth_image_rgb_;
		DepthImage depth_image_;
		bool       has_marker_;

		QFileSystemModel * dir_model_;
		QFileSystemModel * file_model_;

		QStringList file_list_;

	};


}


#endif //NIS_FRAMEVIEWER_H
