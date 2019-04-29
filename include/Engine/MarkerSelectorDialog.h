//
// Created by LinKun on 10/22/15.
//

#ifndef MAPCREATOR_MARKERSELECTORDIALOG_H
#define MAPCREATOR_MARKERSELECTORDIALOG_H

#include "ui_MarkerSelectorDialog.h"

#include "SLAM/KeyFrame.h"

#include <QDialog>
#include <QMouseEvent>

#include <glm/glm.hpp>

namespace Ui {
	class MarkerSelectorDialog;
}

namespace MapCreator {
	class MarkerSelectorDialog : public QDialog
	{
	Q_OBJECT

	public:

		MarkerSelectorDialog ( QWidget * parent );

		void SetKeyFrame ( const KeyFrame & keyframe );

		inline glm::vec3 GetPoint ( ) const { return point_; }

	signals:

		void FetchPointDone ( );

	protected:

		void mousePressEvent ( QMouseEvent * e ) override;
		void mouseMoveEvent ( QMouseEvent * e ) override;
		void mouseDoubleClickEvent ( QMouseEvent * e ) override;

	private:

		KeyFrame keyframe_;

		glm::vec3 point_;

		Ui::MarkerSelectorDialog ui_;

	};
}


#endif //MAPCREATOR_MARKERSELECTORDIALOG_H
