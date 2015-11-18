//
// Created by LinKun on 10/2/15.
//

#ifndef QT5GLVIEWER_BASICVIEWER_H
#define QT5GLVIEWER_BASICVIEWER_H

#include <QOpenGLWidget>
#include <QTimer>

#include "BasicViewer/Camera.h"
#include "BasicViewer/TrajectoryGL.h"
#include "BasicViewer/CorrespondingPointsGL.h"
#include "BasicViewer/PointPairGL.h"
#include "BasicViewer/GridGL.h"
#include "BasicViewer/KeyFrameGL.h"

#include <SLAM/KeyFrame.h>
#include <SLAM/Inliers.h>
#include <SLAM/Alignment.h>
#include <SLAM/Transformation.h>

#include <boost/tuple/tuple.hpp>

//#include "../../Build/BasicViewer/Source/ui_BasicViewer.h"
#include "../../../bin/lib/BasicViewer/ui_BasicViewer.h"

namespace Ui {

	class BasicViewer;
}

namespace NiS {

	class BasicViewer : public QOpenGLWidget
	{

	Q_OBJECT

	public:

		BasicViewer ( QWidget * parent = 0 );

		~BasicViewer ( );

		QOpenGLShaderProgram * SetupShaderProgram ( const QString & vertex_shader_source_path ,
		                                            const QString & fragment_shader_source_path ,
		                                            QObject * parent = 0 );

	protected:

		virtual void initializeGL ( ) override;
		virtual void paintGL ( ) override;
		virtual void resizeGL ( int width , int height ) override;

		void mouseMoveEvent ( QMouseEvent * e ) override;
		void mousePressEvent ( QMouseEvent * e ) override;
		void keyPressEvent ( QKeyEvent * e ) override;
		void wheelEvent ( QWheelEvent * e ) override;

	signals:

		void Message ( QString );

	public slots:

		void SetKeyFrames ( KeyFrames keyframes );
		void SetViewerMode ( int mode );
		void SetCorrespondingPoints ( CorrespondingPointsPair );
		void SetInliers ( CorrespondingPointsPair );
		void SetKeyFramesForInliers ( KeyFrames keyframes );
		void SetEstimationPointPair ( const PointPair & point_pair );
		void SetMarkerPointPair ( const PointPair & point_pair );

	private slots:

		void onSpin ( );
		void onResetCamera ( );
		void onResetModel ( );
		void onChangeDensity ( int value );
		void onTopView ( int option );
		void onSpin ( int option );
		void onRenderGrid ( int option );
		void onRenderTrajectory ( int option );
		void onRenderPointCloud ( int option );
		void onRenderAnswer ( int option );

	private:

		Ui::BasicViewer ui_;

		Camera camera_;
		int    mode_;

		glm::vec4 background_color_;
		float     degree_;
		float     scale_;
		int       density_step_;

		bool render_grid_;
		bool render_inliers_;
		bool render_point_cloud_;
		bool render_trajectory_;
		bool render_answer_;

		std::vector < KeyFrameGL >            keyframes_gl_;
		std::vector < CorrespondingPointsGL > corresponding_points_pair_gl_;
		std::vector < CorrespondingPointsGL > inliers_pair_gl_;
		std::vector < KeyFrameGL >            keyframes_gl_for_inliers_;
		std::vector < TrajectoryGL >          trajectory_gl_;
		std::vector < TrajectoryGL >          answer_trajectory_gl_;
		std::vector < PointPairGL >           estimation_point_pair_gl_;
		std::vector < PointPairGL >           marker_point_pair_gl_;

		QOpenGLShaderProgram      * shader_program_;
		QOpenGLFunctions_4_1_Core * GL;
		GridGL                    * grid_;
		QTimer                    * spin_timer_;

		QPoint last_mouse_position_;

		glm::mat4 model_rotation_matrix_;
		glm::mat4 model_translation_matrix_;

	};
}


#endif //QT5GLVIEWER_BASICVIEWER_H
