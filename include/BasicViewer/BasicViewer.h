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
#include "BasicViewer/LineSegmentGL.h"

#include "SLAM/KeyFrame.h"
#include "SLAM/Inliers.h"
#include "SLAM/Alignment.h"
#include "SLAM/Transformation.h"

#include <boost/tuple/tuple.hpp>

namespace Ui { class BasicViewer; }

namespace MapCreator {

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
		void SetBeginFrame ( int begin_frame ) {

			if ( 0 <= begin_frame and begin_frame < keyframes_gl_.size ( ) ) {
				begin_frame_ = begin_frame;
				emit repaint ( );
			}
		}
		void SetEndFrame ( int end_frame ) {

			if ( 0 <= end_frame and end_frame < keyframes_gl_.size ( ) ) {
				end_frame_ = end_frame;
				emit repaint ( );
			}
		}

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

		Ui::BasicViewer* ui_;

		Camera camera_;
		int    mode_;

		glm::vec4 background_color_;
		float     degree_;
		float     scale_;
		int       density_step_;

		int begin_frame_;
		int end_frame_;

		bool render_grid_;
		bool render_inliers_;
		bool render_point_cloud_;
		bool render_trajectory_;
		bool render_answer_;

		bool top_view_;

		std::vector < KeyFrameGL > keyframes_gl_;
		std::vector < KeyFrameGL > keyframes_gl_for_inliers_;

		std::vector < CorrespondingPointsGL > corresponding_points_pair_gl_;
		std::vector < CorrespondingPointsGL > inliers_pair_gl_;

		std::vector < PointPairGL > estimation_point_pair_gl_;
		std::vector < PointPairGL > marker_point_pair_gl_;

		std::vector < std::vector < LineSegmentGL > > estimation_trajectory_gl_;
		std::vector < std::vector < LineSegmentGL > > marker_trajectory_gl_;

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
