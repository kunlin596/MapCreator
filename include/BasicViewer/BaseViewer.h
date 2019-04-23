//
// Created by LinKun on 10/28/15.
//

#ifndef NIS_BASEVIEWER_H
#define NIS_BASEVIEWER_H

#include <QOpenGLFunctions_4_1_Core>
#include <QOpenGLWidget>

#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <QEvent>

#include "BasicViewer/Camera.h"

namespace MapCreator {

    class BaseViewer : public QOpenGLWidget {
    Q_OBJECT

    public:

        BaseViewer(QWidget *parent = 0);

        virtual void initializeGL() override = 0;

        virtual void paintGL() override = 0;

        virtual void resizeGL(int width, int height) override = 0;

        void mouseMoveEvent(QMouseEvent *e) override;

        void mousePressEvent(QMouseEvent *e) override;

        void keyPressEvent(QKeyEvent *e) override;

        void wheelEvent(QWheelEvent *e) override;

    signals:

        void Message(QString);

    protected:

        QOpenGLShaderProgram *SetupShaderProgram(const QString &vertex_shader_source_path,
                                                 const QString &fragment_shader_source_path,
                                                 QObject *parent = 0);

        void initialize();

        QOpenGLFunctions_4_1_Core *GL;

        Camera camera_;

        QOpenGLShaderProgram *shader_program_;

        QPoint last_mouse_position_;

        glm::vec4 background_color_;

        glm::mat4 model_rotation_matrix_;
        glm::mat4 model_translation_matrix_;

        float scale_;
        float degree_;

        glm::mat4 projection_matrix_;

    protected slots:

        void onResetCamera();

    };
}


#endif //NIS_BASEVIEWER_H
