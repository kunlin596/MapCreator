//
// Created by LinKun on 10/28/15.
//

#include "BasicViewer/BaseViewer.h"

#include <Core/Utility.h>

#include <QResource>
#include <QMouseEvent>
#include <QEasingCurve>

namespace NiS {

    BaseViewer::BaseViewer(QWidget *parent) {

        projection_matrix_ = glm::perspective(glm::radians(45.0f),                           // radian
                                              (float) (this->width()) / this->height(), // aspect ratio
                                              0.001f,                                           // near z
                                              200.0f);                                          // far z
    }

    QOpenGLShaderProgram *BaseViewer::SetupShaderProgram(const QString &vertex_shader_source_path,
                                                         const QString &fragment_shader_source_path,
                                                         QObject *parent) {

        QResource vertex_shader_resource(vertex_shader_source_path);
        QResource fragment_shader_resource(fragment_shader_source_path);

        auto vertex_shader_code = NiS::ConvertConstCStrToStdString(vertex_shader_resource.data(),
                                                                   vertex_shader_resource.size());
        auto fragment_shader_code = NiS::ConvertConstCStrToStdString(fragment_shader_resource.data(),
                                                                     fragment_shader_resource.size());

        QOpenGLShader vertex_shader(QOpenGLShader::Vertex, 0);
        QOpenGLShader fragment_shader(QOpenGLShader::Fragment, 0);

        vertex_shader.compileSourceCode(vertex_shader_code.c_str());
        fragment_shader.compileSourceCode(fragment_shader_code.c_str());

        assert (vertex_shader.isCompiled() and fragment_shader.isCompiled());

        QOpenGLShaderProgram *program = new QOpenGLShaderProgram(parent);

        program->addShader(&vertex_shader);
        program->addShader(&fragment_shader);

        program->link();

        assert (program->isLinked());

        return program;
    }

    void BaseViewer::initialize() {

        GL->initializeOpenGLFunctions();

        GL->glViewport(0, 0, width(), height());
        GL->glEnable(GL_DEPTH_TEST);
        GL->glEnable(GL_LINE_SMOOTH);
        GL->glEnable(GL_PROGRAM_POINT_SIZE);
        GL->glEnable(GL_CULL_FACE);
        background_color_ = glm::vec4(0.1f, 0.1f, 0.1f, 1.0f);
    }

    void BaseViewer::onResetCamera() {

        camera_.Reset();
        emit repaint();
    }

    void BaseViewer::mouseMoveEvent(QMouseEvent *e) {

        float dx = (e->x() - last_mouse_position_.x()) * 0.001f;
        float dy = (e->y() - last_mouse_position_.y()) * 0.001f;

        if ((e->modifiers() & Qt::ShiftModifier)) {

            // rotation around x
            glm::mat4 model_rotation_matrix_x_axis = glm::rotate(glm::mat4(), glm::radians(dy),
                                                                 camera_.GetHorizontalStrifeMatrix());
            model_rotation_matrix_ *= /*model_rotation_matrix_y_axis **/ model_rotation_matrix_x_axis;

        }

        else {
            camera_.MouseUpdate(glm::vec2(dx, dy));
        }

        emit repaint();
    }

    void BaseViewer::mousePressEvent(QMouseEvent *e) {

        last_mouse_position_ = e->pos();

    }

    void BaseViewer::keyPressEvent(QKeyEvent *e) {
        // Camera movement
        const float distance = 0.3f;

        QEasingCurve curve(QEasingCurve::InOutQuad);

        for (auto t = 0.0f; t < distance; t += 0.02f) {

            float easing_value = static_cast<float>(curve.valueForProgress(t));

            switch (e->key()) {
                case Qt::Key_W:
                    camera_.KeyUpdate(Camera::Move::StrafeForward, easing_value);
                    break;
                case Qt::Key_S:
                    camera_.KeyUpdate(Camera::Move::StrafeBackward, easing_value);
                    break;
                case Qt::Key_A:
                    camera_.KeyUpdate(Camera::Move::StrafeLeft, easing_value);
                    break;
                case Qt::Key_D:
                    camera_.KeyUpdate(Camera::Move::StrafeRight, easing_value);
                    break;
                case Qt::Key_Q:
                    camera_.KeyUpdate(Camera::Move::StrafeUp, easing_value);
                    break;
                case Qt::Key_E:
                    camera_.KeyUpdate(Camera::Move::StrafeDown, easing_value);
                    break;
                case Qt::Key_R:
                    camera_.Reset();
                    break;
                case Qt::Key_O:
                    scale_ *= (1.0f + easing_value);
                    break;
                case Qt::Key_P:
                    scale_ /= (1.0f + easing_value);
                    break;
                default:
                    break;
            }

            emit repaint();
        }
    }

    void BaseViewer::wheelEvent(QWheelEvent *e) {

        float distance = 0.001f * e->delta();

        camera_.KeyUpdate(Camera::Move::StrafeForward, distance);

        emit repaint();
    }


}