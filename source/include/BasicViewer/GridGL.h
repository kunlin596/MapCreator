//
// Created by LinKun on 10/5/15.
//

#ifndef NIS_GRID_H
#define NIS_GRID_H

#include "PrimitiveGL.h"

namespace NiS {

    class GridGL : public PrimitiveGL {
    public:

        GridGL(QOpenGLFunctions_4_1_Core *GL, const float center_x, const float center_y, const float center_z,
               const float range,
               const float interval_width);

        GridGL(QOpenGLFunctions_4_1_Core *GL);

        void GenerateGridGL(const float center_x, const float center_y, const float center_z,
                            const float range, const float interval_width);

        void Render() override;

        void SetupData() override;
    };
//
//
//    class GridGL2 : QObject {
//
//    Q_OBJECT
//
//    public:
//
//        explicit GridGL2(const float center_x, const float center_y, const float center_z, const float range,
//                         const float interval_width, QOpenGLFunctions_4_1_Core *GL);
//
//        ~GridGL2();
//
//        void GenerateGridGL(const float center_x, const float center_y, const float center_z,
//                            const float range, const float interval_width);
//
//        void Render();
//
//        void SetupData();
//
//        inline void SetShaderProgram(
//                std::shared_ptr<QOpenGLShaderProgram> shader_program) { shader_program_ = shader_program; }
//
//        inline void SetTransformationMatrix(const glm::mat4 &m) { transformation_matrix_ = m; }
//
//        inline void SetVertexData(const PrimitiveGL::VertexData &data) { data_ = data; }
//
//        inline void SetModelMatrix(const glm::mat4 &model_matrix) {
//
//            model_matrix_ = model_matrix;
//            UpdateTransformationMatrix();
//        }
//
//        inline void SetViewMatrix(const glm::mat4 &view_matrix) {
//
//            view_matrix_ = view_matrix;
//            UpdateTransformationMatrix();
//        }
//
//        inline void SetProjectionMatrix(const glm::mat4 &projection_matrix) {
//
//            projection_matrix_ = projection_matrix;
//            UpdateTransformationMatrix();
//        }
//
//        inline void UpdateTransformationMatrix() {
//            transformation_matrix_ = projection_matrix_ * view_matrix_ * model_matrix_;
//        }
//
//    private:
//
//        glm::mat4 model_matrix_;
//        glm::mat4 view_matrix_;
//        glm::mat4 projection_matrix_;
//        glm::mat4 transformation_matrix_;
//
//        GLuint vbo_id_;
//        GLuint vao_id_;
//
//        std::shared_ptr<QOpenGLShaderProgram> shader_program_;
//        PrimitiveGL::VertexData data_;
//
//        QOpenGLFunctions_4_1_Core *GL;
//    };
}

#endif //NIS_GRID_H
