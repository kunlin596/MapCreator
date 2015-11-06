//
// Created by LinKun on 10/28/15.
//

#include "BasicViewer/PointPairGL.h"

namespace NiS {

    PointPairGL::PointPairGL(QOpenGLFunctions_4_1_Core *GL,
                             const std::pair<glm::vec3, glm::vec3> &point_pair) :
            PrimitiveGL(GL),
            point_pair_(point_pair) {

    }

    void PointPairGL::SetupData() {

        data_.clear();

        data_ = VertexData(3);

        data_[0].position = point_pair_.first;
        data_[1].position = glm::vec3(0.0f);
        data_[2].position = point_pair_.second;

        data_[0].color = glm::vec3(1.0f, 0.0f, 0.0f);
        data_[1].color = glm::vec3(0.5f, 0.5f, 0.5f);
        data_[2].color = glm::vec3(1.0f, 1.0f, 1.0f);

        GL->glGenVertexArrays(1, &vao_id_);
        GL->glGenBuffers(1, &vbo_id_);

        GL->glBindVertexArray(vao_id_);
        GL->glBindBuffer(GL_ARRAY_BUFFER, vbo_id_);
        GL->glBufferData(GL_ARRAY_BUFFER,
                         sizeof(VertexGL) * data_.size(),
                         reinterpret_cast<float *> (data_.data()),
                         GL_STATIC_DRAW);

        GL->glEnableVertexAttribArray(0);
        GL->glEnableVertexAttribArray(1);

        GL->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexGL), nullptr);
        GL->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(VertexGL),
                                  (void *) (sizeof(VertexGL::Position)));

        GL->glBindBuffer(GL_ARRAY_BUFFER, 0);
        GL->glBindVertexArray(0);

    }

    void PointPairGL::Render() {

        using namespace std;
        GLint transformation_matrix_uniform_location = GL->glGetUniformLocation(shader_program_->programId(),
                                                                                "transformation_matrix");

        GL->glUniformMatrix4fv(transformation_matrix_uniform_location, 1, GL_FALSE,
                               &transformation_matrix_[0][0]);

        GL->glUseProgram(shader_program_->programId());
        GL->glBindVertexArray(vao_id_);
        GL->glDrawArrays(GL_LINE_STRIP, 0, data_.size());
        GL->glBindVertexArray(0);
    }

}