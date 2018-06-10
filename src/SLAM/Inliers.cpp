//
// Created by LinKun on 10/7/15.
//

#include "SLAM/Inliers.h"

namespace NiS {

    Inliers::Inliers(QOpenGLFunctions_4_1_Core *GL, const Points &points1, const Points &points2) :
            PrimitiveGL(GL) {
        vbo_id_ = 0;
        vao_id_ = 0;

        DataFormatConversionHelper(points1, points2);
    }

    Inliers::~Inliers() {
    }

    void Inliers::Render() {

        assert(shader_program_->isLinked());

        GLint transformation_matrix_uniform_location = GL->glGetUniformLocation(shader_program_->programId(),
                                                                                "transformation_matrix");
        GL->glUniformMatrix4fv(transformation_matrix_uniform_location, 1, GL_FALSE,
                               &transformation_matrix_[0][0]);

        shader_program_->bind();
        GL->glBindVertexArray(vao_id_);
        GL->glDrawArrays(GL_LINES, 0, data_.size());
        GL->glBindVertexArray(0);
    }

    void Inliers::SetupData() {

        assert (!data_.empty());

        if (vao_id_ == 0) GL->glGenVertexArrays(1, &vao_id_);
        if (vbo_id_ == 0) GL->glGenBuffers(1, &vbo_id_);

        GL->glBindVertexArray(vao_id_);
        GL->glBindBuffer(GL_ARRAY_BUFFER, vbo_id_);
        GL->glBufferData(GL_ARRAY_BUFFER,
                         sizeof(VertexGL) * data_.size(),
                         reinterpret_cast<float *> (data_.data()),
                         GL_DYNAMIC_DRAW);

        GL->glEnableVertexAttribArray(0);
        GL->glEnableVertexAttribArray(1);

        GL->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexGL), nullptr);
        GL->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(VertexGL),
                                  (void *) (sizeof(VertexGL::Position)));

        GL->glBindBuffer(GL_ARRAY_BUFFER, 0);
        GL->glBindVertexArray(0);

    }

    void Inliers::DataFormatConversionHelper(const Points &points1, const Points &points2) {

        data_.clear();
        VertexGL::Color begin_color(1.0f, 0.0f, 0.0f);
        VertexGL::Color end_color(0.0f, 0.0f, 1.0f);

        for (auto i = 0; i < points1.size(); ++i) {
            VertexGL begin, end;

            begin.position = glm::vec3(points1[i].x, points1[i].y, points1[i].z);
            begin.color = begin_color;
            data_.push_back(begin);

            end.position = glm::vec3(points2[i].x, points2[i].y, points2[i].z);
            end.color = end_color;
            data_.push_back(end);
        }
    }

}