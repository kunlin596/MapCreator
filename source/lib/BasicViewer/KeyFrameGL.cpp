//
// Created by LinKun on 10/11/15.
//

#include "BasicViewer/KeyFrameGL.h"

#include <limits>

namespace NiS {


    KeyFrameGL::KeyFrameGL(QOpenGLFunctions_4_1_Core *GL,
                           const KeyFrame &keyframe,
                           const int &point_cloud_density_step) :
            PrimitiveGL(GL),
            keyframe_(keyframe),
            point_cloud_density_step_(point_cloud_density_step) {
    }

    void KeyFrameGL::Render() {

        using namespace std;
        GLint transformation_matrix_uniform_location = GL->glGetUniformLocation(shader_program_->programId(),
                                                                                "transformation_matrix");

        GL->glUniformMatrix4fv(transformation_matrix_uniform_location, 1, GL_FALSE,
                               &transformation_matrix_[0][0]);

        GL->glUseProgram(shader_program_->programId());
        GL->glBindVertexArray(vao_id_);
        GL->glDrawArrays(GL_POINTS, 0, data_.size());
        GL->glBindVertexArray(0);
    }

    void KeyFrameGL::SetupData() {

        // use a local buffer to send data to GPU and then immediately destroy it
        const ColorImage &color_image = keyframe_.GetColorImage();
        const PointImage &point_image = keyframe_.GetPointImage();

        const auto rows = color_image.rows;
        const auto cols = point_image.cols;

        data_.clear();

        for (auto row = 0; row < rows; row += point_cloud_density_step_) {
            for (auto col = 0; col < cols; col += point_cloud_density_step_) {

                VertexGL vertex;

                vertex.position = glm::vec3(point_image.at<cv::Vec3f>(row, col)[0],
                                            point_image.at<cv::Vec3f>(row, col)[1],
                                            point_image.at<cv::Vec3f>(row, col)[2]);

                vertex.color = glm::vec3(color_image.at<cv::Vec3b>(row, col)[0] / 255.0f,
                                         color_image.at<cv::Vec3b>(row, col)[1] / 255.0f,
                                         color_image.at<cv::Vec3b>(row, col)[2] / 255.0f);

                data_.push_back(vertex);
            }
        }

        assert (!data_.empty());

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
        gpu_data_is_new_ = true;
    }
}
