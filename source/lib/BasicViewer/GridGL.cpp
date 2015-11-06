////
//// Created by LinKun on 10/5/15.
////
//
#include <iostream>
#include "BasicViewer/GridGL.h"

namespace NiS {

    void GridGL::Render() {

        using namespace std;
        GLint transformation_matrix_uniform_location = GL->glGetUniformLocation(shader_program_->programId(),
                                                                                "transformation_matrix");

        GL->glUniformMatrix4fv(transformation_matrix_uniform_location, 1, GL_FALSE,
                               &transformation_matrix_[0][0]);

        GL->glUseProgram(shader_program_->programId());
        GL->glBindVertexArray(vao_id_);
        GL->glDrawArrays(GL_LINES, 0, data_.size());
        GL->glBindVertexArray(0);
    }

    void GridGL::SetupData() {

        // std::cout << QOpenGLContext::currentContext ( )->objectName ( ).toStdString ( ) << std::endl;

        using namespace std;

        assert (!data_.empty());

        GL->glGenVertexArrays(1, &vao_id_);
        GL->glBindVertexArray(vao_id_);

        GL->glGenBuffers(1, &vbo_id_);
        GL->glBindBuffer(GL_ARRAY_BUFFER, vbo_id_);
        GL->glBufferData(GL_ARRAY_BUFFER,
                         sizeof(VertexGL) * data_.size(),
                         reinterpret_cast<float *> (data_.data()),
                         GL_STATIC_DRAW);

        GL->glEnableVertexAttribArray(0);
        GL->glEnableVertexAttribArray(1);

        GL->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexGL),
                                  nullptr);                                        // offset
        GL->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(VertexGL),
                                  (void *) (sizeof(VertexGL::Position)));         // offset

        GL->glBindBuffer(GL_ARRAY_BUFFER, 0);
        GL->glBindVertexArray(0);
//
//		ready_to_draw_ = true;

    }

    GridGL::GridGL(QOpenGLFunctions_4_1_Core *GL,
                   const float center_x,
                   const float center_y,
                   const float center_z,
                   const float range,
                   const float interval_width
    ) : PrimitiveGL(GL) {

        GenerateGridGL(center_x, center_y, center_z, range, interval_width);
    }

    GridGL::GridGL(QOpenGLFunctions_4_1_Core *GL) :
            PrimitiveGL(GL) { }

    void GridGL::GenerateGridGL(const float center_x, const float center_y, const float center_z, const float range,
                                const float interval_width) {

        std::vector<VertexGL> vertices;
        VertexGL::Color color(0.2f, 0.2f, 0.2f);

        // horizontal lines
        for (float x = center_x - range; x <= center_x + range; x += interval_width) {
            VertexGL begin, end;
            begin.position = VertexGL::Position(x, center_y, center_z - range);
            end.position = VertexGL::Position(x, center_y, center_z + range);

            // axis
            if (x == 0) {
                begin.color = VertexGL::Color(0.8f, 0.8f, 0.8f);
                end.color = VertexGL::Color(0.0f, 0.0f, 1.0f);
            }
            else {
                begin.color = color;
                end.color = color;
            }
            vertices.push_back(begin);
            vertices.push_back(end);
        }

        // vertical lines
        for (float z = center_z - range; z <= center_z + range; z += interval_width) {
            VertexGL begin, end;
            begin.position = VertexGL::Position(center_x - range, center_y, z);
            end.position = VertexGL::Position(center_x + range, center_y, z);

            // axis
            if (z == 0) {
                begin.color = VertexGL::Color(0.8f, 0.8f, 0.8f);
                end.color = VertexGL::Color(1.0f, 0.0f, 0.0f);
            }
            else {
                begin.color = color;
                end.color = color;
            }
            vertices.push_back(begin);
            vertices.push_back(end);
        }

        // Y axis
        VertexGL begin, end;
        begin.position = VertexGL::Position(0.0f, center_y, 0.0f);
        end.position = VertexGL::Position(0.0f, center_y + range, 0.0f);

        begin.color = VertexGL::Color(0.0f, 1.0f, 0.0f);
        end.color = VertexGL::Color(0.0f, 1.0f, 0.0f);

        vertices.push_back(begin);
        vertices.push_back(end);

        data_ = vertices;

    }

//
//    void GridGL2::Render() {
//
//        using namespace std;
//        GLint transformation_matrix_uniform_location = GL->glGetUniformLocation(shader_program_->programId(),
//                                                                                "transformation_matrix");
//
//        GL->glUniformMatrix4fv(transformation_matrix_uniform_location, 1, GL_FALSE,
//                               &transformation_matrix_[0][0]);
//
//        GL->glUseProgram(shader_program_->programId());
//        GL->glBindVertexArray(vao_id_);
//        GL->glDrawArrays(GL_LINES, 0, data_.size());
//        GL->glBindVertexArray(0);
//    }
//
//    void GridGL2::SetupData() {
//
//        using namespace std;
//
//        assert (!data_.empty());
//
//        GL->glGenVertexArrays(1, &vao_id_);
//        GL->glBindVertexArray(vao_id_);
//
//        GL->glGenBuffers(1, &vbo_id_);
//        GL->glBindBuffer(GL_ARRAY_BUFFER, vbo_id_);
//        GL->glBufferData(GL_ARRAY_BUFFER,
//                         sizeof(VertexGL) * data_.size(),
//                         reinterpret_cast<float *> (data_.data()),
//                         GL_STATIC_DRAW);
//
//        GL->glEnableVertexAttribArray(0);
//        GL->glEnableVertexAttribArray(1);
//
//        GL->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexGL),
//                                  nullptr);                                        // offset
//        GL->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(VertexGL),
//                                  (void *) (sizeof(VertexGL::Position)));         // offset
//
//        GL->glBindBuffer(GL_ARRAY_BUFFER, 0);
//        GL->glBindVertexArray(0);
//
//    }
//
//    GridGL2::GridGL2(const float center_x, const float center_y, const float center_z, const float range,
//                     const float interval_width, QOpenGLFunctions_4_1_Core *GL) : GL(GL) {
//
//        GenerateGridGL(center_x, center_y, center_z, range, interval_width);
//    }
//
//    GridGL2::~GridGL2() { }
//
//    void GridGL2::GenerateGridGL(const float center_x, const float center_y, const float center_z, const float range,
//                                 const float interval_width) {
//
//        std::vector<VertexGL> vertices;
//        VertexGL::Color color(0.3f, 0.3f, 0.3f);
//
//
//        // horizontal lines
//        for (float x = center_x - range; x <= center_x + range; x += interval_width) {
//            VertexGL begin, end;
//            begin.position = VertexGL::Position(x, center_y, center_z - range);
//            end.position = VertexGL::Position(x, center_y, center_z + range);
//            if (x == 0) {
//                begin.color = VertexGL::Color(0.0f, 0.0f, 1.0f);
//                end.color = VertexGL::Color(0.0f, 0.0f, 1.0f);
//            }
//            else {
//                begin.color = color;
//                end.color = color;
//            }
//            vertices.push_back(begin);
//            vertices.push_back(end);
//        }
//
//        // vertical lines
//        for (float z = center_z - range; z <= center_z + range; z += interval_width) {
//            VertexGL begin, end;
//            begin.position = VertexGL::Position(center_x - range, center_y, z);
//            end.position = VertexGL::Position(center_x + range, center_y, z);
//
//            if (z == 0) {
//                begin.color = VertexGL::Color(1.0f, 0.0f, 0.0f);
//                end.color = VertexGL::Color(1.0f, 0.0f, 0.0f);
//            }
//            else {
//                begin.color = color;
//                end.color = color;
//            }
//            vertices.push_back(begin);
//            vertices.push_back(end);
//        }
//
//        // Y axis
//        VertexGL begin, end;
//        begin.position = VertexGL::Position(0.0f, center_y, 0.0f);
//        end.position = VertexGL::Position(0.0f, center_y + range, 0.0f);
//
//        begin.color = VertexGL::Color(0.0f, 1.0f, 0.0f);
//        end.color = VertexGL::Color(0.0f, 1.0f, 0.0f);
//
//        vertices.push_back(begin);
//        vertices.push_back(end);
//
//        data_ = vertices;
//    }

};
