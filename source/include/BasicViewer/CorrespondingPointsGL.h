//
// Created by LinKun on 10/15/15.
//

#ifndef NIS_CORRESPOINDINGPOINTSGL_H
#define NIS_CORRESPOINDINGPOINTSGL_H


#include "BasicViewer/PrimitiveGL.h"
#include <SLAM/CommonDefinitions.h>

namespace NiS {

    class CorrespondingPointsGL : public PrimitiveGL {
    public:

        CorrespondingPointsGL() { }

        CorrespondingPointsGL(QOpenGLFunctions_4_1_Core *GL,
                              const CorrespondingPointsPair &corresponding_points_pair);

        ~CorrespondingPointsGL();

        void SetupData() override;

        void Render() override;

    private:

        CorrespondingPointsPair corresponding_points_pair_;

        inline void ConvertData() {

            data_.clear();
            VertexGL::Color begin_color(1.0f, 0.0f, 0.0f);
            VertexGL::Color end_color(0.0f, 0.0f, 1.0f);

            const Points &points1 = corresponding_points_pair_.first;
            const Points &points2 = corresponding_points_pair_.second;

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

    };

}
#endif //NIS_CORRESPOINDINGPOINTSGL_H
