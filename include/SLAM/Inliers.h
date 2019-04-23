//
// Created by LinKun on 10/7/15.
//

#ifndef NIS_INLIERS_H
#define NIS_INLIERS_H

#include <BasicViewer/PrimitiveGL.h>
#include "SLAM/CommonDefinitions.h"

namespace MapCreator {

    class Inliers : public PrimitiveGL {
    public:

        Inliers(QOpenGLFunctions_4_1_Core *GL, const Points &points1, const Points &points2);

        ~Inliers();

        void Render() override;

        void SetupData() override;

    private:

        void DataFormatConversionHelper(const Points &points1, const Points &points2);

    };
}


#endif //NIS_INLIERS_H
