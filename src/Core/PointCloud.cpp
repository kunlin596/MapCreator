//
// Created by LinKun on 9/12/15.
//

#include "Core/PointCloud.h"

namespace MapCreator {

    PointCloud::PointCloud() { }

    PointCloud::PointCloud(const ColorImage &color_image, const PointImage &point_image)
            : color_image_(color_image), point_image_(point_image) { }

    PointCloud::~PointCloud() { }

    PointCloud PointCloud::Clone() const {
        return PointCloud(color_image_.clone(), point_image_.clone());
    }

//    void PointCloud::Render() const
//    {
//        if (color_image_.empty() || point_image_.empty())
//        {
//            return;
//        }
//
//        glBegin(GL_POINTS);
//
//        for (int y = 0; y < point_image_.rows; ++y)
//        {
//            const cv::Vec3b* color = &color_image_(y, 0);
//            const cv::Vec3f* point = &point_image_(y, 0);
//
//            for (int x = 0; x < point_image_.cols; ++x)
//            {
//                glColor3ub((*color)(0), (*color)(1), (*color)(2));
//                glVertex3f((*point)(0), (*point)(1), (*point)(2));
//                ++color;
//                ++point;
//            }
//        }
//
//        glEnd();
//    }


}