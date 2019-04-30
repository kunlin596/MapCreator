////
//// Created by LinKun on 9/13/15.
////
//
//#ifndef MAPCREATOR_TREEHANDLER_H
//#define MAPCREATOR_TREEHANDLER_H
//
//
//#include <octomap/ScanGraph.h>
//#include <octomap/ColorOcTree.h>
//#include <opencv2/opencv.hpp>
//
//#include "SlamAlgorithms.h"
//
//#include "MyMath.h"
//
//namespace {
//
//    using namespace octomap;
//
//}
//
//
//namespace slam {
//
//    class TreeHandler {
//
//    public:
//
//        TreeHandler(double resolution = 0.3)
//                : color_tree_(ColorOcTree(resolution)) {
//        }
//
//        ~TreeHandler() { }
//
//        ColorOcTree GetTree() {
//            return color_tree_;
//        }
//
//        void AddNewKeyFrame(const cv::Mat_<cv::Vec3b> &color_image, const cv::Mat_<cv::Vec3f> &point_image);
//
//        void WriteFile(const std::string &name) {
//            color_tree_.write(name);
//        }
//
//    private:
//
//        ColorOcTree color_tree_;
//
//    };
//
//    void OutPutOctomapFile(const std::string &name,
//                           const slam::KeyFrames &keyframes,
//                           const double resolution = 0.3);
//
//
//    octomap::Pointcloud *Octomap_PointCloudMaker();
//
//    octomap::ScanGraph *Octomap_ScanGraphMaker(const std::string &name,
//                                               const slam::KeyFrames &keyframes);
//
//
//}
//
//#endif //MAPCREATOR_TREEHANDLER_H
