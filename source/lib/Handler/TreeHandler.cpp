////
//// Created by LinKun on 9/13/15.
////
//
//#include <octomap/math/Pose6D.h>
//
//#include "TreeHandler.h"
//#include "MyMath.h"
//#include <cmath>
//#include <vector>
//#include <iostream>
//#include <numeric>
//
//namespace {
//
//    using namespace std;
//
//    cv::Mat_<cv::Vec3f> TransformPointImage(const cv::Mat_<cv::Vec3f> &point_image, const cv::Matx44f &m) {
//
//        using namespace std;
//
//        cv::Mat_<cv::Vec3f> cvt_point_image(point_image.rows, point_image.cols);
//
//        for (auto row = 0; row < point_image.rows; ++row) {
//            for (auto col = 0; col < point_image.cols; ++col) {
//                const cv::Vec3f &vec = point_image(row, col);
//
//                cv::Matx33f r;
//                cv::Vec3f t;
//
//                r << m;
//                t << m;
//
//                cv::Vec3f res_vec = vec * r + t;
//                cvt_point_image(row, col) = res_vec;
//
//            }
//        }
//
//        return cvt_point_image;
//    }
//
//    octomath::Pose6D Octomap_ConvertTransformationMatrix(const cv::Matx44f &m) {
//
//        using namespace std;
//
//        cv::Vec3f t;
//        t << m;
//        cv::Matx33f r;
//        r << m;
//
//        double yaw, pitch, roll;
//
//        yaw = atan(r(1, 0) / r(0, 0));
//        pitch = atan(-r(2, 0) / sqrt(r(2, 1) * r(2, 1) + r(2, 2) * r(2, 2)));
//        roll = atan(r(2, 1) / r(2, 2));
//
//        return octomath::Pose6D(t(0), t(1), t(2), roll, pitch, yaw);
//    }
//}
//
//namespace slam {
//
//    void OutPutOctomapFile(const std::string &name,
//                           const slam::KeyFrames &keyframes,
//                           const double resolution) {
//
//        using namespace std;
//
//        PrintMessage("Creating ColorOcTree...", 1);
//        TreeHandler t(resolution);
//        PrintMessage("Craeted tree.", 2);
//
//        for (auto i = 0; i < keyframes.size(); ++i) {
//
//            PrintMessage("Adding <" + to_string(i) + "> -th key frame.", 1);
//
//            cv::Mat_<cv::Vec3f> cvt_point_image = TransformPointImage(keyframes[i].point_image, keyframes[i].matrix);
//
//            t.AddNewKeyFrame(keyframes[i].color_image, cvt_point_image);
//
//            PrintMessage("Added <" + to_string(i) + "> -th frame to tree", 2);
//        }
//
//        t.WriteFile(name);
//        PrintMessage("Serialized tree to file " + name, 2);
//    }
//
//
//    void TreeHandler::AddNewKeyFrame(const cv::Mat_<cv::Vec3b> &color_image, const cv::Mat_<cv::Vec3f> &point_image) {
//
//        if (color_image.empty() || point_image.empty())
//            return;
//
//        const auto rows = color_image.rows;
//        const auto cols = color_image.cols;
//
//        static int count = 0;
//
//        for (auto row = 0; row < rows; ++row) {
//            for (auto col = 0; col < cols; ++col) {
//
//                float x = point_image(row, col)[0];
//                float y = point_image(row, col)[1];
//                float z = point_image(row, col)[2];
//
//                if (std::isnan(x) or std::isnan(y) or std::isnan(z)) {
//                    // or x > 10 or y > 10 or z > 10) {
//                    continue;
//                }
//
//                cv::Matx33f xz_rotation = cv::Matx33f::zeros();
//                xz_rotation(0, 2) = xz_rotation(1, 1) = xz_rotation(2, 0) = -1;
//
//                cv::Matx33f yz_rotation = cv::Matx33f::zeros();
//                yz_rotation(0, 0) = 1;
//                yz_rotation(1, 2) = -1;
//                yz_rotation(2, 1) = 1;
//
//                cv::Vec3f v1 = point_image(row, col);
//
//                cv::Vec3f v = v1 * xz_rotation * yz_rotation;
//
//                point3d p(v(0) * 50, v(1) * 50, v(2) * 50);
//
//                // cout << row << ", " << col << ", " << p << endl;
//                ColorOcTreeNode *n = color_tree_.updateNode(p, true); // integrate 'occupied' measurement
//                // ColorOcTreeNode *nn = temp_t.updateNode(p, true);
//
//                n->setColor(color_image.at<cv::Vec3b>(row, col)[0],
//                            color_image.at<cv::Vec3b>(row, col)[1],
//                            color_image.at<cv::Vec3b>(row, col)[2]);
//
////                nn->setColor(color_image.at<cv::Vec3b>(row, col)[0],
////                            color_image.at<cv::Vec3b>(row, col)[1],
////                            color_image.at<cv::Vec3b>(row, col)[2]);
//
//
//
//
//            }
//        }
//
//    }
//
//
//    octomap::Pointcloud *Octomap_PointCloudMaker(const slam::KeyFrame &keyframe) {
//
//        using Pose = octomath::Pose6D;
//
//        octomap::Pointcloud *point_cloud = new octomap::Pointcloud;
//
//        for (auto row = 0; row < keyframe.point_image.rows; ++row) {
//            for (auto col = 0; col < keyframe.point_image.cols; ++col) {
//                auto const &vec = keyframe.point_image.at<cv::Vec3f>(row, col);
//                if (vec(0) != std::numeric_limits<float>::quiet_NaN())
//                    point_cloud->push_back(vec(0), vec(1), vec(2));
//            }
//        }
//
//
//        return point_cloud;
//    }
//
//
//    octomap::ScanGraph *Octomap_ScanGraphMaker(const std::string &name,
//                                               const slam::KeyFrames &keyframes) {
//
//        using Graph = octomap::ScanGraph;
//        using Pose = octomath::Pose6D;
//        using Node = octomap::ScanNode;
//        using Edge = octomap::ScanEdge;
//
//        if (keyframes.empty())
//            return new Graph();
//
//
//        cout << "Number: " << keyframes.size() << endl;
//
//        Graph *graph = new Graph();
//
//        octomap::Pointcloud *first_pointcloud = Octomap_PointCloudMaker(keyframes[0]);
//        Pose initial_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//
//        Node *first_node = new Node(first_pointcloud, initial_pose, 0);
//
//        for (auto i = 1; i < keyframes.size(); ++i) {
//
//            octomap::Pointcloud *second_pointcloud = Octomap_PointCloudMaker(keyframes[i]);
//            Node *second_node = new Node(second_pointcloud, initial_pose, i);
//
//            Pose constraint = Octomap_ConvertTransformationMatrix(keyframes[i].matrix);
//
//            graph->addNode(first_pointcloud, initial_pose);
//            graph->addNode(second_pointcloud, initial_pose);
//            graph->addEdge(second_node, first_node, constraint);
//
//            first_node = second_node;
//        }
//
//        graph->transformScans();
//        graph->writeBinary(name);
//
//        return graph;
//    }
//
//}
