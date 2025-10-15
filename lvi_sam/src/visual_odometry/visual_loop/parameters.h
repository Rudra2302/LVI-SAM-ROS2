#pragma once
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <eigen3/Eigen/Dense>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <cassert>

#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"
#include "ThirdParty/DBoW/TemplatedDatabase.h"
#include "ThirdParty/DBoW/TemplatedVocabulary.h"

#include "../visual_feature/camera_models/CameraFactory.h"
#include "../visual_feature/camera_models/CataCamera.h"
#include "../visual_feature/camera_models/PinholeCamera.h"

using namespace std;
using ament_index_cpp::get_package_share_directory;

extern camodocal::CameraPtr m_camera;

extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;

extern string PROJECT_NAME;
extern string IMAGE_TOPIC;

extern int DEBUG_IMAGE;
extern int LOOP_CLOSURE;
extern double MATCH_IMAGE_SCALE;

extern rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_img;
extern rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_match_msg;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_key_pose;






class BriefExtractor
{
public:

    DVision::BRIEF m_brief;

    virtual void operator()(const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<DVision::BRIEF::bitset> &descriptors) const
    {
        m_brief.compute(im, keys, descriptors);
    }

    BriefExtractor(){};

    BriefExtractor(std::shared_ptr<rclcpp::Node> n, const std::string &pattern_file)
    {
        // The DVision::BRIEF extractor computes a random pattern by default when
        // the object is created.
        // We load the pattern that we used to build the vocabulary, to make
        // the descriptors compatible with the predefined vocabulary

        // loads the pattern
        // cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
        // if(!fs.isOpened()) throw string("Could not open file ") + pattern_file;

        vector<long int> x1_long, y1_long, x2_long, y2_long;
        n->declare_parameter("x1", std::vector<long int>());
        n->get_parameter("x1", x1_long);
        n->declare_parameter("x2", std::vector<long int>());
        n->get_parameter("x2", x2_long);
        n->declare_parameter("y1", std::vector<long int>());
        n->get_parameter("y1", y1_long);
        n->declare_parameter("y2", std::vector<long int>());
        n->get_parameter("y2", y2_long);
        // fs["x1"] >> x1;
        // fs["x2"] >> x2;
        // fs["y1"] >> y1;
        // fs["y2"] >> y2;

        vector<int> x1, y1, x2, y2;
        x1.assign(x1_long.begin(), x1_long.end());
        y1.assign(y1_long.begin(), y1_long.end());
        x2.assign(x2_long.begin(), x2_long.end());
        y2.assign(y2_long.begin(), y2_long.end());

        m_brief.importPairs(x1, y1, x2, y2);
    }
};

extern BriefExtractor briefExtractor;