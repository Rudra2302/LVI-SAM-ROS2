#include "parameters.h"

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/header.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

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

using namespace std;
using std_msgs::msg::Header;

typedef pcl::PointXYZI PointType;

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::string POINT_CLOUD_TOPIC;
std::string PROJECT_NAME;

std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

double L_C_TX;
double L_C_TY;
double L_C_TZ;
double L_C_RX;
double L_C_RY;
double L_C_RZ;

int USE_LIDAR;
int LIDAR_SKIP;


void readParameters(std::shared_ptr<rclcpp::Node> n)
{
    std::string config_file;
    n->declare_parameter("vins_config_file", "");
    n->get_parameter("vins_config_file", config_file);
    // RCLCPP_INFO(n->get_logger(), "config path: %s", config_file.c_str());
    // cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    // if(!fsSettings.isOpened())
    // if(config_file == "")
    // {
    //     std::cerr << "ERROR: Wrong path to settings" << std::endl;
    //     rclcpp::shutdown();
    //     return;
    // }

    // project name
    // fsSettings["project_name"] >> PROJECT_NAME;
    n->declare_parameter("PROJECT_NAME", "");
    n->get_parameter("PROJECT_NAME", PROJECT_NAME);
    std::string pkg_path = get_package_share_directory(PROJECT_NAME);

    // sensor topics
    n->declare_parameter("image_topic", "");
    n->get_parameter("image_topic", IMAGE_TOPIC);
    n->declare_parameter("imu_topic", "");
    n->get_parameter("imu_topic", IMU_TOPIC);
    n->declare_parameter("point_cloud_topic", "");
    n->get_parameter("point_cloud_topic", POINT_CLOUD_TOPIC);
    // fsSettings["image_topic"]       >> IMAGE_TOPIC;
    // fsSettings["imu_topic"]         >> IMU_TOPIC;
    // fsSettings["point_cloud_topic"] >> POINT_CLOUD_TOPIC;

    // lidar configurations
    n->declare_parameter("use_lidar", 1);
    n->get_parameter("use_lidar", USE_LIDAR);
    n->declare_parameter("lidar_skip", 1);
    n->get_parameter("lidar_skip", LIDAR_SKIP);
    // fsSettings["use_lidar"] >> USE_LIDAR;
    // fsSettings["lidar_skip"] >> LIDAR_SKIP;

    // feature and image settings
    n->declare_parameter("max_cnt", 1);
    n->get_parameter("max_cnt", MAX_CNT);
    n->declare_parameter("min_dist", 1);
    n->get_parameter("min_dist", MIN_DIST);
    n->declare_parameter("image_height", 1);
    n->get_parameter("image_height", ROW);
    n->declare_parameter("image_width", 1);
    n->get_parameter("image_width", COL);
    n->declare_parameter("freq", 1);
    n->get_parameter("freq", FREQ);
    n->declare_parameter("F_threshold", 1.0);
    n->get_parameter("F_threshold", F_THRESHOLD);
    n->declare_parameter("show_track", 1);
    n->get_parameter("show_track", SHOW_TRACK);
    n->declare_parameter("equalize", 1);
    n->get_parameter("equalize", EQUALIZE);
    // MAX_CNT = fsSettings["max_cnt"];
    // MIN_DIST = fsSettings["min_dist"];
    // ROW = fsSettings["image_height"];
    // COL = fsSettings["image_width"];
    // FREQ = fsSettings["freq"];
    // F_THRESHOLD = fsSettings["F_threshold"];
    // SHOW_TRACK = fsSettings["show_track"];
    // EQUALIZE = fsSettings["equalize"];

    n->declare_parameter("lidar_to_cam_tx", 0.0);
    n->get_parameter("lidar_to_cam_tx", L_C_TX);
    n->declare_parameter("lidar_to_cam_ty", 0.0);
    n->get_parameter("lidar_to_cam_ty", L_C_TY);
    n->declare_parameter("lidar_to_cam_tz", 0.0);
    n->get_parameter("lidar_to_cam_tz", L_C_TZ);
    n->declare_parameter("lidar_to_cam_rx", 0.0);
    n->get_parameter("lidar_to_cam_rx", L_C_RX);
    n->declare_parameter("lidar_to_cam_ry", 0.0);
    n->get_parameter("lidar_to_cam_ry", L_C_RY);
    n->declare_parameter("lidar_to_cam_rz", 0.0);
    n->get_parameter("lidar_to_cam_rz", L_C_RZ);

    // L_C_TX = fsSettings["lidar_to_cam_tx"];
    // L_C_TY = fsSettings["lidar_to_cam_ty"];
    // L_C_TZ = fsSettings["lidar_to_cam_tz"];
    // L_C_RX = fsSettings["lidar_to_cam_rx"];
    // L_C_RY = fsSettings["lidar_to_cam_ry"];
    // L_C_RZ = fsSettings["lidar_to_cam_rz"];

    // fisheye mask
    n->declare_parameter("fisheye", 1);
    n->get_parameter("fisheye", FISHEYE);
    // FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
    {
        std::string mask_name;
        n->declare_parameter("fisheye_mask", "");
        n->get_parameter("fisheye_mask", mask_name);
        // fsSettings["fisheye_mask"] >> mask_name;
        FISHEYE_MASK = pkg_path + mask_name;
    }

    // camera config
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    // fsSettings.release();
    usleep(100);
}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
{
    if (thisPub->get_subscription_count() == 0)
        return;
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    thisPub->publish(tempCloud); 
}