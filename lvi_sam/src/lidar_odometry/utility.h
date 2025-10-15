#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

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

class ParamServer : public rclcpp::Node
{
public:
    std::string PROJECT_NAME;
    std::string robot_id;

    std::string pointCloudTopic;
    std::string imuTopic;
    std::string odomTopic;
    std::string gpsTopic;
    std::string imageTopic;

    // Parameters
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    bool savePCD;
    std::string savePCDDirectory;

    int N_SCAN;
    int Horizon_SCAN;
    std::string timeField;
    std::string pcap_file_;
    std::string mcap_file_;
    std::string image_file1;
    std::string image_file2;
    int downsampleRate;

    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;

    std::vector<double> extRotV, extRPYV, extTransV;
    Eigen::Matrix3d extRot, extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    float edgeThreshold, surfThreshold;
    int edgeFeatureMinValidNum, surfFeatureMinValidNum;

    float odometrySurfLeafSize, mappingCornerLeafSize, mappingSurfLeafSize;
    float z_tollerance, rotation_tollerance;

    int numberOfCores;
    double mappingProcessInterval;

    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    bool loopClosureEnableFlag;
    int surroundingKeyframeSize;
    float historyKeyframeSearchRadius, historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer() : Node("param_server")
    {
        this->declare_parameter("pcap_file", "");
        this->get_parameter("pcap_file", pcap_file_);
        this->declare_parameter("mcap_file", "");
        this->get_parameter("mcap_file", mcap_file_);
        this->declare_parameter("image_file1", "");
        this->get_parameter("image_file1", image_file1);
        this->declare_parameter("image_file2", "");
        this->get_parameter("image_file2", image_file2);
        RCLCPP_INFO(this->get_logger(), "pcap_file param: %s", pcap_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "mcap_file param: %s", mcap_file_.c_str());
        this->declare_parameter("PROJECT_NAME", "sam");
        this->get_parameter("PROJECT_NAME", PROJECT_NAME);
        this->declare_parameter("robot_id", "roboat");
        this->get_parameter("robot_id", robot_id);

        this->declare_parameter(PROJECT_NAME + ".pointCloudTopic", "points_raw");
        this->declare_parameter(PROJECT_NAME + ".imuTopic", "imu_correct");
        this->declare_parameter(PROJECT_NAME + ".odomTopic", "odometry/imu");
        this->declare_parameter(PROJECT_NAME + ".gpsTopic", "odometry/gps");
        this->declare_parameter(PROJECT_NAME + ".image_topic", "/camera/image_raw");
        this->get_parameter(PROJECT_NAME + ".pointCloudTopic", pointCloudTopic);
        this->get_parameter(PROJECT_NAME + ".imuTopic", imuTopic);
        this->get_parameter(PROJECT_NAME + ".odomTopic", odomTopic);
        this->get_parameter(PROJECT_NAME + ".gpsTopic", gpsTopic);
        this->get_parameter(PROJECT_NAME + ".image_topic", imageTopic);

        this->declare_parameter(PROJECT_NAME + ".useImuHeadingInitialization", false);
        this->declare_parameter(PROJECT_NAME + ".useGpsElevation", false);
        this->declare_parameter(PROJECT_NAME + ".gpsCovThreshold", 2.0);
        this->declare_parameter(PROJECT_NAME + ".poseCovThreshold", 25.0);
        this->get_parameter(PROJECT_NAME + ".useImuHeadingInitialization", useImuHeadingInitialization);
        this->get_parameter(PROJECT_NAME + ".useGpsElevation", useGpsElevation);
        this->get_parameter(PROJECT_NAME + ".gpsCovThreshold", gpsCovThreshold);
        this->get_parameter(PROJECT_NAME + ".poseCovThreshold", poseCovThreshold);

        this->declare_parameter(PROJECT_NAME + ".savePCD", false);
        this->declare_parameter(PROJECT_NAME + ".savePCDDirectory", "/tmp/loam/");
        this->get_parameter(PROJECT_NAME + ".savePCD", savePCD);
        this->get_parameter(PROJECT_NAME + ".savePCDDirectory", savePCDDirectory);

        this->declare_parameter(PROJECT_NAME + ".N_SCAN", 10);
        this->declare_parameter(PROJECT_NAME + ".Horizon_SCAN", 1800);
        this->declare_parameter(PROJECT_NAME + ".timeField", "time");
        this->declare_parameter(PROJECT_NAME + ".downsampleRate", 1);
        this->get_parameter(PROJECT_NAME + ".N_SCAN", N_SCAN);
        this->get_parameter(PROJECT_NAME + ".Horizon_SCAN", Horizon_SCAN);
        this->get_parameter(PROJECT_NAME + ".timeField", timeField);
        this->get_parameter(PROJECT_NAME + ".downsampleRate", downsampleRate);

        this->declare_parameter(PROJECT_NAME + ".imuAccNoise", 0.01f);
        this->get_parameter(PROJECT_NAME + ".imuAccNoise", imuAccNoise);

        this->declare_parameter(PROJECT_NAME + ".imuGyrNoise", 0.001f);
        this->get_parameter(PROJECT_NAME + ".imuGyrNoise", imuGyrNoise);

        this->declare_parameter(PROJECT_NAME + ".imuAccBiasN", 0.0002f);
        this->get_parameter(PROJECT_NAME + ".imuAccBiasN", imuAccBiasN);

        this->declare_parameter(PROJECT_NAME + ".imuGyrBiasN", 0.00003f);
        this->get_parameter(PROJECT_NAME + ".imuGyrBiasN", imuGyrBiasN);

        this->declare_parameter(PROJECT_NAME + ".imuGravity", 9.80511f);
        this->get_parameter(PROJECT_NAME + ".imuGravity", imuGravity);

        this->declare_parameter(PROJECT_NAME + ".extrinsicRot", std::vector<double>{});
        this->declare_parameter(PROJECT_NAME + ".extrinsicRPY", std::vector<double>{});
        this->declare_parameter(PROJECT_NAME + ".extrinsicTrans", std::vector<double>{});

        this->get_parameter(PROJECT_NAME + ".extrinsicRot", extRotV);
        this->get_parameter(PROJECT_NAME + ".extrinsicRPY", extRPYV);
        this->get_parameter(PROJECT_NAME + ".extrinsicTrans", extTransV);

        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        this->declare_parameter(PROJECT_NAME + ".edgeThreshold", 0.1f);
        this->get_parameter(PROJECT_NAME + ".edgeThreshold", edgeThreshold);

        this->declare_parameter(PROJECT_NAME + ".surfThreshold", 0.1f);
        this->get_parameter(PROJECT_NAME + ".surfThreshold", surfThreshold);

        this->declare_parameter(PROJECT_NAME + ".odometrySurfLeafSize", 0.2f);
        this->get_parameter(PROJECT_NAME + ".odometrySurfLeafSize", odometrySurfLeafSize);

        this->declare_parameter(PROJECT_NAME + ".mappingCornerLeafSize", 0.2f);
        this->get_parameter(PROJECT_NAME + ".mappingCornerLeafSize", mappingCornerLeafSize);

        this->declare_parameter(PROJECT_NAME + ".mappingSurfLeafSize", 0.2f);
        this->get_parameter(PROJECT_NAME + ".mappingSurfLeafSize", mappingSurfLeafSize);

        this->declare_parameter(PROJECT_NAME + ".z_tollerance", FLT_MAX);
        this->get_parameter(PROJECT_NAME + ".z_tollerance", z_tollerance);

        this->declare_parameter(PROJECT_NAME + ".rotation_tollerance", FLT_MAX);
        this->get_parameter(PROJECT_NAME + ".rotation_tollerance", rotation_tollerance);

        this->declare_parameter(PROJECT_NAME + ".surroundingkeyframeAddingDistThreshold", 1.0f);
        this->get_parameter(PROJECT_NAME + ".surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold);

        this->declare_parameter(PROJECT_NAME + ".surroundingkeyframeAddingAngleThreshold", 0.2f);
        this->get_parameter(PROJECT_NAME + ".surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold);

        this->declare_parameter(PROJECT_NAME + ".surroundingKeyframeDensity", 1.0f);
        this->get_parameter(PROJECT_NAME + ".surroundingKeyframeDensity", surroundingKeyframeDensity);

        this->declare_parameter(PROJECT_NAME + ".surroundingKeyframeSearchRadius", 50.0f);
        this->get_parameter(PROJECT_NAME + ".surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius);

        this->declare_parameter(PROJECT_NAME + ".historyKeyframeSearchRadius", 10.0f);
        this->get_parameter(PROJECT_NAME + ".historyKeyframeSearchRadius", historyKeyframeSearchRadius);

        this->declare_parameter(PROJECT_NAME + ".historyKeyframeSearchTimeDiff", 30.0f);
        this->get_parameter(PROJECT_NAME + ".historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff);

        this->declare_parameter(PROJECT_NAME + ".historyKeyframeFitnessScore", 0.3f);
        this->get_parameter(PROJECT_NAME + ".historyKeyframeFitnessScore", historyKeyframeFitnessScore);

        this->declare_parameter(PROJECT_NAME + ".globalMapVisualizationSearchRadius", 1000.0f);
        this->get_parameter(PROJECT_NAME + ".globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius);

        this->declare_parameter(PROJECT_NAME + ".globalMapVisualizationPoseDensity", 10.0f);
        this->get_parameter(PROJECT_NAME + ".globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity);

        this->declare_parameter(PROJECT_NAME + ".globalMapVisualizationLeafSize", 1.0f);
        this->get_parameter(PROJECT_NAME + ".globalMapVisualizationLeafSize", globalMapVisualizationLeafSize);

        // Declare and get int parameters
        this->declare_parameter(PROJECT_NAME + ".edgeFeatureMinValidNum", 10);
        this->get_parameter(PROJECT_NAME + ".edgeFeatureMinValidNum", edgeFeatureMinValidNum);

        this->declare_parameter(PROJECT_NAME + ".surfFeatureMinValidNum", 100);
        this->get_parameter(PROJECT_NAME + ".surfFeatureMinValidNum", surfFeatureMinValidNum);

        this->declare_parameter(PROJECT_NAME + ".numberOfCores", 2);
        this->get_parameter(PROJECT_NAME + ".numberOfCores", numberOfCores);

        this->declare_parameter(PROJECT_NAME + ".surroundingKeyframeSize", 50);
        this->get_parameter(PROJECT_NAME + ".surroundingKeyframeSize", surroundingKeyframeSize);

        this->declare_parameter(PROJECT_NAME + ".historyKeyframeSearchNum", 25);
        this->get_parameter(PROJECT_NAME + ".historyKeyframeSearchNum", historyKeyframeSearchNum);

        // Declare and get double parameters
        this->declare_parameter(PROJECT_NAME + ".mappingProcessInterval", 0.15);
        this->get_parameter(PROJECT_NAME + ".mappingProcessInterval", mappingProcessInterval);

        // Declare and get bool parameters
        this->declare_parameter(PROJECT_NAME + ".loopClosureEnableFlag", false);
        this->get_parameter(PROJECT_NAME + ".loopClosureEnableFlag", loopClosureEnableFlag);
    }

    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu &imu_in)
    {
        auto imu_out = imu_in;
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();

        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();

        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (q_final.norm() < 0.1)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
            rclcpp::shutdown();
        }

        return imu_out;
    }
};

// ROS 2-compatible publishCloud
template<typename T>
sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, T cloud, rclcpp::Time stamp, std::string frame_id)
{
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    if (pub->get_subscription_count() > 0)
        pub->publish(msg);
    return msg;
}

template<typename T>
double ROS_TIME(T msg)
{
    return rclcpp::Time(msg->header.stamp).seconds();
}

template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *msg, T *x, T *y, T *z)
{
    *x = msg->angular_velocity.x;
    *y = msg->angular_velocity.y;
    *z = msg->angular_velocity.z;
}

template<typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *msg, T *x, T *y, T *z)
{
    *x = msg->linear_acceleration.x;
    *y = msg->linear_acceleration.y;
    *z = msg->linear_acceleration.z;
}

void imuRPY2rosRPY(sensor_msgs::msg::Imu* imu_in, float* roll, float* pitch, float* yaw)
{
    tf2::Quaternion q;
    tf2::fromMsg(imu_in->orientation, q);

    double r, p, y;
    tf2::Matrix3x3(q).getRPY(r, p, y);

    *roll  = static_cast<float>(r);
    *pitch = static_cast<float>(p);
    *yaw   = static_cast<float>(y);
}

float pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                (p1.y - p2.y) * (p1.y - p2.y) +
                (p1.z - p2.z) * (p1.z - p2.z));
}

#endif
