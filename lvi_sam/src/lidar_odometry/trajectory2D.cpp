#include "utility.h"
#include "lvi_sam_msgs/msg/cloud_info.hpp"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <opencv2/opencv.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cmath>
#include <deque>

using namespace gtsam;
using std::placeholders::_1;

class carTrajectory : public ParamServer
{
public:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

    nav_msgs::msg::Path pathMsg;
    bool refInit = false;
    double refLat, refLon, refAlt;
    geometry_msgs::msg::Point lastPoint;

    std::deque<geometry_msgs::msg::Point> smoothingBuffer;
    int smoothWindow = 5;

    carTrajectory(){
        subGPS = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gpsTopic, 50,
            std::bind(&carTrajectory::gpsHandler, this, _1));

        pubPath = this->create_publisher<nav_msgs::msg::Path>(PROJECT_NAME + "/lidar/trajectory", 1);

        pathMsg.header.frame_id = "map";
    }

    void gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr gpsMsg)
    {
        if (gpsMsg->status.status < 0) {
            return;
        }

        if (!refInit) {
            refLat = gpsMsg->latitude;
            refLon = gpsMsg->longitude;
            refAlt = gpsMsg->altitude;
            refInit = true;
            return;
        }

        // Convert GPS → ENU
        geometry_msgs::msg::Point enu = gps2enu(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);

        // Outlier rejection (skip large jumps)
        if (!pathMsg.poses.empty()) {
            double dx = enu.x - lastPoint.x;
            double dy = enu.y - lastPoint.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist > 10.0) { // >10 meters jump in one step
                RCLCPP_WARN(this->get_logger(), "Skipping GPS outlier (jump = %.2f m)", dist);
                return;
            }
        }

        lastPoint = enu;

        // Add to smoothing buffer
        smoothingBuffer.push_back(enu);
        if ((int)smoothingBuffer.size() > smoothWindow)
            smoothingBuffer.pop_front();

        // Compute moving average
        geometry_msgs::msg::Point smoothed;
        for (auto &p : smoothingBuffer) {
            smoothed.x += p.x;
            smoothed.y += p.y;
            smoothed.z += p.z;
        }
        smoothed.x /= smoothingBuffer.size();
        smoothed.y /= smoothingBuffer.size();
        smoothed.z /= smoothingBuffer.size();

        // Add to Path
        geometry_msgs::msg::PoseStamped pose;
        pose.header = gpsMsg->header;
        pose.header.frame_id = "map";
        pose.pose.position = smoothed;
        pose.pose.orientation.w = 1.0;
        pose.header.stamp = this->now();

        pathMsg.poses.push_back(pose);
        pathMsg.header.stamp = this->now();
        pathMsg.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "GPS point published");
        pubPath->publish(pathMsg);
    }

    geometry_msgs::msg::Point gps2enu(double lat, double lon, double alt)
    {
        // WGS84 ellipsoid constants
        double a = 6378137.0;
        double f = 1 / 298.257223563;
        double b = a * (1 - f);
        double e_sq = f * (2 - f);

        double deg2rad = M_PI / 180.0;
        double lambda = lat * deg2rad;
        double phi = lon * deg2rad;

        double sin_lambda = std::sin(lambda);
        double cos_lambda = std::cos(lambda);
        double sin_phi = std::sin(phi);
        double cos_phi = std::cos(phi);

        double N = a / std::sqrt(1 - e_sq * sin_lambda * sin_lambda);

        // ECEF coordinates
        double x = (N + alt) * cos_lambda * cos_phi;
        double y = (N + alt) * cos_lambda * sin_phi;
        double z = (N * (1 - e_sq) + alt) * sin_lambda;

        // Reference point in ECEF
        double N0 = a / std::sqrt(1 - e_sq * std::sin(refLat * deg2rad) * std::sin(refLat * deg2rad));
        double x0 = (N0 + refAlt) * std::cos(refLat * deg2rad) * std::cos(refLon * deg2rad);
        double y0 = (N0 + refAlt) * std::cos(refLat * deg2rad) * std::sin(refLon * deg2rad);
        double z0 = (N0 * (1 - e_sq) + refAlt) * std::sin(refLat * deg2rad);

        // ENU rotation
        double dx = x - x0;
        double dy = y - y0;
        double dz = z - z0;

        double sin_lat0 = std::sin(refLat * deg2rad);
        double cos_lat0 = std::cos(refLat * deg2rad);
        double sin_lon0 = std::sin(refLon * deg2rad);
        double cos_lon0 = std::cos(refLon * deg2rad);

        geometry_msgs::msg::Point enu;
        enu.x = -sin_lon0 * dx + cos_lon0 * dy;
        enu.y = -cos_lon0 * sin_lat0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz;
        enu.z = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz;

        return enu;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<carTrajectory>();

    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Car Trajectory Plotting Started.\033[0m");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
