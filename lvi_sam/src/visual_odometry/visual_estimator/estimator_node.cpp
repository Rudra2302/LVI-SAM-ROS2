#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"

using std::placeholders::_1;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buf; 
queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> feature_buf;

// global variable saving the lidar odometry
deque<nav_msgs::msg::Odometry> odomQueue;
odometryRegister *odomRegister;

std::mutex m_buf;
std::mutex m_state;
std::mutex m_estimator;
std::mutex m_odom;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

void predict(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg, Estimator &estimator)
{
    double t = rclcpp::Time(imu_msg->header.stamp).seconds();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update(Estimator &estimator)
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::msg::Imu::ConstSharedPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::msg::Imu::ConstSharedPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front(), estimator);
}

std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>>
getMeasurements(Estimator &estimator)
{
    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>> measurements;

    while (rclcpp::ok())
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(rclcpp::Time(imu_buf.back()->header.stamp).seconds() > rclcpp::Time(feature_buf.front()->header.stamp).seconds() + estimator.td))
        {
            return measurements;
        }

        if (!(rclcpp::Time(imu_buf.front()->header.stamp).seconds() < rclcpp::Time(feature_buf.front()->header.stamp).seconds() + estimator.td))
        {
            // RCLPCPP_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::msg::PointCloud::ConstSharedPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> IMUs;
        while (rclcpp::Time(imu_buf.front()->header.stamp).seconds() < rclcpp::Time(img_msg->header.stamp).seconds() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        // if (IMUs.empty())
        //     ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg, Estimator &estimator)
{
    if (rclcpp::Time(imu_msg->header.stamp).seconds() <= last_imu_t)
    {
        // ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = rclcpp::Time(imu_msg->header.stamp).seconds();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = rclcpp::Time(imu_msg->header.stamp).seconds();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg, estimator);
        std_msgs::msg::Header header = imu_msg->header;
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header, estimator.failureCount);
    }
}

void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) 
{
    m_odom.lock();
    odomQueue.push_back(*odom_msg);
    m_odom.unlock();
}

void feature_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void restart_callback(const std_msgs::msg::Bool::ConstSharedPtr &restart_msg, Estimator &estimator) 
{
    if (restart_msg->data == true)
    {
        // ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

// thread: visual-inertial odometry
void process(Estimator &estimator)
{
    while (rclcpp::ok())
    {
        std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements(estimator)).size() != 0;
                 });
        lk.unlock();

        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;

            // 1. IMU pre-integration
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = rclcpp::Time(imu_msg->header.stamp).seconds();
                double img_t = rclcpp::Time(img_msg->header.stamp).seconds() + estimator.td;
                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    assert(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            // 2. VINS Optimization
            // TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                double depth = img_msg->channels[5].values[i];

                assert(z == 1);
                Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
                xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
            }

            // Get initialization info from lidar odometry
            vector<float> initialization_info;
            m_odom.lock();
            initialization_info = odomRegister->getOdometry(odomQueue, rclcpp::Time(img_msg->header.stamp).seconds() + estimator.td);
            m_odom.unlock();


            estimator.processImage(image, initialization_info, img_msg->header);
            // double whole_t = t_s.toc();
            // printStatistics(estimator, whole_t);

            // 3. Visualization
            std_msgs::msg::Header header = img_msg->header;
            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
        }
        m_estimator.unlock();

        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update(estimator);
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("vins");
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator Started.\033[0m");
    // if (rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_WARN) != RCUTILS_RET_OK) {
    //     RCLCPP_WARN(node->get_logger(), "Failed to set logger level.");
    // }
    Estimator estimator;

    readParameters(node);
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator readParameters completed.\033[0m");
    estimator.setParameter();
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator setParameter completed.\033[0m");
    registerPub(node);
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator registerPub completed.\033[0m");
    odomRegister = new odometryRegister(node);
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator odometryRegister constructor completed.\033[0m");

    // ros::Subscriber sub_imu     = n.subscribe(IMU_TOPIC,      5000, imu_callback,  ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_odom    = n.subscribe("odometry/imu", 5000, odom_callback);
    // ros::Subscriber sub_image   = n.subscribe(PROJECT_NAME + "/vins/feature/feature", 1, feature_callback);
    // ros::Subscriber sub_restart = n.subscribe(PROJECT_NAME + "/vins/feature/restart", 1, restart_callback);

    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
        IMU_TOPIC, 5000,
        std::function<void(const sensor_msgs::msg::Imu::ConstSharedPtr&)>(
            std::bind(imu_callback, std::placeholders::_1, std::ref(estimator))));

    auto sub_odom = node->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/imu", 5000,
        odom_callback);

    auto sub_image = node->create_subscription<sensor_msgs::msg::PointCloud>(
        PROJECT_NAME + std::string("/vins/feature/feature"), 1,
        feature_callback);

    auto sub_restart = node->create_subscription<std_msgs::msg::Bool>(
        PROJECT_NAME + std::string("/vins/feature/restart"), 1,
        std::function<void(const std_msgs::msg::Bool::ConstSharedPtr&)>(
            std::bind(restart_callback, std::placeholders::_1, std::ref(estimator))));

    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator Subscribers created.\033[0m");

    if (!USE_LIDAR)
        sub_odom.reset();
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator reset done.\033[0m");

    std::thread measurement_process{process, std::ref(estimator)};
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator measurement_process created.\033[0m");

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator executor created.\033[0m");
    executor.add_node(node);
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator add_node done.\033[0m");
    executor.spin();
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator spin done.\033[0m");

    // if (measurement_process.joinable())
    //     measurement_process.join();

    rclcpp::shutdown();

    return 0;
}