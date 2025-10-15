#include "parameters.h"

std::string PROJECT_NAME;

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string IMU_TOPIC;
int ROW, COL;
int TD, TR;

int USE_LIDAR;
int ALIGN_CAMERA_LIDAR_COORDINATE;


void readParameters(std::shared_ptr<rclcpp::Node> n)
{
    std::string config_file;
    n->get_parameter("vins_config_file", config_file);
    // cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    // if(!fsSettings.isOpened())
    // {
    //     std::cerr << "ERROR: Wrong path to settings" << std::endl;
    // }

    n->declare_parameter("PROJECT_NAME", "");
    n->get_parameter("PROJECT_NAME", PROJECT_NAME);
    // fsSettings["project_name"] >> PROJECT_NAME;
    std::string pkg_path = get_package_share_directory(PROJECT_NAME);

    n->declare_parameter("imu_topic", "");
    n->get_parameter("imu_topic", IMU_TOPIC);
    // fsSettings["imu_topic"] >> IMU_TOPIC;

    n->declare_parameter("use_lidar", 1);
    n->get_parameter("use_lidar", USE_LIDAR);
    n->declare_parameter("align_camera_lidar_estimation", 1);
    n->get_parameter("align_camera_lidar_estimation", ALIGN_CAMERA_LIDAR_COORDINATE);
    // fsSettings["use_lidar"] >> USE_LIDAR;
    // fsSettings["align_camera_lidar_estimation"] >> ALIGN_CAMERA_LIDAR_COORDINATE;

    n->declare_parameter("max_solver_time", 1.0);
    n->get_parameter("max_solver_time", SOLVER_TIME);
    n->declare_parameter("max_num_iterations", 1);
    n->get_parameter("max_num_iterations", NUM_ITERATIONS);
    n->declare_parameter("keyframe_parallax", 1.0);
    n->get_parameter("keyframe_parallax", MIN_PARALLAX);
    // SOLVER_TIME = fsSettings["max_solver_time"];
    // NUM_ITERATIONS = fsSettings["max_num_iterations"];
    // MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    n->declare_parameter("acc_n", 1.0);
    n->get_parameter("acc_n", ACC_N);
    n->declare_parameter("acc_w", 1.0);
    n->get_parameter("acc_w", ACC_W);
    n->declare_parameter("gyr_n", 1.0);
    n->get_parameter("gyr_n", GYR_N);
    n->declare_parameter("gyr_w", 1.0);
    n->get_parameter("gyr_w", GYR_W);
    n->declare_parameter("g_norm", 1.0);
    n->get_parameter("g_norm", G.z());
    n->declare_parameter("image_height", 720);
    n->get_parameter("image_height", ROW);
    n->declare_parameter("image_width", 540);
    n->get_parameter("image_width", COL);
    // ACC_N = fsSettings["acc_n"];
    // ACC_W = fsSettings["acc_w"];
    // GYR_N = fsSettings["gyr_n"];
    // GYR_W = fsSettings["gyr_w"];
    // G.z() = fsSettings["g_norm"];
    // ROW = fsSettings["image_height"];
    // COL = fsSettings["image_width"];
    RCLCPP_INFO(n->get_logger(), "Image dimention: ROW: %d COL: %d ", ROW, COL);

    n->declare_parameter("estimate_extrinsic", 1);
    n->get_parameter("estimate_extrinsic", ESTIMATE_EXTRINSIC);
    // ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        RCLCPP_INFO(n->get_logger(), "have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = pkg_path + "/config/extrinsic_parameter.csv";

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            RCLCPP_INFO(n->get_logger(), " Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = pkg_path + "/config/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            RCLCPP_INFO(n->get_logger(), " Fix extrinsic param.");

        cv::Mat cv_R, cv_T;
        std::vector<long int> cv_R_int;
        std::vector<double> cv_T_vec, cv_R_vec;
        n->declare_parameter("extrinsicRotation", std::vector<long int>(9, 0.0));
        n->get_parameter("extrinsicRotation", cv_R_int);
        n->declare_parameter("extrinsicTranslation", std::vector<double>(3, 0.0));
        n->get_parameter("extrinsicTranslation", cv_T_vec);
        cv_R_vec.assign(cv_R_int.begin(), cv_R_int.end());
        cv_R = cv::Mat(3, 3, CV_64F, cv_R_vec.data()).clone(); 
        cv_T = cv::Mat(3, 1, CV_64F, cv_T_vec.data()).clone(); 
        // fsSettings["extrinsicRotation"] >> cv_R;
        // fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        RCLCPP_INFO_STREAM(n->get_logger(), "Extrinsic_R : " << std::endl << RIC[0]);
        RCLCPP_INFO_STREAM(n->get_logger(), "Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    
    n->declare_parameter("td", 1);
    n->get_parameter("td", TD);
    n->declare_parameter("estimate_td", 1);
    n->get_parameter("estimate_td", ESTIMATE_TD);
    // TD = fsSettings["td"];
    // ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        RCLCPP_INFO_STREAM(n->get_logger(), "Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        RCLCPP_INFO_STREAM(n->get_logger(), "Synchronized sensors, fix time offset: " << TD);

    n->declare_parameter("rolling_shutter", 1);
    n->get_parameter("rolling_shutter", ROLLING_SHUTTER);
    // ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        // TR = fsSettings["rolling_shutter_tr"];
        n->declare_parameter("rolling_shutter_tr", 1);
        n->get_parameter("rolling_shutter_tr", TR);
        RCLCPP_INFO_STREAM(n->get_logger(), "rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }
    
    // fsSettings.release();
    usleep(100);
}
