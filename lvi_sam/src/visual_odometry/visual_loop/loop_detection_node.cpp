#include "parameters.h"
#include "keyframe.h"
#include "loop_detection.h"

#define SKIP_FIRST_CNT 10

queue<sensor_msgs::msg::Image::ConstSharedPtr>      image_buf; 
queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> point_buf;
queue<nav_msgs::msg::Odometry::ConstSharedPtr>    pose_buf;

std::mutex m_buf;
std::mutex m_process;

LoopDetector loopDetector;

double SKIP_TIME = 0;
double SKIP_DIST = 0;

camodocal::CameraPtr m_camera;

Eigen::Vector3d tic;
Eigen::Matrix3d qic;

std::string PROJECT_NAME;
std::string IMAGE_TOPIC;

int DEBUG_IMAGE;
int LOOP_CLOSURE;
double MATCH_IMAGE_SCALE;


rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_img;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_match_msg;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_key_pose;



BriefExtractor briefExtractor;

void new_sequence()
{
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    m_buf.unlock();
}

void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received image data");
    // std::cout<<"Received image data in Visual Odometry\n";
    if(!LOOP_CLOSURE)
        return;

    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();

    // detect unstable camera stream
    static double last_image_time = -1;
    if (last_image_time == -1)
        last_image_time = rclcpp::Time(image_msg->header.stamp).seconds();
    else if (rclcpp::Time(image_msg->header.stamp).seconds() - last_image_time > 1.0 || rclcpp::Time(image_msg->header.stamp).seconds() < last_image_time)
    {
        // ROS_WARN("image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = rclcpp::Time(image_msg->header.stamp).seconds();
}

void point_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr &point_msg)
{
    if(!LOOP_CLOSURE)
        return;

    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
}

void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
{
    if(!LOOP_CLOSURE)
        return;

    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
}

void extrinsic_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}

void process()
{
    if (!LOOP_CLOSURE)
        return;

    while (rclcpp::ok())
    {
        sensor_msgs::msg::Image::ConstSharedPtr image_msg = NULL;
        sensor_msgs::msg::PointCloud::ConstSharedPtr point_msg = NULL;
        nav_msgs::msg::Odometry::ConstSharedPtr pose_msg = NULL;

        // find out the messages with same time stamp
        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (rclcpp::Time(image_buf.front()->header.stamp).seconds() > rclcpp::Time(pose_buf.front()->header.stamp).seconds())
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (rclcpp::Time(image_buf.front()->header.stamp).seconds() > rclcpp::Time(point_buf.front()->header.stamp).seconds())
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (rclcpp::Time(image_buf.back()->header.stamp).seconds() >= rclcpp::Time(pose_buf.front()->header.stamp).seconds()
                && rclcpp::Time(point_buf.back()->header.stamp).seconds() >= rclcpp::Time(pose_buf.front()->header.stamp).seconds())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (rclcpp::Time(image_buf.front()->header.stamp).seconds() < rclcpp::Time(pose_msg->header.stamp).seconds())
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (rclcpp::Time(point_buf.front()->header.stamp).seconds() < rclcpp::Time(pose_msg->header.stamp).seconds())
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != NULL)
        {
            // skip fisrt few
            static int skip_first_cnt = 0;
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            // limit frequency
            static double last_skip_time = -1;
            if (rclcpp::Time(pose_msg->header.stamp).seconds() - last_skip_time < SKIP_TIME)
                continue;
            else
                last_skip_time = rclcpp::Time(pose_msg->header.stamp).seconds();

            // get keyframe pose
            static Eigen::Vector3d last_t(-1e6, -1e6, -1e6);
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();

            // add keyframe
            if((T - last_t).norm() > SKIP_DIST)
            {
                // convert image
                cv_bridge::CvImageConstPtr ptr;
                if (image_msg->encoding == "8UC1")
                {
                    sensor_msgs::msg::Image img;
                    img.header = image_msg->header;
                    img.height = image_msg->height;
                    img.width = image_msg->width;
                    img.is_bigendian = image_msg->is_bigendian;
                    img.step = image_msg->step;
                    img.data = image_msg->data;
                    img.encoding = "mono8";
                    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
                }
                else
                    ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
                
                cv::Mat image = ptr->image;

                vector<cv::Point3f> point_3d; 
                vector<cv::Point2f> point_2d_uv; 
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);
                }

                // new keyframe
                static int global_frame_index = 0;
                KeyFrame* keyframe = new KeyFrame(rclcpp::Time(pose_msg->header.stamp).seconds(), global_frame_index, 
                                                  T, R, 
                                                  image,
                                                  point_3d, point_2d_uv, point_2d_normal, point_id);   

                // detect loop
                m_process.lock();
                loopDetector.addKeyFrame(keyframe, 1);
                m_process.unlock();

                loopDetector.visualizeKeyPoses(rclcpp::Time(pose_msg->header.stamp).seconds());

                global_frame_index++;
                last_t = T;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
} 


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // std::shared_ptr<rclcpp::Node> n;
    auto n = rclcpp::Node::make_shared("visual_loop");
    RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection Started.\033[0m");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    // Load params
    std::string config_file;
    n->declare_parameter("vins_config_file", "");
    n->get_parameter("vins_config_file", config_file);
    RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection config_file found.\033[0m");
    // cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    // if(!fsSettings.isOpened())
    // {
    //     std::cerr << "ERROR: Wrong path to settings" << std::endl;
    // }
    usleep(100);

    // Initialize global params
    n->declare_parameter("PROJECT_NAME", "");
    n->get_parameter("PROJECT_NAME", PROJECT_NAME);
    n->declare_parameter("image_topic", "");
    n->get_parameter("image_topic", IMAGE_TOPIC);
    std::cout<<"Image topic in visual loop is: "<<IMAGE_TOPIC<<std::endl;
    n->declare_parameter("loop_closure", 1);
    n->get_parameter("loop_closure", LOOP_CLOSURE);
    n->declare_parameter("skip_time", 0.0);
    n->get_parameter("skip_time", SKIP_TIME);
    n->declare_parameter("skip_dist", 0.0);
    n->get_parameter("skip_dist", SKIP_DIST);
    n->declare_parameter("debug_image", 0);
    n->get_parameter("debug_image", DEBUG_IMAGE);
    n->declare_parameter("match_image_scale", 0.0);
    n->get_parameter("match_image_scale", MATCH_IMAGE_SCALE);
    RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection parameters declared.\033[0m");
    // fsSettings["project_name"] >> PROJECT_NAME;  
    // fsSettings["image_topic"]  >> IMAGE_TOPIC;  
    // fsSettings["loop_closure"] >> LOOP_CLOSURE;
    // fsSettings["skip_time"]    >> SKIP_TIME;
    // fsSettings["skip_dist"]    >> SKIP_DIST;
    // fsSettings["debug_image"]  >> DEBUG_IMAGE;
    // fsSettings["match_image_scale"] >> MATCH_IMAGE_SCALE;
    
    if (LOOP_CLOSURE)
    {
        RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection LOOP_CLOSURE Started.\033[0m");
        string pkg_path = get_package_share_directory(PROJECT_NAME);
        RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection pkg_path done.\033[0m");

        // initialize vocabulary
        string vocabulary_file;
        n->declare_parameter("vocabulary_file", "");
        n->get_parameter("vocabulary_file", vocabulary_file);
        // fsSettings["vocabulary_file"] >> vocabulary_file;  
        vocabulary_file = pkg_path + vocabulary_file;
        loopDetector.loadVocabulary(vocabulary_file);
        RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection loadVocabulary done.\033[0m");

        // initialize brief extractor
        string brief_pattern_file;
        n->declare_parameter("brief_pattern_file", "");
        n->get_parameter("brief_pattern_file", brief_pattern_file);
        // fsSettings["brief_pattern_file"] >> brief_pattern_file;  
        brief_pattern_file = pkg_path + brief_pattern_file;
        briefExtractor = BriefExtractor(n, brief_pattern_file);
        RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection BriefExtractor done.\033[0m");

        // initialize camera model
        // m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());
    }

    // ros::Subscriber sub_image     = n.subscribe(IMAGE_TOPIC, 30, image_callback);
    // ros::Subscriber sub_pose      = n.subscribe(PROJECT_NAME + "/vins/odometry/keyframe_pose",  3, pose_callback);
    // ros::Subscriber sub_point     = n.subscribe(PROJECT_NAME + "/vins/odometry/keyframe_point", 3, point_callback);
    // ros::Subscriber sub_extrinsic = n.subscribe(PROJECT_NAME + "/vins/odometry/extrinsic",      3, extrinsic_callback);

    auto sub_image = n->create_subscription<sensor_msgs::msg::Image>(
        IMAGE_TOPIC, 30,
        image_callback);

    auto sub_pose = n->create_subscription<nav_msgs::msg::Odometry>(
        PROJECT_NAME + "/vins/odometry/keyframe_pose",  3,
        pose_callback);

    auto sub_point = n->create_subscription<sensor_msgs::msg::PointCloud>(
        PROJECT_NAME + "/vins/odometry/keyframe_point", 3,
        point_callback);

    auto sub_extrinsic = n->create_subscription<nav_msgs::msg::Odometry>(
        PROJECT_NAME + "/vins/odometry/extrinsic", 3,
        extrinsic_callback);
    RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection subscribers created.\033[0m");

    pub_match_img = n->create_publisher<sensor_msgs::msg::Image> (PROJECT_NAME + "/vins/loop/match_image", 3);
    pub_match_msg = n->create_publisher<std_msgs::msg::Float64MultiArray> (PROJECT_NAME + "/vins/loop/match_frame", 3);
    pub_key_pose = n->create_publisher<visualization_msgs::msg::MarkerArray> (PROJECT_NAME + "/vins/loop/keyframe_pose", 3);
    RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection publishers created.\033[0m");

    if (!LOOP_CLOSURE)
    {
        sub_image.reset();
        sub_pose.reset();
        sub_point.reset();
        sub_extrinsic.reset();

        pub_match_img.reset();
        pub_match_msg.reset();
        pub_key_pose.reset();
        rclcpp::shutdown();
    }

    std::thread measurement_process{process};

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(n);
    executor.spin();

    if (measurement_process.joinable())
        measurement_process.join();
        
    rclcpp::shutdown();

    return 0;
}