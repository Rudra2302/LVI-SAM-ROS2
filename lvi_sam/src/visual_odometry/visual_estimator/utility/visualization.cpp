#include "visualization.h"

using namespace Eigen;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry, pub_latest_odometry, pub_latest_odometry_ros;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_point_cloud, pub_margin_cloud;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_poses;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_camera_pose;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_keyframe_pose;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_keyframe_point;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_extrinsic;

nav_msgs::msg::Path path;

CameraPoseVisualization cameraposevisual(0, 1, 0, 1);
CameraPoseVisualization keyframebasevisual(0.0, 0.0, 1.0, 1.0);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

void registerPub(std::shared_ptr<rclcpp::Node> n)
{
    pub_latest_odometry     = n->create_publisher<nav_msgs::msg::Odometry>(PROJECT_NAME + "/vins/odometry/imu_propagate", 10);
    pub_latest_odometry_ros = n->create_publisher<nav_msgs::msg::Odometry>(PROJECT_NAME + "/vins/odometry/imu_propagate_ros", 10);
    pub_path                = n->create_publisher<nav_msgs::msg::Path>(PROJECT_NAME + "/vins/odometry/path", 10);
    pub_odometry            = n->create_publisher<nav_msgs::msg::Odometry>(PROJECT_NAME + "/vins/odometry/odometry", 10);
    pub_point_cloud         = n->create_publisher<sensor_msgs::msg::PointCloud>(PROJECT_NAME + "/vins/odometry/point_cloud", 10);
    pub_margin_cloud        = n->create_publisher<sensor_msgs::msg::PointCloud>(PROJECT_NAME + "/vins/odometry/history_cloud", 10);
    pub_key_poses           = n->create_publisher<visualization_msgs::msg::Marker>(PROJECT_NAME + "/vins/odometry/key_poses", 10);
    pub_camera_pose         = n->create_publisher<nav_msgs::msg::Odometry>(PROJECT_NAME + "/vins/odometry/camera_pose", 10);
    pub_camera_pose_visual  = n->create_publisher<visualization_msgs::msg::MarkerArray>(PROJECT_NAME + "/vins/odometry/camera_pose_visual", 10);
    pub_keyframe_pose       = n->create_publisher<nav_msgs::msg::Odometry>(PROJECT_NAME + "/vins/odometry/keyframe_pose", 10);
    pub_keyframe_point      = n->create_publisher<sensor_msgs::msg::PointCloud>(PROJECT_NAME + "/vins/odometry/keyframe_point", 10);
    pub_extrinsic           = n->create_publisher<nav_msgs::msg::Odometry>(PROJECT_NAME + "/vins/odometry/extrinsic", 10);

    cameraposevisual.setScale(1);
    cameraposevisual.setLineWidth(0.05);
    keyframebasevisual.setScale(0.1);
    keyframebasevisual.setLineWidth(0.01);
}

geometry_msgs::msg::TransformStamped transformConversion(const geometry_msgs::msg::TransformStamped &t)
{
    geometry_msgs::msg::TransformStamped tf_out;
    tf_out.header = t.header;
    tf_out.child_frame_id = t.child_frame_id;
    tf_out.transform.translation = t.transform.translation;

    // Get RPY from quaternion
    tf2::Quaternion quat;
    tf2::fromMsg(t.transform.rotation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // Reconstruct quaternion from RPY to clean it (if needed)
    tf2::Quaternion quat_clean;
    quat_clean.setRPY(roll, pitch, yaw);
    tf_out.transform.rotation = tf2::toMsg(quat_clean);

    return tf_out;
}

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::msg::Header &header, const int& failureId)
{
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    static double last_align_time = -1;

    // Quternion not normalized
    if (Q.norm() < 0.99)
        return;

    // imu odometry in camera frame
    nav_msgs::msg::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "vins_world";
    odometry.child_frame_id = "vins_body";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry->publish(odometry);

    // imu odometry in ROS format (change rotation), used for lidar odometry initial guess
    odometry.pose.covariance[0] = static_cast<double>(failureId); // notify lidar odometry failure

    tf2::Quaternion q_odom_cam(Q.x(), Q.y(), Q.z(), Q.w());
    tf2::Quaternion q_cam_to_lidar(0, 1, 0, 0); // rotation
    tf2::Quaternion q_odom_ros = q_odom_cam * q_cam_to_lidar;

    geometry_msgs::msg::Quaternion q_msg;
    q_msg = tf2::toMsg(q_odom_ros);
    odometry.pose.pose.orientation = q_msg;

    pub_latest_odometry_ros->publish(odometry);

    // TF of camera in vins_world in ROS format (change rotation), used for depth registration
    geometry_msgs::msg::TransformStamped tf_cam;
    tf_cam.header = header;
    tf_cam.child_frame_id = "vins_body_ros";
    tf_cam.header.frame_id = "vins_world";
    tf_cam.transform.translation.x = P.x();
    tf_cam.transform.translation.y = P.y();
    tf_cam.transform.translation.z = P.z();
    tf_cam.transform.rotation = q_msg;
    tf_broadcaster->sendTransform(tf_cam);

    // Handle camera-lidar alignment
    if (ALIGN_CAMERA_LIDAR_COORDINATE)
    {
        static geometry_msgs::msg::TransformStamped tf_odom_world;

        rclcpp::Time stamp = header.stamp;

        if ((stamp.seconds() - last_align_time) > 1.0)
        {
            try
            {
                auto tf_odom_baselink = tf_buffer->lookupTransform(
                    "odom", "base_link", tf2::TimePointZero);

                tf2::Transform T_odom_baselink, T_world_vins;
                tf2::fromMsg(tf_odom_baselink.transform, T_odom_baselink);
                tf2::fromMsg(tf_cam.transform, T_world_vins);

                tf2::Transform T_odom_world = T_odom_baselink * T_world_vins.inverse();
                tf_odom_world.header.stamp = header.stamp;
                tf_odom_world.header.frame_id = "odom";
                tf_odom_world.child_frame_id = "vins_world";
                tf_odom_world.transform = tf2::toMsg(T_odom_world);
                last_align_time = stamp.seconds();
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(rclcpp::get_logger("estimator"), "TF Exception: %s", ex.what());
                return;
            }
        }

        tf_broadcaster->sendTransform(tf_odom_world);
    }
    else
    {
        tf2::Quaternion q_static;
        q_static.setRPY(0, 0, M_PI);
        geometry_msgs::msg::TransformStamped tf_static;
        tf_static.header = header;
        tf_static.header.frame_id = "odom";
        tf_static.child_frame_id = "vins_world";
        tf_static.transform.translation.x = 0.0;
        tf_static.transform.translation.y = 0.0;
        tf_static.transform.translation.z = 0.0;
        tf_static.transform.rotation = tf2::toMsg(q_static);
        tf_broadcaster->sendTransform(tf_static);
    }
}

void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("estimator"), "position: " << estimator.Ps[WINDOW_SIZE].transpose());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("estimator"), "orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        //ROS_DEBUG("calibration result for camera %d", i);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("estimator"), "extirnsic tic: " << estimator.tic[i].transpose());
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("estimator"), "extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
        if (ESTIMATE_EXTRINSIC)
        {
            // cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            eigen_R = estimator.ric[i];
            eigen_T = estimator.tic[i];
            cv::Mat cv_R, cv_T;
            cv::eigen2cv(eigen_R, cv_R);
            cv::eigen2cv(eigen_T, cv_T);
            // fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
            // fs.release();
        }
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "vo solver costs: %f ms", t);
    RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "sum of path %f", sum_of_path);
    if (ESTIMATE_TD)
        RCLCPP_INFO(rclcpp::get_logger("estimator"), "td %f", estimator.td);
}

void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::msg::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "vins_world";
        odometry.child_frame_id = "vins_world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry->publish(odometry);

        static double path_save_time = -1;
        if (rclcpp::Time(header.stamp).seconds() - path_save_time > 0.5)
        {
            path_save_time = rclcpp::Time(header.stamp).seconds();
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = header;
            pose_stamped.header.frame_id = "vins_world";
            pose_stamped.pose = odometry.pose.pose;
            path.header = header;
            path.header.frame_id = "vins_world";
            path.poses.push_back(pose_stamped);
            pub_path->publish(path);
        }
    }
}

void pubKeyPoses(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (pub_key_poses->get_subscription_count() == 0)
        return;

    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::msg::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "vins_world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::msg::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = rclcpp::Duration::from_seconds(0.0);

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::msg::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses->publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (pub_camera_pose_visual->get_subscription_count() == 0)
        return;

    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        nav_msgs::msg::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "vins_world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose->publish(odometry);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}


void pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if (pub_point_cloud->get_subscription_count() != 0)
    {
        sensor_msgs::msg::PointCloud point_cloud;
        point_cloud.header = header;
        point_cloud.header.frame_id = "vins_world";

        sensor_msgs::msg::ChannelFloat32 intensity_channel;
        intensity_channel.name = "intensity";

        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int used_num;
            used_num = it_per_id.feature_per_frame.size();
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
                continue;
            
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

            geometry_msgs::msg::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            point_cloud.points.push_back(p);

            if (it_per_id.lidar_depth_flag == false)
                intensity_channel.values.push_back(0);
            else
                intensity_channel.values.push_back(1);
        }

        point_cloud.channels.push_back(intensity_channel);
        pub_point_cloud->publish(point_cloud);
    }
    
    // pub margined potin
    if (pub_margin_cloud->get_subscription_count() != 0)
    {
        sensor_msgs::msg::PointCloud margin_cloud;
        margin_cloud.header = header;
        margin_cloud.header.frame_id = "vins_world";

        sensor_msgs::msg::ChannelFloat32 intensity_channel;
        intensity_channel.name = "intensity";

        for (auto &it_per_id : estimator.f_manager.feature)
        { 
            int used_num;
            used_num = it_per_id.feature_per_frame.size();
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

            if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 
                && it_per_id.solve_flag == 1 )
            {
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

                geometry_msgs::msg::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                margin_cloud.points.push_back(p);

                if (it_per_id.lidar_depth_flag == false)
                    intensity_channel.values.push_back(0);
                else
                    intensity_channel.values.push_back(1);
            }
        }

        margin_cloud.channels.push_back(intensity_channel);
        pub_margin_cloud->publish(margin_cloud);
    }
}


void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    const std::shared_ptr<rclcpp::Node> node;
    static tf2_ros::TransformBroadcaster tf_broadcaster(node);
    geometry_msgs::msg::TransformStamped body_tf;
    body_tf.header.stamp = header.stamp;
    body_tf.header.frame_id = "vins_world";
    body_tf.child_frame_id = "vins_body";

    body_tf.transform.translation.x = estimator.Ps[0].x() ;
    body_tf.transform.translation.y = estimator.Ps[0].y() ;
    body_tf.transform.translation.z = estimator.Ps[0].z() ;

    Eigen::Quaterniond q(estimator.Rs[0]);
    body_tf.transform.rotation.x = q.x();
    body_tf.transform.rotation.y = q.y();
    body_tf.transform.rotation.z = q.z();
    body_tf.transform.rotation.w = q.w();

    tf_broadcaster.sendTransform(body_tf);

    // camera frame
    geometry_msgs::msg::TransformStamped cam_tf;
    cam_tf.header = header;
    cam_tf.header.frame_id = "vins_body";
    cam_tf.child_frame_id = "vins_camera";

    cam_tf.transform.translation.x = estimator.tic[0].x();
    cam_tf.transform.translation.y = estimator.tic[0].y();
    cam_tf.transform.translation.z = estimator.tic[0].z();

    Eigen::Quaterniond ric_quat(estimator.ric[0]);
    cam_tf.transform.rotation.x = ric_quat.x();
    cam_tf.transform.rotation.y = ric_quat.y();
    cam_tf.transform.rotation.z = ric_quat.z();
    cam_tf.transform.rotation.w = ric_quat.w();

    tf_broadcaster.sendTransform(cam_tf);

    nav_msgs::msg::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "vins_world";
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();

    odometry.pose.pose.orientation.x = ric_quat.x();
    odometry.pose.pose.orientation.y = ric_quat.y();
    odometry.pose.pose.orientation.z = ric_quat.z();
    odometry.pose.pose.orientation.w = ric_quat.w();
    pub_extrinsic->publish(odometry);
}

void pubKeyframe(const Estimator &estimator)
{
    if (pub_keyframe_pose->get_subscription_count() == 0 && pub_keyframe_point->get_subscription_count() == 0)
        return;

    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::msg::Odometry odometry;
        odometry.header = estimator.Headers[WINDOW_SIZE - 2];
        odometry.header.frame_id = "vins_world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_keyframe_pose->publish(odometry);


        sensor_msgs::msg::PointCloud point_cloud;
        point_cloud.header = estimator.Headers[WINDOW_SIZE - 2];
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if(it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                                      + estimator.Ps[imu_i];
                geometry_msgs::msg::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::msg::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }
        }
        pub_keyframe_point->publish(point_cloud);
    }
}