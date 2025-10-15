from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    lvi_sam_dir = get_package_share_directory('lvi_sam')

    lidar_config = os.path.join(lvi_sam_dir, 'config', 'params_lidar.yaml')
    config_file = os.path.join(lvi_sam_dir, 'config', 'params_camera.yaml')
    pattern_file = os.path.join(lvi_sam_dir, 'config', 'brief_pattern.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'pcap_file',
            default_value='/home/ros2_ws/src/lvi_sam/Honda_data/Lidar_02.pcap',
            description='Path to the input PCAP file'
        ),

        DeclareLaunchArgument(
            'mcap_file',
            default_value='/home/ros2_ws/src/lvi_sam/Honda_data/xsens_can/xsens_can_0.mcap',
            description='Path to the input MCAP file'
        ),

        DeclareLaunchArgument(
            'image_file1',
            default_value='/home/ros2_ws/src/lvi_sam/Honda_data/Camera_01.h265',
            description='Path to the input camera.h265 file'
        ),

        DeclareLaunchArgument(
            'image_file2',
            default_value='/home/ros2_ws/src/lvi_sam/Honda_data/Camera_01.txt',
            description='Path to the input camera.txt file'
        ),

        Node(
            package='lvi_sam',
            executable='pcap_publisher_node',
            name='pcap_publisher_node',
            output='screen',
            parameters=[
                {'pcap_file': LaunchConfiguration('pcap_file')},
                {'mcap_file': LaunchConfiguration('mcap_file')},
                {'image_file1': LaunchConfiguration('image_file1')},
                {'image_file2': LaunchConfiguration('image_file2')},
                lidar_config
            ]
        ),

        Node(
            package='lvi_sam',
            executable='image_projection_node',
            name='image_projection_node',
            output='screen',
            parameters=[
                {'pcap_file': LaunchConfiguration('pcap_file')},
                {'mcap_file': LaunchConfiguration('mcap_file')},
                lidar_config
            ]
        ),

        Node(
            package='lvi_sam',
            executable='feature_extraction_node',
            name='feature_extraction_node',
            output='screen',
            parameters=[
                {'pcap_file': LaunchConfiguration('pcap_file')},
                {'mcap_file': LaunchConfiguration('mcap_file')},
                lidar_config
            ]
        ),

        Node(
            package='lvi_sam',
            executable='map_optimization_node',
            name='map_optimization_node',
            output='screen',
            parameters=[
                {'pcap_file': LaunchConfiguration('pcap_file')},
                {'mcap_file': LaunchConfiguration('mcap_file')},
                lidar_config
            ]
        ),

        Node(
            package='lvi_sam',
            executable='imu_preintegration_node',
            name='imu_preintegration_node',
            output='screen',
            parameters=[
                {'pcap_file': LaunchConfiguration('pcap_file')},
                {'mcap_file': LaunchConfiguration('mcap_file')},
                lidar_config
            ]
        ),

        Node(
            package='lvi_sam',
            executable='trajectory_node',
            name='trajectory_node',
            output='screen',
            parameters=[
                {'pcap_file': LaunchConfiguration('pcap_file')},
                {'mcap_file': LaunchConfiguration('mcap_file')},
                lidar_config
            ]
        ),

        Node(
            package='lvi_sam',
            executable='visual_estimator_node',
            name='visual_estimator_node',
            output='screen',
            parameters=[
                config_file,
                pattern_file
            ]
        ),

        Node(
            package='lvi_sam',
            executable='visual_feature_node',
            name='visual_feature_node',
            output='screen',
            parameters=[
                config_file,
                pattern_file
            ]
        ),

        Node(
            package='lvi_sam',
            executable='visual_loop_node',
            name='visual_loop_node',
            output='screen',
            parameters=[
                config_file,
                pattern_file
            ]
        )
    ])
