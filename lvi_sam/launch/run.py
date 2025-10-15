from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # pcap_file_arg = DeclareLaunchArgument(
    #     'pcap_file',
    #     default_value='/home/ros2_ws/src/Honda_data/Lidar_02.pcap',
    #     description='Path to the input PCAP file'
    # )

    lvi_sam_dir = get_package_share_directory('lvi_sam')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lvi_sam_dir, 'launch/include/module_robot_state_publisher.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lvi_sam_dir, 'launch/include/module_rviz.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lvi_sam_dir, 'launch/include/module_sam.py')
            ),
            # launch_arguments={'pcap_file': LaunchConfiguration('pcap_file')}.items()
        ),
    ])
