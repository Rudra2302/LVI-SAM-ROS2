from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('lvi_sam'),
        'urdf',
        'robot.urdf.xacro'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['xacro', urdf_path],
            output='screen',
            shell=True
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path])
            }]
        )
    ])
