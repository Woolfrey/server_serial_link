import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([                 
        Node(
            package='serial_link_action_server',
            executable='demo_client',
            output='screen',
            parameters=[
                {
                    'number_of_joints': 7 # Change this to match the robot
                }
            ]
        )
    ])

