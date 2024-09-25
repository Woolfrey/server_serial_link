import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
    
        # Declare a launch argument to allow overriding from command line
        DeclareLaunchArgument(
            'number_of_joints',
            default_value='7',                                                                      # Default value if not overridden
            description='Number of joints'
        ),

        # Node definition with the parameter
        Node(
            package='serial_link_action_server',
            executable='demo_client',
            output='screen',
            parameters=[{
                'number_of_joints': LaunchConfiguration('number_of_joints')
            }]
        )
    ])

