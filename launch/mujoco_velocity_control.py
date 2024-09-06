import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Load configuration files
    config_dir = os.path.join(get_package_share_directory('serial_link_action_server'), 'config')   # Get the path to the config directory
    control_params = os.path.join(config_dir, 'mujoco_velocity_control.yaml')                       # Change this for different control modes, gains
   
    return LaunchDescription([
        Node(
            package='serial_link_action_server',
            executable='velocity_control_server',
            output='screen',
            parameters=[
                control_params,
                {
                    'urdf_location': '/home/woolfrey/workspace/colcon/src/server_serial_link/test/iiwa14.urdf', # CHANGE THIS
                    'endpoint_name': 'link7',
                    'control_topic_name': 'joint_commands'
                }
            ]
        )
    ])
