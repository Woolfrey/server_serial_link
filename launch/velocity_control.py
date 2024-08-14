import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression

def generate_launch_description():

    urdf_path = '/home/woolfrey/workspace/colcon/src/server_serial_link/test/iiwa14.urdf' # CHANGE THIS
    
    return LaunchDescription([
        
        # Model settings
        DeclareLaunchArgument('urdf_location', default_value=urdf_path),
        DeclareLaunchArgument('endpoint_name', default_value="link7"),
        
       
        # MuJoCo Interface Node
        Node(
            package='serial_link_action_server',
            executable='velocity_control_server',
            output='screen',
            parameters=[{
                'urdf_location': LaunchConfiguration('urdf_location'),
                'endpoint_name': LaunchConfiguration('endpoint_name')
            }]
        )
    ])

