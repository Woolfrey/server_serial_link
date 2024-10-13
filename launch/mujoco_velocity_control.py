import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Load configuration files
    config_dir = os.path.join(get_package_share_directory('serial_link_action_server'), 'config')
    control_params = os.path.join(config_dir, 'mujoco_velocity_control.yaml')

    # Define LaunchConfigurations for parameters with default values
    urdf_location = os.path.join(get_package_share_directory('serial_link_action_server'), 'test', 'iiwa14.urdf')
    endpoint_name = LaunchConfiguration('endpoint_name', default='link7')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('urdf_location', default_value=urdf_location, description='Path to the URDF file'),
        DeclareLaunchArgument('endpoint_name', default_value='link7', description='Name of the endpoint'),

        # Node configuration for velocity control server
        Node(
            package='serial_link_action_server',
            executable='velocity_control_server',
            output='screen',
            parameters=[
                control_params,
                {
                    'urdf_location': urdf_location,
                    'endpoint_name': endpoint_name,
                    'control_topic_name': 'joint_commands'
                }
            ]
        ),

        # Node configuration for MujocoRelayNode
        Node(
            package='serial_link_action_server',  # Your package name
            executable='mujoco_relay',            # The name of your executable
            output='screen',                      # Display output on the screen
            name='mujoco_relay',                  # Optional: you can name the node
            parameters=[{
                'subscription_name': 'mujoco_relay',  # Subscription topic
                'publication_name': 'joint_commands'  # Publication topic
            }]
        )
    ])

