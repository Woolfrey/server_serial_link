import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Load configuration files
    config_dir = os.path.join(get_package_share_directory('serial_link_action_server'), 'config')
    control_params = os.path.join(config_dir, 'velocity_mode_sim.yaml')

    # Define LaunchConfigurations for parameters with default values
    urdf = LaunchConfiguration('urdf', default=os.path.join(get_package_share_directory('serial_link_action_server'), 'urdf/kuka_iiwa_14/iiwa14.urdf'))    
    endpoint = LaunchConfiguration('endpoint', default='link7')

    return LaunchDescription([

        # Launch the action server
        Node
        (
            package    = 'serial_link_action_server',
            executable = 'trajectory_tracking_server',
            output     = 'screen',
            parameters =
            [
                control_params,
                {
                    'urdf'         : urdf,
                    'endpoint'     : endpoint,
                    'control_topic': 'joint_commands'
                }
            ]
        ),

        # Launch the MuJoCo control relay
        Node
        (
            package    = 'serial_link_action_server',  
            executable = 'joint_command_relay',
            output     = 'screen',
            name       = 'joint_command_relay',
            parameters =
            [{
                'subscription_name': 'joint_command_relay',
                'publication_name' : 'control_array'
            }]
        )
    ])
