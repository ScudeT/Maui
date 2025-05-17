import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    ld = LaunchDescription()

    default_config_arg = os.path.join( # Path to parameters
        get_package_share_directory('cerebellum'),
        'config', 
        'BMF.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value = default_config_arg,
        description='Full path to the YAML configuration file'
    )
    ld.add_action(config_arg)

    config_file = LaunchConfiguration('config_file') 

    # ------------------------------------------ #

    ctrl = Node(
        package='cerebellum',
        executable='ctrl_node',
        name='ctrl',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/omega_m', '/omega_m'),
            ('/A_m',     '/A_m',   ),            
            ('/A_d',     '/A_d',   ),
            ('/alpha_m', '/alpha_m'),
            ('/alpha_d', '/alpha_d'),
            
            ('/theta_m', '/theta_m'),
            ('/theta_d', '/theta_d'),           
            
            # -------------------------- #
            ('/command', '/command'),
            # -------------------------- #

            ('/start_and_stop', '/start_and_stop'),
        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(ctrl)
    # ------------------------------------------ #

    return ld