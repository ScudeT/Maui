import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    ld = LaunchDescription()

    override_config_arg = os.path.join( # Path to parameters
        get_package_share_directory('frontal_lobe'),
        'config', 
        'GPSFollow.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value = override_config_arg,
        description='Full path to the YAML configuration file'
    )
    ld.add_action(config_arg)

    config_file = LaunchConfiguration('config_file')

    # ------------------------------------------ #
    # ------------------------------------------ #

    gps_follower = Node(
        package='frontal_lobe',
        executable='gps_target_follower_node',
        name='gps_follower',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'INFO'],
        output='screen',
        namespace='path',
        remappings=[
           ('gps_data', '/gps_data'), 
           ('odom', '/est/odom'),
            
            # -------------------------- #       
            ('yaw_set', '/attitude/yaw_set'), 
            ('gps_target', 'gps_target'), 
            # -------------------------- #

        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(gps_follower)
    # ------------------------------------------ #
    
    

    # Run everything
    return ld
