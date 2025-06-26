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
        'CirclePath.yaml'
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

    yaw_setter = Node(
        package='frontal_lobe',
        executable='yaw_integrator_node',
        name='circle_yaw',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        namespace='path',
        remappings=[
           ('measure', '/est/odom'), 
            
            # -------------------------- #       
            ('yaw', '/attitude/yaw_set'), 
            # -------------------------- #
            
            ('trigger', 'circle_reset'),
        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(yaw_setter)
    # ------------------------------------------ #
    
    

    # Run everything
    return ld