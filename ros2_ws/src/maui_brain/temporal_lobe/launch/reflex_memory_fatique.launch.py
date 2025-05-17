import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    ld = LaunchDescription()
    

    default_config_arg = os.path.join( # Path to parameters
        get_package_share_directory('temporal_lobe'),
        'config', 
        'ReflexMemoryFatique.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value = default_config_arg,
        description='Full path to the YAML configuration file'
    )
    ld.add_action(config_arg)

    config_file = LaunchConfiguration('config_file') 

    # ------------------------------------------ #

    reflex = Node(
        package='temporal_lobe',
        executable='button_service_caller_node',
        name='reflex',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'INFO'],
        output='screen',
        remappings=[
             ('/button_state', '/button_state'),
        ]
    )

    ld.add_action(reflex)

    # ------------------------------------------ #

    memory = Node(
        package='temporal_lobe',
        executable='record_service_node',
        name='memory',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        remappings=[
            ('/toggle_bag_recording', '/record'),
        ]
    )

    ld.add_action(memory)

    # ------------------------------------------ #

    fatique = Node(
        package='temporal_lobe',
        executable='button_timeout_node',
        name='timeout',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        remappings=[
            ('/button_state', '/button_state'),
        ]
    )

    ld.add_action(fatique)

    # ------------------------------------------ #

    return ld