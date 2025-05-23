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
        get_package_share_directory('cerebellum'),
        'config', 
        'Depth&AttitudeController.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value = override_config_arg,
        description='Full path to the YAML configuration file'
    )
    ld.add_action(config_arg)

    config_file = LaunchConfiguration('config_file')

    # ------------------------------------------ #

    # ------ Start Basic Motor Functions ---------- #
    attitude_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cerebellum'),
                'launch',  
                'attitude_controller.launch.py'
            )
        ),
        launch_arguments={
            'config_file': config_file
        }.items()
    )
    
    # Add the hardware launch at the beginning
    ld.add_action(attitude_ctrl_launch)

    # ------------------------------------------ #
    depth_pid_node = Node(
        package='cerebellum',
        executable='pid_node',
        name='ctrl',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'INFO'],
        namespace='depth',
        remappings=[
            ('reference', 'z_set'),
            ('measurement', '/est/odom'),

            ('control_value', '/attitude/pitch_set'),

            ('reset_controller', 'reset_ctrl')
        ]
    )

    ld.add_action(depth_pid_node)
    # ------------------------------------------ #
    

    # Run everything
    return ld