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

    config_file = os.path.join( # Path to parameters
        get_package_share_directory('cerebellum'),
        'config', 
        'Depth&AttitudeController.yaml'
    )

    # ------------------------------------------ #

    # ------ Start Basic Motor Functions ---------- #
    attitude_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cerebellum'),
                'launch',  
                'compact_attitude_controller.launch.py'
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
        arguments=['--ros-args', '--log-level', 'WARN'],
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

    # ------------------------------------------ #
    speed_pid_node = Node(
        package='cerebellum',
        executable='pid_node',
        name='ctrl',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        namespace='speed',
        remappings=[
            ('reference', 'dx_set'),
            ('measurement', 'none'),

            ('control_value', '/input/A_m'),

            ('reset_controller', 'reset_ctrl')
        ]
    )

    ld.add_action(speed_pid_node)
    # ------------------------------------------ #
    

    # Run everything
    return ld