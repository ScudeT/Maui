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
        get_package_share_directory('maui'),
        'config', 
        'TestYawDepthCtrl.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value = override_config_arg,
        description='Full path to the YAML configuration file'
    )
    ld.add_action(config_arg)

    config_file = LaunchConfiguration('config_file')

    # ------------------------------------------ #

    fake_odom = Node(
        package='maui',
        executable='odom_test_node',
        name='fake_odom',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        namespace='est',
        remappings=[

            # -------------------------- #       
            ('odom', 'odom'), 
            # -------------------------- #
        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(fake_odom)
    # ------------------------------------------ #

    # ------------------------------------------ #

    attitude_debug = Node(
        package='maui',
        executable='attitude_debug_node',
        name='attitude_debug',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        namespace='debug',
        remappings=[
            ('odom_est', '/est/odom'), 
            ('q_ref', '/attitude/q_ref'), 
            # -------------------------- #       
            ('pose_est', 'q_est'), 
            ('pose_ref', 'q_ref'), 
            # -------------------------- #
        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(attitude_debug)
    # ------------------------------------------ #

    # ------ Start Depth and Attitude control ---------- #
    DA_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cerebellum'),
                'launch',  
                'depth_and_attitude_controller.launch.py'
            )
        ),
        launch_arguments={
            'config_file': config_file
        }.items()
    )
    
    # Add the hardware launch at the beginning
    ld.add_action(DA_launch)

    # ------ Start path "planner" ---------- #
    circle_path_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('frontal_lobe'),
                'launch',  
                'circle_path.launch.py'
            )
        ),
        launch_arguments={
            'config_file': config_file
        }.items()
    )
    
    # Add the hardware launch at the beginning
    ld.add_action(circle_path_launch)
    

    # Run everything
    return ld