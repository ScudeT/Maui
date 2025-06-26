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
        'TestAttitudeCtrl.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value = override_config_arg,
        description='Full path to the YAML configuration file'
    )
    ld.add_action(config_arg)

    config_file = LaunchConfiguration('config_file')

    # ------------------------------------------ #

    attitude_ctrl = Node(
        package='cerebellum',
        executable='attitude_ctrl_node',
        name='ctrl',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'INFO'],
        output='screen',
        namespace='attitude',
        remappings=[
           ('measure', '/est/odom'), 
           ('setpoint', 'q_ref'), 
            
            # -------------------------- #       
            ('wx_act', '/input/alpha_d'), 
            ('wy_act', '/input/theta_m'),
            ('wz_act', '/input/A_d',   ),
            # -------------------------- #
            
            ('reset_controllers', 'reset_ctrl'),
        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(attitude_ctrl)
    # ------------------------------------------ #
    # ------------------------------------------ #

    q_test = Node(
        package='maui',
        executable='attitude_test_node',
        name='attitude_test_node',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[

            # -------------------------- #       
            ('odom', '/est/odom'), 
            ('q_ref', '/attitude/q_ref'), 
            # -------------------------- #
        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(q_test)
    # ------------------------------------------ #
    

    # Run everything
    return ld