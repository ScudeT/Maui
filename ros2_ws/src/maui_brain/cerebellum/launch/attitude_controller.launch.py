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
        'AttitudeController.yaml'
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
    BMF_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cerebellum'),
                'launch',  
                'basic_motor_functions.launch.py'
            )
        ),
    )
    
    # Add the hardware launch at the beginning
    ld.add_action(BMF_launch)

    # ------------------------------------------ #

    w_ctrl = Node(
        package='cerebellum',
        executable='w_ctrl_node',
        name='w_ctrl',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'INFO'],
        output='screen',
        namespace='attitude',
        remappings=[
           ('measure', '/est/odom'), 
           ('setpoint', 'w_ref'), 
            
            # -------------------------- #       
            ('wx_act', '/input/alpha_d'), 
            ('wy_act', '/input/theta_m'),
            ('wz_act', '/input/A_d',   ),
            # -------------------------- #
            ('reset_controllers', 'reset_w_ctrl'),
        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(w_ctrl)
    # ------------------------------------------ #

    # ------------------------------------------ #

    q_ctrl = Node(
        package='cerebellum',
        executable='q_ctrl_node',
        name='q_ctrl',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        namespace='attitude',
        remappings=[
            ('measure', '/est/odom'), 
            ('setpoint', 'q_ref'), 
            
            # -------------------------- #       
            ('w_set', 'w_ref'), 
            # -------------------------- #
        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(q_ctrl)
    # ------------------------------------------ #
    # ------------------------------------------ #

    ypr2q = Node(
        package='cerebellum',
        executable='ypr2q_node',
        name='ypr_to_q',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        namespace='attitude',
        remappings=[
            ('roll', 'roll_set'), 
            ('pitch', 'pitch_set'), 
            ('yaw', 'yaw_set'), 
                
            # -------------------------- #       
            ('q', 'q_ref'), 
            # -------------------------- #


        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(ypr2q)
    # ------------------------------------------ #
    

    # Run everything
    return ld