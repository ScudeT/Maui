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
        'SpeedTest.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value = override_config_arg,
        description='Full path to the YAML configuration file'
    )
    ld.add_action(config_arg)

    config_file = LaunchConfiguration('config_file')

    # ------------------------------------------ #

    # ------ Start the PNS ---------- #
    PNS_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('brain_stem'),
                'launch',  
                'peripheral_nervous_system.launch.py'
            )
        )
    )
    
    # Add the hardware launch at the beginning
    ld.add_action(PNS_launch)

    
    # ------ Start the Thalamus ---------- #
    Thalamus_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('parietal_lobe'),
                'launch',  
                'thalamus.launch.py'
            )
        )
    )
    
    # Add the minimal state estimation launch at the beginning
    ld.add_action(Thalamus_launch)

    # ------ Start control ---------- #
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
        arguments=['--ros-args', '--log-level', 'WARN'],
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
            ('reset_controllers', 'reset_ctrl'),
        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(w_ctrl)

    # ------------------------------------------ #
    # ------ Start the command test ---------- #

    w_ref_test_node = Node(
        package='maui',
        executable='vector3_test_node',
        name='w_ref_test_node',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        namespace='attitude',
        remappings=[
            ('input', 'w_ref'),

            ('trigger', '/on_and_off'),
        ]
    )
    # Add the included launch description to your LaunchDescription
    ld.add_action(w_ref_test_node)

    w_record = Node(
        package='maui',
        executable='odom_w_plot_node',
        name='w_record',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        namespace='attitude',
        remappings=[
            ('odom', '/est/odom'),

            ('snap', '/snap'),
        ]
    )
    # Add the included launch description to your LaunchDescription
    ld.add_action(w_record)
    # ------------------------------------------ #

    # ------ Manage services, record and timeout ---------- #
    RMF_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('temporal_lobe'),
                'launch',  
                'reflex_memory_fatique.launch.py'
            )
        ),
        launch_arguments={
            'config_file': config_file
        }.items()
    )
    
    # Add the hardware launch at the beginning
    ld.add_action(RMF_launch)

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
    )
    # Add the included launch description to your LaunchDescription
    ld.add_action(foxglove)

    # Run everything
    return ld
