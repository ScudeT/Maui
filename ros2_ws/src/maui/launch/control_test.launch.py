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
        'ControlTest.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value = override_config_arg,
        description='Full path to the YAML configuration file'
    )
    ld.add_action(config_arg)

    config_file = LaunchConfiguration('config_file')

    # ------------------------------------------ #
    # ------ Start the command test ---------- #

    command_test_node = Node(
        package='maui',
        executable='comand_test_node',
        name='command_test_node',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/input1', '/A_m'),
            ('/input2', '/A_d'),
            ('/input3', '/alpha_m'),
            ('/input4', '/alpha_d'),
            ('/input5', '/theta_m'),
            ('/input6', '/theta_d'),

            ('/trigger', '/on_and_off'),
        ]
    )
    # Add the included launch description to your LaunchDescription
    ld.add_action(command_test_node)
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
    
    # Add the hardware launch at the beginning
    ld.add_action(Thalamus_launch)

    # ------ Start testing controller ---------- #
    BMF_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cerebellum'),
                'launch',  
                'basic_motor_functions.launch.py'
            )
        ),
        launch_arguments={
            'config_file': config_file
        }.items()
    )
    
    # Add the hardware launch at the beginning
    ld.add_action(BMF_launch)

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

    # Run everything
    return ld
