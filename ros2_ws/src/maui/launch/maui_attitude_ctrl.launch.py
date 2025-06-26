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
        'MauiAttitudeCtrl.yaml'
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
                'launch',  # assuming the launch file is in the 'launch' folder
                'peripheral_nervous_system.launch.py'
            )
        ),
        launch_arguments={
            'config_file': config_file
        }.items()
    )
    
    # Add the hardware launch at the beginning
    ld.add_action(PNS_launch)

    # ------------------------------------------ #

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