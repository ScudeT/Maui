import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    ld = LaunchDescription()

    default_config_arg = os.path.join( # Path to parameters
        get_package_share_directory('parietal_lobe'),
        'config', 
        'Thalamus.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value = default_config_arg,
        description='Full path to the YAML configuration file'
    )
    ld.add_action(config_arg)

    config_file = LaunchConfiguration('config_file')

    # -------------- Start Proprioception ----------------- #

    Proprioception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('parietal_lobe'),
                'launch',  # assuming the launch file is in the 'launch' folder
                'proprioception.launch.py'
            )
        )
    )
    
    # Add the hardware launch at the beginning
    ld.add_action(Proprioception_launch)

    # ------------------------------------------ #

    # -------------- Estimates necessary and initialization----------------- #
    
    simple_odom = Node(
        package='parietal_lobe',
        executable='imu_bar_odom',
        name='simple_odom',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        namespace='est',
        remappings=[
            ("/imu/data","/imu/data"),
            ("/ms5837/pose","/ms5837/pose"),
            #---------------#
            ("odom","odom")
        ]
    )
    
    ld.add_action(simple_odom)

    # ------------------------------- #

    # Run the node
    return ld
