import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    ld = LaunchDescription()
    

    params_file = os.path.join( # Path to mpu parameters
        get_package_share_directory('brain_stem'),
        'config', 
        'Thalamus.yaml'
    )
    # ------ Start the PNS ---------- #
    PNS_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('brain_stem'),
                'launch',  # assuming the launch file is in the 'launch' folder
                'peripheral_nervous_system.launch.py'
            )
        )
    )
    
    # Add the hardware launch at the beginning
    ld.add_action(PNS_launch)

    # -------------- Estimates necessary and initialization----------------- #
    
    simple_odom = Node(
        package='parietal_lobe',
        executable='imu_bar_odom',
        name='simple_odom',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
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
