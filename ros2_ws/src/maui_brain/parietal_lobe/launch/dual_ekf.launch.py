from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Yaw depth control

def generate_launch_description():

    ld = LaunchDescription()
    

    config_file = os.path.join( # Path to parameters
        get_package_share_directory('parietal_lobe'),
        'config', 
        'dual_ekf_navsat.yaml'
    )

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

    # ------------------------------- #

    imu_to_enu = Node(
        package='parietal_lobe',
        executable='imu_change_ref',
        name='imu_to_enu',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        namespace='est',
        parameters=[config_file],
        remappings=[
            ("imu/data_ned","/imu/data"),
            #---------------#
            ("imu/data_enu","/imu/data_enu"),
        ]
    )

    ld.add_action(imu_to_enu)

    # ------------------------------- #

    # -------------- ekf ----------------- #
    
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'INFO'],
        namespace='est',
        remappings=[
            #---------------#
            ("odometry/filtered","ekf_odom"),
        ]
    )
    
    ld.add_action(ekf_local)

    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'INFO'],
        namespace='est',
        remappings=[
            #---------------#
            ("odometry/filtered","ekf_map"),
        ]
    )

    ld.add_action(ekf_global)

    # ------------GPS--------------- #

    gps_navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='gps_navsat',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'INFO'],
        namespace='est',
        remappings=[
            ("gps/fix","/gps/data"),
            ("imu","/imu/data_enu"),
            ("odometry/filtered","ekf_map"),
            #---------------#
            ("odometry/gps","odom_gps"),
            ("gps/filtered","gps_filtered"),
        ]
    )

    ld.add_action(gps_navsat)

    # ------------------------------- #

    return ld
