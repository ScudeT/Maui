import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    ld = LaunchDescription()

    config_file = os.path.join( # Path to parameters
        get_package_share_directory('brain_stem'),
        'config', 
        'PNS.yaml'
    )

    # ------------------------------------------ #

    mpu9250_node = Node(
        package='mpu9250',
        executable='mpu9250_node',
        name='mpu9250',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/imu/data', '/imu/data'),
            ('/imu/mag', '/imu/mag_raw'),
        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(mpu9250_node)
    # ------------------------------------------ #

    pca9685_node = Node(
        package='pwm_pca9685',
        executable='pca9685_node',
        name='pca9685',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            
        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(pca9685_node)

    # ------------------------------------------ #
    


    gps = Node(
        package='gtu7_gps_comm',
        executable='gps_node',
        name='GPS',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        remappings=[
            ('/gps_data', '/gps_data')
        ]
    )

    #ld.add_action(gps)
    # ------------------------------------------ #
    
    ms5837 = Node(
        package='ms5837',
        executable='pose_node',
        name='ms5837',
	parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/ms5837/pose', '/ms5837/pose'),
        ]
    )

    ld.add_action(ms5837)
    # ------------------------------------------ #

    button = Node(
        package='temporal_lobe',
        executable='button_read_node',
        name='button',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/button_state', '/button_state'),
        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(button)

    # Run the node
    return ld
