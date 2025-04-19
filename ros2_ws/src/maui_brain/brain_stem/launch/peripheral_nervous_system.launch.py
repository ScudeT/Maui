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
        get_package_share_directory('manta_main'),
        'config', 
        'PNS.yaml'
    )

    # ------------------------------------------ #

    mpu9250_node = Node(
        package='mpu9250',
        executable='mpu9250_node',
        name='IMU',
        parameters=[params_file],
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
        parameters=[params_file],
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

    ld.add_action(gps)
    # ------------------------------------------ #
    
    depth = Node(
        package='ms5837',
        executable='pose_node',
        name='depth',
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/ms5837/pose', '/ms5837/pose'),
        ]
    )

    ld.add_action(depth)
    # ------------------------------------------ #

    button = Node(
        package='manta_main',
        executable='button_read_node',
        name='button',
        parameters=[params_file],
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
