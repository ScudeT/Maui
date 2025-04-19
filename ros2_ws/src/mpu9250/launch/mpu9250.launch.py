import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Paths to parameter files
    pkg_share = get_package_share_directory('mpu9250')
    mpu_params_file = os.path.join(pkg_share, 'config', 'mpu9250_params.yaml')

    mpu9250_node = Node(
        package='mpu9250',
        executable='mpu9250_node',
        name='IMU',
        parameters=[mpu_params_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/imu/data', '/imu/data'),
            ('/imu/mag', '/imu/mag_raw'),
        ]
    )

    # Launch Description
    ld = LaunchDescription()

    # Add actions
    ld.add_action(mpu9250_node)

    return ld
