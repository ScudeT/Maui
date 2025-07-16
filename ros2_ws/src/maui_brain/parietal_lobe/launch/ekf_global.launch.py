import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # ------------------------------- #
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

    imu_to_enu = Node(
        package='parietal_lobe',
        executable='imu_change_ref',
        name='imu_to_enu',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        namespace='est',
        parameters=[os.path.join(get_package_share_directory("parietal_lobe"), 'config', 'dual_ekf.yaml')],
        remappings=[
            ("imu/data_ned","/imu/data"),
            #---------------#
            ("imu/data_enu","/imu/data_enu"),
        ]
    )

    ld.add_action(imu_to_enu)

    # ------------------------------- #
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        parameters=[os.path.join(get_package_share_directory("parietal_lobe"), 'config', 'ekf.yaml')],
        remappings=[('/myodometry/filtered', '/odometry/filtered')],
    )
    ld.add_action(ekf_node)
    
    gps_node = Node(
        package= 'parietal_lobe',
        executable = 'gps_to_local',
        name = 'gps_to_local',
    )
    
    ld.add_action(gps_node)
    
    visualization_node = Node(
        package = 'parietal_lobe',
        executable = 'local_to_gps',
        name = 'local_to_gps',
    )
    
    ld.add_action(visualization_node)
    
    return ld  