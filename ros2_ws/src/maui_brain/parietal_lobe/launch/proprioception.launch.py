import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    ld = LaunchDescription()

    config_file = os.path.join( # Path to parameters
        get_package_share_directory('parietal_lobe'),
        'config', 
        'Proprioception.yaml'
    )

    maui_xacro = os.path.join( # Path to parameters
        get_package_share_directory('parietal_lobe'),
        'description', 
        'maui_online.urdf.xacro'
    )
    maui_description = xacro.process_file(maui_xacro).toxml()

    # -------------- Description Publisher ----------------- #

    maui_description_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        namespace='est',
        parameters=[{'robot_description': maui_description}] # add other parameters here if required
    )

    ld.add_action(maui_description_pub)

    # ------------------------------- #

    joint_pose = Node(
        package='parietal_lobe',
        executable='joint_pose_node',
        name='joint_pose',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        namespace='est',
        parameters=[config_file],
        remappings=[
            ("command","/command"),
            #---------------#
            ("joint_states","joint_states"),
        ]
    )

    ld.add_action(joint_pose)

    # ------------------------------- #

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    ld.add_action(foxglove)

    # ------------------------------- #

    return ld
