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
        get_package_share_directory('pwm_pca9685'),
        'config', 
        'Test.yaml'
    )

    # ------------------------------------------ #

    ctrl = Node(
        package='pwm_pca9685',
        executable='ctrl_node',
        name='ctrl',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/wing_freq', '/wing_freq'),
            ('/wing_Amp', '/wing_Amp'),
            ('/wing_diff', '/wing_diff'),
            ('/flap_same', '/flap_same'),
            ('/flap_dif', '/flap_dif'),
            # -------------------------- #
            ('/command', '/command'),
            # -------------------------- #
            ('/start_and_stop', '/start_and_stop'),
        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(ctrl)
    # ------------------------------------------ #

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

    return ld
