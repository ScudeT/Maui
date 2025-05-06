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
        get_package_share_directory('cerebellum'),
        'config', 
        'BMF.yaml'
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
            ('/omega_m', '/omega_m'),
            ('/A_m',     '/A_m',   ),            
            ('/A_d',     '/A_d',   ),
            ('/alpha_d', '/alpha_d'),
            
            ('/theta_m', '/theta_m'),
            ('/theta_d', '/theta_d'),           
            
            # -------------------------- #
            ('/command', '/command'),
            # -------------------------- #

            ('/start_and_stop', '/start_and_stop'),
        ]
    )

    # Add the included launch description to your LaunchDescription
    ld.add_action(ctrl)
    # ------------------------------------------ #

    return ld