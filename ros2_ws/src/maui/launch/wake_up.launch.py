import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    ld = LaunchDescription()


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

    
    # ------ Start the Thalamus ---------- #
    Thalamus_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('parietal_lobe'),
                'launch',  # assuming the launch file is in the 'launch' folder
                'thalamus.launch.py'
            )
        )
    )
    
    # Add the hardware launch at the beginning
    ld.add_action(Thalamus_launch)

    # ------ Start basic motor functions ---------- #
    BMF_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cerebellum'),
                'launch',  # assuming the launch file is in the 'launch' folder
                'basic_motor_functions.launch.py'
            )
        )
    )
    
    # Add the hardware launch at the beginning
    ld.add_action(BMF_launch)

    # Run everything
    return ld
