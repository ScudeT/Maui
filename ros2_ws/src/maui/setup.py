from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'maui'

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files(data_files, ['launch', 'config']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='95103311+ScudeT@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'comand_test_node = maui.comand_test_node:main',
            'vector3_test_node = maui.vector3_test_node:main',
            'attitude_test_node = maui.attitude_test_node:main',
            'attitude_debug_node = maui.attitude_debug_node:main',
            'odom_test_node = maui.odom_test_node:main',
            'odom_w_plot_node = maui.odom_w_plot_node:main',
        ],
    },
)
