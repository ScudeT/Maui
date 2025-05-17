from setuptools import find_packages, setup
import os


package_name = 'temporal_lobe'

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
            'titan_control = temporal_lobe.titan_control:main',
            'button_read_node = temporal_lobe.button_read_node:main',
            'button_service_caller_node = temporal_lobe.button_service_caller_node:main',
            'depth_service_caller_node = temporal_lobe.depth_service_caller_node:main',
            'record_service_node=temporal_lobe.record_service_node:main',
            'button_timeout_node = temporal_lobe.button_timeout_node:main',
            'depth_setter_node = temporal_lobe.depth_setter_node:main'
        ],
    },
)
