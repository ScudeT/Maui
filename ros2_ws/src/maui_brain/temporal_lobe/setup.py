from setuptools import find_packages, setup

package_name = 'temporal_lobe'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
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
