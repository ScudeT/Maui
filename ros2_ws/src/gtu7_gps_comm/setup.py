from setuptools import find_packages, setup

package_name = 'gtu7_gps_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','gpsd-py3'],
    zip_safe=True,
    maintainer='ray',
    maintainer_email='tommaso.scudeletti@mail.polimi.it',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_node = gtu7_gps_comm.gps_node:main'
        ],
    },
)
