from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'depth_camera_processing'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bonfr',
    maintainer_email='bonfr@todo.todo',
    description='ROS 2 package for RealSense D405 depth processing and weed localization.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_camera_node = depth_camera_processing.depth_camera_node:main',
        ],
    },
)
