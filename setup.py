from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'graph_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name), ['wireless_data_completeLab.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AntoArroyo',
    maintainer_email='arroyo.antom@gmail.com',
    description='Package for publishing map and wireless data as pointcloud',
    license='GNU GENERAL PUBLIC LICENSE Version 3',
   # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_publisher = graph_pub.map_publisher:main',
            'wireless_data_publisher = graph_pub.wireless_data_publisher:main',
        ],
    },
)
