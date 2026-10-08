import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'buggy_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='divagar',
    maintainer_email='n.divagar18@gmail.com',
    description='2D SLAM + 3D octomap mapping launches',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_marker_recorder = buggy_mapping.goal_marker_recorder:main',
            'pose_graph_editor = buggy_mapping.pose_graph_editor:main',
            'pose_graph_to_lanelet = buggy_mapping.pose_graph_to_lanelet:main',
            'lanelet_editor = buggy_mapping.lanelet_editor:main',
        ],
    },
)
