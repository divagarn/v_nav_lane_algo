import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'buggy_nav'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='divagar',
    maintainer_email='n.divagar18@gmail.com',
    description='Navigation-algo layer: TEB controller + ThetaStarPlanner/RRT* + carrot_path_publisher',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carrot_path_publisher = buggy_nav.carrot_path_publisher:main',
            'rrt_star_planner_node = buggy_nav.rrt_star_planner_node:main',
            'speed_governor = buggy_nav.speed_governor:main',
            'freespace_grid_node = buggy_nav.freespace_grid_node:main',
            'sim_ackermann_bridge = buggy_nav.sim_ackermann_bridge:main',
        ],
    },
)
