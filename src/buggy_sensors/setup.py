import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'buggy_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='divagar',
    maintainer_email='n.divagar18@gmail.com',
    description='Sensor-side nodes for the fresh buggy workspace (radar_sim UMRR-A4 stand-in)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radar_sim = buggy_sensors.radar_sim:main',
            'fake_pedestrian_mover = buggy_sensors.fake_pedestrian_mover:main',
        ],
    },
)
