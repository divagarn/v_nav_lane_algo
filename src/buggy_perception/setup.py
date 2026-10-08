from setuptools import find_packages, setup

package_name = 'buggy_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='divagar',
    maintainer_email='n.divagar18@gmail.com',
    description='Ground segmentation + self-hit filtering + tf_odom_relay + real-hardware TF/odometry/actuation bridges',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'segment_ground = buggy_perception.segment_ground:main',
            'self_hit_filter = buggy_perception.self_hit_filter:main',
            'tf_odom_relay = buggy_perception.tf_odom_relay:main',
            'initialpose_to_slam_toolbox = buggy_perception.initialpose_to_slam_toolbox:main',
            'velodyne_static_tf = buggy_perception.velodyne_static_tf:main',
            'wheel_odometry = buggy_perception.wheel_odometry:main',
            'steering_uart_bridge = buggy_perception.steering_uart_bridge:main',
            'manual_pose_aligner = buggy_perception.manual_pose_aligner:main',
            'steering_calculator_node = buggy_perception.steering_calculator_node:main',
            'pose_graph_turn_direction_node = buggy_perception.pose_graph_turn_direction_node:main',
            'freespace_uart_bridge = buggy_perception.freespace_uart_bridge:main',
            'camera_static_tf = buggy_perception.camera_static_tf:main',
            'camera_lidar_fusion_node = buggy_perception.camera_lidar_fusion_node:main',
        ],
    },
)
