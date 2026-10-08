from setuptools import find_packages, setup

package_name = 'buggy_uart'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='divagar',
    maintainer_email='n.divagar18@gmail.com',
    description='UART communication nodes for the real buggy',
    license='Apache-2.0',
    tests_require=['pytest'],
    scripts=[
        'scripts/check_uart.py',
        'scripts/speed_preset_test.py',
        'scripts/real_param_calibration.py',
    ],
    entry_points={
        'console_scripts': [
            'uart_bridge = buggy_uart.uart_bridge:main',
            'uart_sender_node = buggy_uart.uart_sender_node:main',
            'topic_relay_node = buggy_uart.topic_relay_node:main',
        ],
    },
)
