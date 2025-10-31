from setuptools import setup

package_name = 'temp_sens'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sensors_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Temperature and humidity sensor system with custom message (ROS2 Humble)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sensor_node = temp_sens.sensor_node:main',
            'monitor_node = temp_sens.monitor_node:main',
        ],
    },
)
