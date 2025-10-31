from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sensor = Node(
        package='temp_sens',
        executable='sensor_node',
        name='sensor_node',
        parameters=[{
            'rate_hz': 2.0,
            'temp_base': 22.0,
            'hum_base': 0.45
        }]
    )

    monitor = Node(
        package='temp_sens',
        executable='monitor_node',
        name='monitor_node',
        parameters=[{
            'temp_threshold': 28.0,
            'hum_threshold': 0.65,
            'alert_cooldown_s': 5.0
        }]
    )

    return LaunchDescription([sensor, monitor])
