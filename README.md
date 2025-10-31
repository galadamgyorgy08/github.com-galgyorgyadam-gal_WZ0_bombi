# temp_sens

Custom ROS2 Humble package with a simulated temperature and humidity sensor using a custom message type.

## Message
**msg/SensorData.msg**
```
float64 temperature
float64 humidity
```

## Topics
- `/sensor_data` (`temp_sens/msg/SensorData`) — published by `sensor_node`
- `/sensor_alert` (`std_msgs/String`) — published by `monitor_node`

## Build
```bash
colcon build --packages-select temp_sens
source install/setup.bash
```

## Run
```bash
ros2 launch temp_sens sensors_launch.py
```

## Test
```bash
ros2 topic echo /sensor_data
ros2 topic echo /sensor_alert
```
