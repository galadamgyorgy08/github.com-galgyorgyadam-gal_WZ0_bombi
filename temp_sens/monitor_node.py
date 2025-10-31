#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from temp_sens.msg import SensorData
from std_msgs.msg import String
import time

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        self.declare_parameter('temp_threshold', 30.0)
        self.declare_parameter('hum_threshold', 0.70)
        self.declare_parameter('alert_cooldown_s', 10.0)

        self.temp_threshold = self.get_parameter('temp_threshold').value
        self.hum_threshold = self.get_parameter('hum_threshold').value
        self.cooldown = self.get_parameter('alert_cooldown_s').value

        self.last_alert_time = 0.0

        self.alert_pub = self.create_publisher(String, 'sensor_alert', 10)
        self.sub = self.create_subscription(SensorData, 'sensor_data', self.data_callback, 10)

        self.get_logger().info(f'Monitor started (temp_th={self.temp_threshold}, hum_th={self.hum_threshold})')

    def data_callback(self, msg):
        now = time.time()
        if now - self.last_alert_time < self.cooldown:
            return

        alerts = []
        if msg.temperature > self.temp_threshold:
            alerts.append(f'TEMP HIGH: {msg.temperature:.2f} > {self.temp_threshold}')
        if msg.humidity > self.hum_threshold:
            alerts.append(f'HUM HIGH: {msg.humidity:.3f} > {self.hum_threshold}')

        if alerts:
            alert_msg = String()
            alert_msg.data = '; '.join(alerts)
            self.alert_pub.publish(alert_msg)
            self.get_logger().warn(f'Alert: {alert_msg.data}')
            self.last_alert_time = now


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
