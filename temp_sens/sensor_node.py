#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from temp_sens.msg import SensorData
import random

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('temp_base', 22.0)
        self.declare_parameter('hum_base', 0.45)

        self.rate_hz = self.get_parameter('rate_hz').value
        self.temp = self.get_parameter('temp_base').value
        self.hum = self.get_parameter('hum_base').value

        self.publisher = self.create_publisher(SensorData, 'sensor_data', 10)
        timer_period = 1.0 / max(self.rate_hz, 0.001)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f'Sensor node started, publishing at {self.rate_hz} Hz')

    def timer_callback(self):
        self.temp += random.uniform(-0.3, 0.3)
        self.hum += random.uniform(-0.01, 0.01)
        self.hum = max(0.0, min(1.0, self.hum))

        msg = SensorData()
        msg.temperature = float(self.temp)
        msg.humidity = float(self.hum)

        self.publisher.publish(msg)
        self.get_logger().debug(f'Published SensorData: temp={msg.temperature:.2f}, hum={msg.humidity:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
