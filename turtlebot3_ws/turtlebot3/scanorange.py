#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanMinMaxNode(Node):
    def __init__(self):
        super().__init__('scan_min_max_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.get_logger().info("Subscrito ao tópico /scan para medir distâncias.")

    def scan_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            min_dist = min(valid_ranges)
            max_dist = max(valid_ranges)
            self.get_logger().info(f"Distância mínima: {min_dist:.2f} m | máxima: {max_dist:.2f} m")
        else:
            self.get_logger().warn("Sem leituras válidas no LIDAR.")

def main():
    rclpy.init()
    node = ScanMinMaxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()