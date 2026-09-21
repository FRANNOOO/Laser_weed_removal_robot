#!/usr/bin/env python3
import sys
import rclpy
from laser_client.Action_client_laser import ActionClientLaser
from rclpy.node import Node


class LaserNode(Node):
    def __init__(self):
        super().__init__('action_client_laser_node')

        # 1. Initialization
        self.laser = ActionClientLaser(self)

    def run(self, duration_us: float):
        # 2. Execution of action
        future = self.laser.trigger_laser(duration_us)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            self.get_logger().info(f'OUTCOME: {response.message}')
        else:
            self.get_logger().error(f'ERROR: {response.message}')
        

def main(args=None):
    rclpy.init(args=args)

    duration_us = 500000  # Default: 0.5s
    if len(sys.argv) > 1:
        try:
            duration_us = int(sys.argv[1])
        except ValueError:
            pass

    node = LaserNode()
    node.run(duration_us)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()