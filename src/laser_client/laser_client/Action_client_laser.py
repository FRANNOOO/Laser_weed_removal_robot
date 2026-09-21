#!/usr/bin/env python3
import rclpy
from laser_control_interfaces.srv import ActivateLaser

class ActionClientLaser():
    def __init__(self, node):
        
        # Client initialization for the laser service
        self.client = node.create_client(
            ActivateLaser, 
            '/laser_control/activate_laser_timed'
        )
        self._node = node
        
        # Awaiting service availability
        while not self.client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info('Awaiting service /laser_control/activate_laser_timed...')

    def trigger_laser(self, duration_us: int):
        request = ActivateLaser.Request()
        request.timestamp = self._node.get_clock().now().to_msg()
        request.duration_us = duration_us

        self._node.get_logger().info(f'Send laser activation request for {duration_us} us...')
        future = self.client.call_async(request)

        return future

    