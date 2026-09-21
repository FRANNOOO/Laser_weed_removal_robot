#!/usr/bin/env python3
"""Client wrapper for activating the weed removal laser service."""

from typing import Optional

from laser_control_interfaces.srv import ActivateLaser
from rclpy.node import Node
from rclpy.task import Future


class ActionClientLaser:
    """Service client wrapper for /laser_control/activate_laser_timed."""

    def __init__(
        self,
        node: Node,
        service_name: str = '/laser_control/activate_laser_timed',
        wait_on_init: bool = False,
        timeout_sec: float = 2.0,
    ) -> None:
        """Initialize the laser service client."""
        self._node = node
        self._service_name = service_name
        self._client = node.create_client(ActivateLaser, service_name)

        if wait_on_init:
            self.wait_for_service(timeout_sec=timeout_sec)

    @property
    def service_name(self) -> str:
        """Return the target service name."""
        return self._service_name

    def wait_for_service(self, timeout_sec: float = 5.0) -> bool:
        """Wait for the laser service to become available."""
        ready = self._client.wait_for_service(timeout_sec=timeout_sec)
        if not ready:
            self._node.get_logger().warn(
                f'Laser service {self._service_name} not available after {timeout_sec:.1f}s'
            )
        return ready

    def is_service_ready(self) -> bool:
        """Check if laser service is immediately ready."""
        return self._client.service_is_ready()

    def trigger_laser(self, duration_us: int) -> Optional[Future]:
        """
        Send a timed laser activation request.

        :param duration_us: Duration to fire the laser in microseconds.
        :return: Future object for the service response, or None if service unavailable.
        """
        if not self._client.service_is_ready() and not self.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().error(
                f'Cannot trigger laser: {self._service_name} is unavailable.'
            )
            return None

        request = ActivateLaser.Request()
        request.timestamp = self._node.get_clock().now().to_msg()
        request.duration_us = int(duration_us)

        self._node.get_logger().info(
            f'Sending laser activation request for {duration_us} us...'
        )
        return self._client.call_async(request)
