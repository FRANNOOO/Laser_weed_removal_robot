#!/usr/bin/env python3
"""
Arm Controller Node for Jaetrobi Robot.

This node provides ROS 2 parameter and topic interfaces (~/target_position, ~/target_pose)
and uses CartesianActionClient to execute Cartesian motions on the Jaetrobi arm.
"""

from typing import Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped

from arm_control.action_client import CartesianActionClient


class ArmControllerNode(Node):
    """ROS 2 Node for controlling the Cartesian position of the Jaetrobi robot arm."""

    def __init__(self, node_name: str = "arm_controller_node") -> None:
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter("target_x", 0.35)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_z", -0.10)
        self.declare_parameter("duration_sec", 2.0)
        self.declare_parameter("auto_move", False)
        self.declare_parameter("enforce_workspace_limits", True)
        self.declare_parameter("workspace_min_x", 0.290)
        self.declare_parameter("workspace_max_x", 0.400)
        self.declare_parameter("workspace_min_y", -0.070)
        self.declare_parameter("workspace_max_y", 0.070)
        self.declare_parameter("workspace_min_z", -0.140)
        self.declare_parameter("workspace_max_z", -0.080)

        self._target_x = float(self.get_parameter("target_x").value)
        self._target_y = float(self.get_parameter("target_y").value)
        self._target_z = float(self.get_parameter("target_z").value)
        self._duration_sec = float(self.get_parameter("duration_sec").value)
        self._auto_move = bool(self.get_parameter("auto_move").value)
        enforce_limits = bool(self.get_parameter("enforce_workspace_limits").value)

        ws_min = (
            float(self.get_parameter("workspace_min_x").value),
            float(self.get_parameter("workspace_min_y").value),
            float(self.get_parameter("workspace_min_z").value),
        )
        ws_max = (
            float(self.get_parameter("workspace_max_x").value),
            float(self.get_parameter("workspace_max_y").value),
            float(self.get_parameter("workspace_max_z").value),
        )

        self._current_pose: Optional[PoseStamped] = None

        # Instantiate Action Client
        self.action_client = CartesianActionClient(
            self,
            workspace_min=ws_min,
            workspace_max=ws_max,
            enforce_workspace_limits=enforce_limits,
        )

        # Subscriptions
        self._current_pose_sub = self.create_subscription(
            PoseStamped,
            "/jaetrobi_controller/current_pose",
            self._current_pose_callback,
            10,
        )
        self._target_pos_sub = self.create_subscription(
            Point,
            "~/target_position",
            self._target_position_callback,
            10,
        )
        self._target_pose_sub = self.create_subscription(
            PoseStamped,
            "~/target_pose",
            self._target_pose_callback,
            10,
        )

        self.get_logger().info(f"Initialized {node_name}.")

        # Check and activate controllers if needed
        self._check_controllers_timer = self.create_timer(
            1.0, self._check_controllers_timer_callback
        )

        # If auto_move is set, trigger initial movement via a one-shot timer
        if self._auto_move:
            self.get_logger().info(
                f"Auto move enabled: moving to ({self._target_x:.3f}, {self._target_y:.3f}, {self._target_z:.3f}) in {self._duration_sec:.1f}s"
            )
            self._timer = self.create_timer(1.5, self._auto_move_timer_callback)
        else:
            self._timer = None

    def _check_controllers_timer_callback(self) -> None:
        if self._check_controllers_timer:
            self._check_controllers_timer.cancel()
            self._check_controllers_timer = None
        self.action_client.check_and_activate_controllers_async()

    def _auto_move_timer_callback(self) -> None:
        if self._timer:
            self._timer.cancel()
            self._timer = None
        self.action_client.move_to_position(
            self._target_x, self._target_y, self._target_z, self._duration_sec
        )

    def _current_pose_callback(self, msg: PoseStamped) -> None:
        self._current_pose = msg

    def _target_position_callback(self, msg: Point) -> None:
        self.get_logger().info(
            f"Received target position topic command: x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}"
        )
        self.action_client.move_to_position(msg.x, msg.y, msg.z, self._duration_sec)

    def _target_pose_callback(self, msg: PoseStamped) -> None:
        pos = msg.pose.position
        self.get_logger().info(
            f"Received target pose topic command: x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}"
        )
        self.action_client.move_to_position(pos.x, pos.y, pos.z, self._duration_sec)


def main(args=None) -> None:
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        if "rcl_shutdown already called" not in str(e) and "context is not valid" not in str(e):
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
