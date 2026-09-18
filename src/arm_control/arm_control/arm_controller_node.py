#!/usr/bin/env python3
"""
Arm Controller Node for Jaetrobi Robot.

This node acts as an action client to /jaetrobi_controller/follow_cartesian_trajectory,
enabling Cartesian motion commands to be sent to the Jaetrobi arm in simulation and on
the physical robot.
"""

from typing import Optional
import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node

from builtin_interfaces.msg import Duration as MsgDuration
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.msg import CartesianTrajectory, CartesianTrajectoryPoint
from jaetrobi_controller_msgs.action import FollowCartesianTrajectory

try:
    from controller_manager_msgs.srv import SwitchController, ListControllers
    HAS_CONTROLLER_MANAGER = True
except ImportError:
    HAS_CONTROLLER_MANAGER = False
    SwitchController = None
    ListControllers = None


class ArmControllerNode(Node):
    """ROS 2 Node for controlling the Cartesian position of the Jaetrobi robot arm."""

    # Workspace boundaries defined in robot_base_link frame
    WORKSPACE_MIN = (0.290, -0.070, -0.140)
    WORKSPACE_MAX = (0.400, 0.070, -0.080)
    BASE_FRAME = "robot_base_link"
    TRACKED_FRAME = "endeffector_link"

    def __init__(self, node_name: str = "arm_controller_node") -> None:
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter("target_x", 0.35)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_z", -0.10)
        self.declare_parameter("duration_sec", 2.0)
        self.declare_parameter("auto_move", False)

        self._target_x = float(self.get_parameter("target_x").value)
        self._target_y = float(self.get_parameter("target_y").value)
        self._target_z = float(self.get_parameter("target_z").value)
        self._duration_sec = float(self.get_parameter("duration_sec").value)
        self._auto_move = bool(self.get_parameter("auto_move").value)

        self._current_pose: Optional[PoseStamped] = None
        self._goal_handle: Optional[ClientGoalHandle] = None
        self._is_moving: bool = False

        # Action Client
        self._action_client = ActionClient(
            self,
            FollowCartesianTrajectory,
            "/jaetrobi_controller/follow_cartesian_trajectory",
        )

        # Controller Manager Service Clients (optional)
        if HAS_CONTROLLER_MANAGER:
            self._list_controllers_client = self.create_client(
                ListControllers,
                "/controller_manager/list_controllers",
            )
            self._switch_controller_client = self.create_client(
                SwitchController,
                "/controller_manager/switch_controller",
            )
        else:
            self._list_controllers_client = None
            self._switch_controller_client = None

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

        self.get_logger().info(
            f"Initialized {node_name}. Action: /jaetrobi_controller/follow_cartesian_trajectory"
        )
        self.get_logger().info(
            f"Workspace limits: x=[{self.WORKSPACE_MIN[0]:.3f}, {self.WORKSPACE_MAX[0]:.3f}], "
            f"y=[{self.WORKSPACE_MIN[1]:.3f}, {self.WORKSPACE_MAX[1]:.3f}], "
            f"z=[{self.WORKSPACE_MIN[2]:.3f}, {self.WORKSPACE_MAX[2]:.3f}]"
        )

        # Periodically check controller status asynchronously on startup
        if HAS_CONTROLLER_MANAGER:
            self._check_controllers_timer = self.create_timer(
                1.0, self._check_controllers_timer_callback
            )
        else:
            self._check_controllers_timer = None

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
        self.check_and_activate_controllers_async()

    def check_and_activate_controllers_async(self) -> None:
        """Query active controllers asynchronously and activate required ones if inactive."""
        if not HAS_CONTROLLER_MANAGER or self._list_controllers_client is None:
            return

        if not self._list_controllers_client.service_is_ready():
            return

        req = ListControllers.Request()
        future = self._list_controllers_client.call_async(req)
        future.add_done_callback(self._on_list_controllers_response)

    def _on_list_controllers_response(self, future) -> None:
        try:
            response: ListControllers.Response = future.result()
        except Exception as e:
            self.get_logger().debug(f"Could not list controllers: {e}")
            return

        required = ["jaetrobi_controller", "joint_trajectory_controller"]
        active_controllers = {c.name for c in response.controller if c.state == "active"}
        to_activate = [c for c in required if c not in active_controllers]

        if to_activate and self._switch_controller_client and self._switch_controller_client.service_is_ready():
            self.get_logger().info(f"Controllers {to_activate} inactive. Activating...")
            switch_req = SwitchController.Request()
            switch_req.activate_controllers = to_activate
            switch_req.deactivate_controllers = []
            switch_req.strictness = SwitchController.Request.STRICT
            switch_req.activate_asap = False
            switch_req.timeout = MsgDuration(sec=1, nanosec=0)
            switch_future = self._switch_controller_client.call_async(switch_req)
            switch_future.add_done_callback(
                lambda f: self.get_logger().info(
                    f"Controller activation result: {f.result().ok if f.result() else False}"
                )
            )

    def _auto_move_timer_callback(self) -> None:
        if self._timer:
            self._timer.cancel()
            self._timer = None
        self.move_to_position(self._target_x, self._target_y, self._target_z, self._duration_sec)

    def _current_pose_callback(self, msg: PoseStamped) -> None:
        self._current_pose = msg

    def _target_position_callback(self, msg: Point) -> None:
        self.get_logger().info(
            f"Received target position topic command: x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}"
        )
        self.move_to_position(msg.x, msg.y, msg.z, self._duration_sec)

    def _target_pose_callback(self, msg: PoseStamped) -> None:
        pos = msg.pose.position
        self.get_logger().info(
            f"Received target pose topic command: x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}"
        )
        self.move_to_position(pos.x, pos.y, pos.z, self._duration_sec)

    def is_within_workspace(self, x: float, y: float, z: float) -> bool:
        """Check if target coordinates are within the physical workspace limits."""
        return (
            self.WORKSPACE_MIN[0] <= x <= self.WORKSPACE_MAX[0]
            and self.WORKSPACE_MIN[1] <= y <= self.WORKSPACE_MAX[1]
            and self.WORKSPACE_MIN[2] <= z <= self.WORKSPACE_MAX[2]
        )

    def create_cartesian_trajectory(
        self, x: float, y: float, z: float, duration_sec: float
    ) -> CartesianTrajectory:
        """Construct a valid MoveIt CartesianTrajectory message for the given target."""
        trajectory = CartesianTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = self.BASE_FRAME
        trajectory.tracked_frame = self.TRACKED_FRAME

        point = CartesianTrajectoryPoint()
        point.point.pose.position.x = float(x)
        point.point.pose.position.y = float(y)
        point.point.pose.position.z = float(z)
        point.point.pose.orientation.x = 0.0
        point.point.pose.orientation.y = 0.0
        point.point.pose.orientation.z = 0.0
        point.point.pose.orientation.w = 1.0

        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start = MsgDuration(sec=sec, nanosec=nanosec)

        trajectory.points.append(point)
        return trajectory

    def move_to_position(
        self, x: float, y: float, z: float, duration_sec: float = 2.0
    ) -> bool:
        """
        Command the arm to move to the desired Cartesian coordinates (x, y, z).
        
        Returns True if the goal was successfully validated and sent.
        """
        if not self.is_within_workspace(x, y, z):
            self.get_logger().error(
                f"Target position ({x:.3f}, {y:.3f}, {z:.3f}) is outside robot workspace! "
                f"Allowed ranges: x=[{self.WORKSPACE_MIN[0]:.3f}, {self.WORKSPACE_MAX[0]:.3f}], "
                f"y=[{self.WORKSPACE_MIN[1]:.3f}, {self.WORKSPACE_MAX[1]:.3f}], "
                f"z=[{self.WORKSPACE_MIN[2]:.3f}, {self.WORKSPACE_MAX[2]:.3f}]"
            )
            return False

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("FollowCartesianTrajectory action server is not available!")
            return False

        trajectory = self.create_cartesian_trajectory(x, y, z, duration_sec)
        goal_msg = FollowCartesianTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.get_logger().info(
            f"Sending Cartesian trajectory goal to ({x:.4f}, {y:.4f}, {z:.4f}) over {duration_sec:.1f}s"
        )
        self._is_moving = True

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        return True

    def _goal_response_callback(self, future) -> None:
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Cartesian trajectory goal was REJECTED by action server.")
            self._is_moving = False
            return

        self.get_logger().info("Cartesian trajectory goal was ACCEPTED. Executing motion...")
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg) -> None:
        actual_pos = feedback_msg.feedback.actual.pose.position
        self.get_logger().info(
            f"Trajectory progress: current position = ({actual_pos.x:.4f}, {actual_pos.y:.4f}, {actual_pos.z:.4f})"
        )

    def _result_callback(self, future) -> None:
        result = future.result()
        status = result.status
        self._is_moving = False

        if status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info("Cartesian trajectory motion SUCCEEDED!")
        elif status == 5:  # STATUS_CANCELED
            self.get_logger().warn("Cartesian trajectory motion was CANCELED.")
        elif status == 6:  # STATUS_ABORTED
            self.get_logger().error(f"Cartesian trajectory motion ABORTED: {result.result.msg}")
        else:
            self.get_logger().warn(f"Cartesian trajectory finished with status: {status}")

    def cancel_current_goal(self) -> None:
        """Cancel the active trajectory goal if one is running."""
        if self._goal_handle and self._is_moving:
            self.get_logger().info("Requesting cancellation of current goal...")
            self._goal_handle.cancel_goal_async()


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
