#!/usr/bin/env python3
"""
Action Client for Jaetrobi Follow Cartesian Trajectory Action.

Encapsulates logic for validating workspace boundaries, creating MoveIt
Cartesian trajectories, communicating with the action server, and ensuring
the necessary controllers are active.
"""

from typing import Callable, Optional, Tuple

from builtin_interfaces.msg import Duration as MsgDuration
try:
    from jaetrobi_controller_msgs.action import FollowCartesianTrajectory
    HAS_JAETROBI_MSGS = True
except ImportError:
    HAS_JAETROBI_MSGS = False
    FollowCartesianTrajectory = None

try:
    from moveit_msgs.msg import CartesianTrajectory, CartesianTrajectoryPoint
    HAS_MOVEIT_MSGS = True
except ImportError:
    HAS_MOVEIT_MSGS = False
    CartesianTrajectory = None
    CartesianTrajectoryPoint = None
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future

try:
    from controller_manager_msgs.srv import ListControllers, SwitchController
    HAS_CONTROLLER_MANAGER = True
except ImportError:
    HAS_CONTROLLER_MANAGER = False
    SwitchController = None
    ListControllers = None


class CartesianActionClient:
    """Action client wrapper for /jaetrobi_controller/follow_cartesian_trajectory."""

    WORKSPACE_MIN: Tuple[float, float, float] = (0.290, -0.070, -0.140)
    WORKSPACE_MAX: Tuple[float, float, float] = (0.400, 0.070, -0.080)
    BASE_FRAME: str = 'robot_base_link'
    TRACKED_FRAME: str = 'endeffector_link'

    def __init__(
        self,
        node: Node,
        action_name: str = '/jaetrobi_controller/follow_cartesian_trajectory',
        workspace_min: Optional[Tuple[float, float, float]] = None,
        workspace_max: Optional[Tuple[float, float, float]] = None,
        enforce_workspace_limits: bool = True,
    ) -> None:
        """Initialize CartesianActionClient with node, workspace limits, and action client."""
        self._node = node
        self._action_name = action_name
        self._logger = node.get_logger()

        self.workspace_min = workspace_min if workspace_min is not None else self.WORKSPACE_MIN
        self.workspace_max = workspace_max if workspace_max is not None else self.WORKSPACE_MAX
        self.enforce_workspace_limits = enforce_workspace_limits

        self._goal_handle: Optional[ClientGoalHandle] = None
        self._is_moving: bool = False
        self._controllers_ready: bool = False

        if HAS_JAETROBI_MSGS and FollowCartesianTrajectory is not None:
            self._client: Optional[ActionClient] = ActionClient(
                node,
                FollowCartesianTrajectory,
                action_name,
            )
        else:
            self._client = None
            self._logger.warn(
                f'Action {action_name} msg interface not available; client in mock/offline mode.'
            )

        if HAS_CONTROLLER_MANAGER:
            self._list_controllers_client = node.create_client(
                ListControllers,
                '/controller_manager/list_controllers',
            )
            self._switch_controller_client = node.create_client(
                SwitchController,
                '/controller_manager/switch_controller',
            )
        else:
            self._list_controllers_client = None
            self._switch_controller_client = None

        self._logger.info(
            f'CartesianActionClient initialized for action: {action_name}'
        )
        self._logger.info(
            f'Workspace limits (enforce={self.enforce_workspace_limits}): '
            f'x=[{self.workspace_min[0]:.3f}, {self.workspace_max[0]:.3f}], '
            f'y=[{self.workspace_min[1]:.3f}, {self.workspace_max[1]:.3f}], '
            f'z=[{self.workspace_min[2]:.3f}, {self.workspace_max[2]:.3f}]'
        )

    @property
    def is_controllers_ready(self) -> bool:
        """Return True if required controllers are confirmed active."""
        return self._controllers_ready

    @property
    def is_moving(self) -> bool:
        """Return True if a trajectory motion is currently in progress."""
        return self._is_moving

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        """Wait for the action server to become available."""
        if self._client is None:
            return False
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def check_workspace(self, x: float, y: float, z: float) -> Tuple[bool, str]:
        """
        Check if target coordinates are within physical workspace limits.

        Returns (is_valid, violation_detail).
        """
        if not self.enforce_workspace_limits:
            return True, ''

        min_x, min_y, min_z = self.workspace_min
        max_x, max_y, max_z = self.workspace_max

        violations = []
        if not (min_x <= x <= max_x):
            violations.append(f'x={x:.4f} outside [{min_x:.3f}, {max_x:.3f}]')
        if not (min_y <= y <= max_y):
            violations.append(f'y={y:.4f} outside [{min_y:.3f}, {max_y:.3f}]')
        if not (min_z <= z <= max_z):
            tag = '> max' if z > max_z else '< min'
            violations.append(
                f'z={z:.4f} outside [{min_z:.3f}, {max_z:.3f}] ({tag})'
            )

        if violations:
            return False, '; '.join(violations)
        return True, ''

    def is_within_workspace(self, x: float, y: float, z: float) -> bool:
        """Check if target coordinates are within physical workspace limits."""
        valid, _ = self.check_workspace(x, y, z)
        return valid

    def create_cartesian_trajectory(
        self, x: float, y: float, z: float, duration_sec: float
    ) -> Optional[CartesianTrajectory]:
        """Construct a valid MoveIt CartesianTrajectory message for the given target."""
        if not HAS_MOVEIT_MSGS or CartesianTrajectory is None or CartesianTrajectoryPoint is None:
            return None
        trajectory = CartesianTrajectory()
        trajectory.header.stamp = self._node.get_clock().now().to_msg()
        trajectory.header.frame_id = self.BASE_FRAME
        trajectory.tracked_frame = self.TRACKED_FRAME

        # Clamp strictly within physical limits to guard against floating-point roundoff
        ws_min = self.workspace_min
        ws_max = self.workspace_max
        clamped_x = min(max(float(x), ws_min[0]), ws_max[0])
        clamped_y = min(max(float(y), ws_min[1]), ws_max[1])
        clamped_z = min(max(float(z), ws_min[2]), ws_max[2])

        point = CartesianTrajectoryPoint()
        point.point.pose.position.x = clamped_x
        point.point.pose.position.y = clamped_y
        point.point.pose.position.z = clamped_z
        point.point.pose.orientation.x = 0.0
        point.point.pose.orientation.y = 0.0
        point.point.pose.orientation.z = 0.0
        point.point.pose.orientation.w = 1.0

        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start = MsgDuration(sec=sec, nanosec=nanosec)

        trajectory.points.append(point)
        return trajectory

    def check_and_activate_controllers_async(self) -> None:
        """Query active controllers asynchronously and activate required ones if inactive."""
        if not HAS_CONTROLLER_MANAGER or self._list_controllers_client is None:
            return

        def _do_query() -> None:
            req = ListControllers.Request()
            future = self._list_controllers_client.call_async(req)
            future.add_done_callback(self._on_list_controllers_response)

        if self._list_controllers_client.service_is_ready():
            _do_query()
        else:
            self._logger.info(
                'Waiting for /controller_manager/list_controllers service...'
            )
            retry_timer = None

            def _check_and_call() -> None:
                nonlocal retry_timer
                if self._list_controllers_client.service_is_ready():
                    if retry_timer is not None:
                        retry_timer.cancel()
                        retry_timer = None
                    _do_query()

            retry_timer = self._node.create_timer(1.0, _check_and_call)

    def _on_list_controllers_response(self, future: Future) -> None:
        try:
            response = future.result()
        except Exception as e:
            self._logger.debug(f'Could not list controllers: {e}')
            return

        required = [
            'joint_state_broadcaster',
            'jaetrobi_controller',
            'joint_trajectory_controller',
        ]
        active_controllers = {
            c.name for c in response.controller if c.state == 'active'
        }
        to_activate = [c for c in required if c not in active_controllers]

        if not to_activate:
            self._logger.info('All required arm controllers are active.')
            self._controllers_ready = True
            return

        def _do_switch() -> None:
            self._logger.info(
                f'Controllers {to_activate} inactive. Activating...'
            )
            switch_req = SwitchController.Request()
            switch_req.activate_controllers = to_activate
            switch_req.deactivate_controllers = []
            switch_req.strictness = SwitchController.Request.BEST_EFFORT
            switch_req.activate_asap = True
            switch_req.timeout = MsgDuration(sec=1, nanosec=0)
            switch_future = self._switch_controller_client.call_async(
                switch_req
            )

            def _on_switch_done(f: Future) -> None:
                try:
                    res = f.result()
                    ok = res.ok if res else False
                    self._logger.info(f'Controller activation result: {ok}')
                    if ok:
                        self._controllers_ready = True
                except Exception as ex:
                    self._logger.error(f'SwitchController call failed: {ex}')

            switch_future.add_done_callback(_on_switch_done)

        if (
            self._switch_controller_client and
            self._switch_controller_client.service_is_ready()
        ):
            _do_switch()
        elif self._switch_controller_client:
            self._logger.info(
                'Waiting for /controller_manager/switch_controller service...'
            )
            switch_retry_timer = None

            def _check_switch_and_call() -> None:
                nonlocal switch_retry_timer
                if self._switch_controller_client.service_is_ready():
                    if switch_retry_timer is not None:
                        switch_retry_timer.cancel()
                        switch_retry_timer = None
                    _do_switch()

            switch_retry_timer = self._node.create_timer(1.0, _check_switch_and_call)

    def move_to_position(
        self,
        x: float,
        y: float,
        z: float,
        duration_sec: float = 2.0,
        feedback_callback: Optional[Callable] = None,
        result_callback: Optional[Callable] = None,
    ) -> bool:
        """
        Send a Cartesian movement command to the arm.

        :param x: Target X coordinate in meters.
        :param y: Target Y coordinate in meters.
        :param z: Target Z coordinate in meters.
        :param duration_sec: Motion execution duration in seconds.
        :param feedback_callback: Optional callback for feedback messages.
        :param result_callback: Optional callback when goal finishes.
        :return: True if goal was validated and sent, False otherwise.
        """
        if not self._controllers_ready:
            self.check_and_activate_controllers_async()

        valid, reason = self.check_workspace(x, y, z)
        if not valid:
            self._logger.error(
                f'Target position ({x:.3f}, {y:.3f}, {z:.3f}) is outside robot workspace! '
                f'Violation: {reason}. '
                f'Allowed ranges: x=[{self.workspace_min[0]:.3f}, {self.workspace_max[0]:.3f}], '
                f'y=[{self.workspace_min[1]:.3f}, {self.workspace_max[1]:.3f}], '
                f'z=[{self.workspace_min[2]:.3f}, {self.workspace_max[2]:.3f}]'
            )
            return False

        if not self.wait_for_server(timeout_sec=5.0):
            self._logger.error('FollowCartesianTrajectory action server is not available!')
            return False

        trajectory = self.create_cartesian_trajectory(x, y, z, duration_sec)
        if trajectory is None or FollowCartesianTrajectory is None:
            self._logger.error('Cannot construct FollowCartesianTrajectory goal.')
            return False

        goal_msg = FollowCartesianTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self._logger.info(
            f'Sending Cartesian trajectory goal to ({x:.4f}, {y:.4f}, {z:.4f}) '
            f'over {duration_sec:.1f}s'
        )
        self._is_moving = True

        def internal_feedback(feedback_msg):
            pos = feedback_msg.feedback.actual.pose.position
            self._logger.debug(
                f'Trajectory progress: current position = '
                f'({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f})'
            )
            if feedback_callback:
                feedback_callback(feedback_msg)

        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=internal_feedback,
        )

        def internal_goal_response(future: Future):
            goal_handle: Optional[ClientGoalHandle] = future.result()
            if not goal_handle or not goal_handle.accepted:
                self._logger.error('Cartesian trajectory goal was REJECTED by action server.')
                self._controllers_ready = False
                self.check_and_activate_controllers_async()
                self._is_moving = False
                if result_callback:
                    result_callback(None)
                return

            self._logger.info('Cartesian trajectory goal was ACCEPTED. Executing motion...')
            self._goal_handle = goal_handle
            res_future = goal_handle.get_result_async()

            def internal_result(res_f: Future):
                self._is_moving = False
                res = res_f.result()
                status = res.status if res else None
                if status == 4:
                    self._logger.info('Cartesian trajectory motion SUCCEEDED!')
                elif status == 5:
                    self._logger.warn('Cartesian trajectory motion was CANCELED.')
                elif status == 6:
                    msg_text = res.result.msg if res and res.result else 'aborted'
                    self._logger.error(f'Cartesian trajectory motion ABORTED: {msg_text}')
                    self._controllers_ready = False
                    self.check_and_activate_controllers_async()
                else:
                    self._logger.warn(f'Cartesian trajectory finished with status: {status}')

                if result_callback:
                    result_callback(res)

            res_future.add_done_callback(internal_result)

        send_future.add_done_callback(internal_goal_response)
        return True

    def cancel_current_goal(self) -> None:
        """Cancel the currently active goal if one is running."""
        if self._goal_handle and self._is_moving:
            self._logger.info('Requesting cancellation of current goal...')
            self._goal_handle.cancel_goal_async()
