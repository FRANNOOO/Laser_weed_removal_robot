#!/usr/bin/env python3
"""
Weed Removal State Machine Node.

Combines arm trajectory execution (via FollowCartesianTrajectory action)
and laser triggering (via ActivateLaser service) into a coordinated
weed removal sequence.
"""

from enum import Enum
import sys
from typing import Optional

from geometry_msgs.msg import Point, PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from state_machine.action_client_arm_control import CartesianActionClient
from state_machine.action_client_laser import ActionClientLaser
from std_msgs.msg import String


class State(str, Enum):
    """Execution states for weed removal state machine."""

    IDLE = 'IDLE'
    MOVING_TO_TARGET = 'MOVING_TO_TARGET'
    FIRING_LASER = 'FIRING_LASER'
    ERROR = 'ERROR'


class StateMachineNode(Node):
    """ROS 2 Node coordinating arm motion and laser activation for weed removal."""

    def __init__(self, node_name: str = 'state_machine_node') -> None:
        """Initialize the state machine node, parameters, clients, and topics."""
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter('target_x', 0.35)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', -0.10)
        self.declare_parameter('duration_sec', 2.0)
        self.declare_parameter('laser_duration_us', 500000)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('enforce_workspace_limits', True)
        self.declare_parameter('workspace_min_x', 0.290)
        self.declare_parameter('workspace_max_x', 0.400)
        self.declare_parameter('workspace_min_y', -0.070)
        self.declare_parameter('workspace_max_y', 0.070)
        self.declare_parameter('workspace_min_z', -0.140)
        self.declare_parameter('workspace_max_z', -0.080)

        # Read parameters
        self._target_x = float(self.get_parameter('target_x').value)
        self._target_y = float(self.get_parameter('target_y').value)
        self._target_z = float(self.get_parameter('target_z').value)
        self._duration_sec = float(self.get_parameter('duration_sec').value)
        self._laser_duration_us = int(self.get_parameter('laser_duration_us').value)
        self._auto_start = bool(self.get_parameter('auto_start').value)

        ws_min = (
            float(self.get_parameter('workspace_min_x').value),
            float(self.get_parameter('workspace_min_y').value),
            float(self.get_parameter('workspace_min_z').value),
        )
        ws_max = (
            float(self.get_parameter('workspace_max_x').value),
            float(self.get_parameter('workspace_max_y').value),
            float(self.get_parameter('workspace_max_z').value),
        )
        enforce_limits = bool(self.get_parameter('enforce_workspace_limits').value)

        # Clients
        self.arm = CartesianActionClient(
            self,
            workspace_min=ws_min,
            workspace_max=ws_max,
            enforce_workspace_limits=enforce_limits,
        )
        self.laser = ActionClientLaser(self)

        # State tracking
        self._state = State.IDLE
        self._cycle_finished = False

        # Publishers
        self._state_pub = self.create_publisher(String, '~/state', 10)
        # NEW: Publisher for the final removal status
        self._status_pub = self.create_publisher(String, '~/weed_removal_status', 10)

        # Subscriptions
        self._target_pos_sub = self.create_subscription(
            Point,
            '~/target_position',
            self._target_position_callback,
            10,
        )
        self._target_pose_sub = self.create_subscription(
            PoseStamped,
            '~/target_pose',
            self._target_pose_callback,
            10,
        )

        self._set_state(State.IDLE)
        self.get_logger().info(f'Initialized {node_name} in state {self._state.value}.')

        # Activate controllers after startup
        self._check_controllers_timer = self.create_timer(
            1.0, self._check_controllers_timer_callback
        )

        # Optional auto-start sequence
        if self._auto_start:
            self.get_logger().info('Auto-start enabled: starting weed removal sequence...')
            self._auto_start_timer = self.create_timer(2.0, self._auto_start_timer_callback)
        else:
            self._auto_start_timer = None

    @property
    def current_state(self) -> State:
        """Return the current state of the state machine."""
        return self._state

    def _set_state(self, new_state: State) -> None:
        """Update and publish machine state."""
        self._state = new_state
        msg = String()
        msg.data = new_state.value
        self._state_pub.publish(msg)
        self.get_logger().info(f'State -> {new_state.value}')

    def _check_controllers_timer_callback(self) -> None:
        """Ensure controllers are activated on node start."""
        if self._check_controllers_timer:
            self._check_controllers_timer.cancel()
            self._check_controllers_timer = None
        self.arm.check_and_activate_controllers_async()

    def _auto_start_timer_callback(self) -> None:
        """Trigger initial cycle if auto_start is configured."""
        if self._auto_start_timer:
            self._auto_start_timer.cancel()
            self._auto_start_timer = None
        self.execute_weed_removal(
            self._target_x,
            self._target_y,
            self._target_z,
            self._duration_sec,
            self._laser_duration_us,
        )

    def _target_position_callback(self, msg: Point) -> None:
        """Handle target position command."""
        self.get_logger().info(
            f'Received target position: x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}'
        )
        self.execute_weed_removal(
            msg.x,
            msg.y,
            msg.z,
            self._duration_sec,
            self._laser_duration_us,
        )

    def _target_pose_callback(self, msg: PoseStamped) -> None:
        """Handle target pose command."""
        pos = msg.pose.position
        self.get_logger().info(
            f'Received target pose: x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}'
        )
        self.execute_weed_removal(
            pos.x,
            pos.y,
            pos.z,
            self._duration_sec,
            self._laser_duration_us,
        )

    def execute_weed_removal(
        self,
        x: float,
        y: float,
        z: float,
        duration_sec: float = 2.0,
        laser_duration_us: int = 500000,
    ) -> bool:
        """
        Execute coordinated weed removal: arm motion -> laser trigger.
        """
        if self._state not in (State.IDLE, State.ERROR):
            self.get_logger().warn(
                'Rejecting weed removal request: machine currently busy '
                f'in state {self._state.value}'
            )
            return False

        valid, reason = self.arm.check_workspace(x, y, z)
        if not valid:
            self.get_logger().error(f'Target ({x:.3f}, {y:.3f}, {z:.3f}) invalid: {reason}')
            self._set_state(State.ERROR)
            self._set_state(State.IDLE)
            return False

        self._cycle_finished = False
        self._set_state(State.MOVING_TO_TARGET)

        def on_arm_goal_complete(result):
            if result is None or result.status != 4:
                status_code = result.status if result else 'REJECTED'
                self.get_logger().error(
                    f'Arm movement to target failed with status {status_code}.'
                )
                self._set_state(State.ERROR)
                self._set_state(State.IDLE)
                self._cycle_finished = True
                return

            self.get_logger().info('Arm arrived at target position. Triggering laser...')
            self._fire_laser_step(laser_duration_us)

        sent = self.arm.move_to_position(
            x, y, z, duration_sec=duration_sec, result_callback=on_arm_goal_complete
        )
        if not sent:
            self._set_state(State.ERROR)
            self._set_state(State.IDLE)
            self._cycle_finished = True
            return False

        return True

    def _fire_laser_step(self, laser_duration_us: int) -> None:
        """Trigger the laser and handle completion."""
        self._set_state(State.FIRING_LASER)
        future = self.laser.trigger_laser(laser_duration_us)

        if future is None:
            self.get_logger().error('Laser service unavailable. Aborting cycle.')
            self._set_state(State.ERROR)
            self._set_state(State.IDLE)
            self._cycle_finished = True
            return

        def on_laser_done(res_future):
            try:
                response = res_future.result()
                if response and response.success:
                    self.get_logger().info(f'Laser firing succeeded: {response.message}')
                    
                    # NEW: Publish the message that weed has been removed
                    status_msg = String()
                    status_msg.data = "TASK_COMPLETED: Weed removed successfully."
                    self._status_pub.publish(status_msg)
                    
                else:
                    msg_text = response.message if response else 'No response'
                    self.get_logger().error(f'Laser firing failed: {msg_text}')
                    
                    # NEW Publush the error of laser on the topic
                    error_msg = String()
                    error_msg.data = f"TASK_FAILED: Laser error - {msg_text}"
                    self._status_pub.publish(error_msg)
                    
                    self._set_state(State.ERROR)
            except Exception as e:
                self.get_logger().error(f'Exception while waiting for laser response: {e}')
                self._set_state(State.ERROR)

            self._set_state(State.IDLE)
            self._cycle_finished = True

        future.add_done_callback(on_laser_done)

    def run_one_shot(
        self,
        x: float,
        y: float,
        z: float,
        laser_duration_us: int = 500000,
        duration_sec: float = 2.0,
    ) -> bool:
        """
        Execute one weed removal cycle synchronously, spinning until complete.
        """
        success = self.execute_weed_removal(
            x, y, z, duration_sec=duration_sec, laser_duration_us=laser_duration_us
        )
        if not success:
            return False

        while rclpy.ok() and not self._cycle_finished:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self._state != State.ERROR


def main(args: Optional[list] = None) -> None:
    """Run the state machine node."""
    rclpy.init(args=args)

    node = StateMachineNode()

    # In ROS 2, positional user arguments precede '--ros-args'
    user_args = []
    for arg in sys.argv[1:]:
        if arg == '--ros-args':
            break
        user_args.append(arg)

    is_cli_coords = False
    target_x = 0.0
    target_y = 0.0
    target_z = 0.0
    dur_us = 500000

    if len(user_args) >= 3:
        try:
            target_x = float(user_args[0])
            target_y = float(user_args[1])
            target_z = float(user_args[2])
            dur_us = int(user_args[3]) if len(user_args) >= 4 else 500000
            is_cli_coords = True
        except ValueError:
            is_cli_coords = False

    if is_cli_coords:
        node.get_logger().info(
            f'Running one-shot CLI command to ({target_x}, {target_y}, {target_z})'
        )
        node.run_one_shot(target_x, target_y, target_z, laser_duration_us=dur_us)
    else:
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except Exception as e:
            if 'rcl_shutdown already called' not in str(e):
                raise

    try:
        node.destroy_node()
    except Exception:
        pass

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()