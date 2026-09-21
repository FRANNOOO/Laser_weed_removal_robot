#!/usr/bin/env python3
# Copyright 2026 Franek
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Navigation Systems Integration Node for Weed Removal.

Coordinates pre-planned path navigation (Nav2 / opennav_coverage) with the
weed removal state machine. To maximize weeding efficiency and treat as many
weeds as possible in a single stop, navigation is only paused when the oldest
unremoved weed in the queue approaches the back edge of the removal workspace.
Once stopped, the manipulator state machine lasers all reachable queued weeds.
After completion, navigation is resumed with an odometry-based re-detection
cooldown.
"""

from enum import Enum
import math
import time
from typing import Optional, Tuple

from geometry_msgs.msg import Point, PointStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from state_machine.weed_queue_manager import WeedQueueManager
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener

try:
    from custom_msgs.msg import WeedInfo
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    CUSTOM_MSGS_AVAILABLE = False
    WeedInfo = None


class NavigationState(Enum):
    """Lifecycle states of the navigation coordinator."""

    IDLE = 'IDLE'          # Robot navigating along planned coverage path
    PAUSE = 'PAUSE'        # Weed reached back edge; calling Nav2 pause
    TASK_EXE = 'TASK_EXE'  # Robot stopped; weed removal state machine active
    RESUMING = 'RESUMING'  # Weeding complete; calling Nav2 resume
    ERROR = 'ERROR'        # Error condition (timeout, service failure)


class NavigationCoordinatorStateMachine:
    """
    Event-driven FSM coordinating Nav2 pause/resume with weed removal.

    Implements:
    - Queueing incoming weeds while moving.
    - Back-edge stopping threshold evaluation based on arm workspace and velocity.
    - Asynchronous Nav2 pause and resume service invocation.
    - Safety watchdog timeout during weeding.
    - Odometry distance cooldown on resumption.
    """

    def __init__(self, node: 'NavigationCoordinatorNode') -> None:
        """Initialize the state machine in IDLE state."""
        self.node = node
        self.state = NavigationState.IDLE

        # Cooldown state tracking
        self._cooldown_pending = False
        self._cooldown_active = False
        self._cooldown_origin: Optional[Tuple[float, float]] = None

        self._enter_fns = {
            NavigationState.IDLE: self._enter_idle,
            NavigationState.PAUSE: self._enter_pause,
            NavigationState.TASK_EXE: self._enter_task_exe,
            NavigationState.RESUMING: self._enter_resuming,
            NavigationState.ERROR: self._enter_error,
        }

        self._enter_fns[self.state]()

    @property
    def is_cooldown_active(self) -> bool:
        """Return True if re-detection cooldown is currently active."""
        return self._cooldown_active

    def _transition(self, next_state: NavigationState) -> None:
        """Transition to the next state and execute its entry function."""
        self.node.get_logger().info(f'{self.state.value} -> {next_state.value}')
        self.state = next_state
        self.node.publish_state(next_state.value)
        enter_fn = self._enter_fns.get(next_state)
        if enter_fn:
            enter_fn()

    # -- IDLE -----------------------------------------------------------------
    def _enter_idle(self) -> None:
        """Enter IDLE: publish flags and arm cooldown if pending."""
        self.node.stop_hold_position()
        self.node.publish_nav_paused(False)
        self.node.publish_robot_stopped(False)
        if self._cooldown_pending:
            self._cooldown_pending = False
            self._arm_cooldown()
        else:
            self._cooldown_active = False
            self.node.publish_cooldown_active(False)

    def _arm_cooldown(self) -> None:
        """Arm distance-based cooldown from current odometry position."""
        self._cooldown_active = True
        self._cooldown_origin = self.node.current_position
        self.node.publish_cooldown_active(True)
        if self._cooldown_origin is None:
            self.node.get_logger().warn(
                'Entering cooldown without odometry; will lock origin once odom arrives'
            )
        else:
            self.node.get_logger().info(
                f'Armed distance cooldown ({self.node.min_resume_distance_m:.2f}m) '
                f'from ({self._cooldown_origin[0]:.2f}, {self._cooldown_origin[1]:.2f})'
            )

    def compute_stop_trigger_x(self, velocity_x: float) -> float:
        """
        Compute the X coordinate threshold that triggers robot stopping.

        Stopping target is the back edge plus a safety margin:
            x_target = workspace_min_x + back_edge_margin_x
        Taking forward velocity v_x and stopping delay dt_stop into account:
            x_trigger = x_target + (v_x * dt_stop if use_velocity_lookahead else 0)
        """
        x_target = self.node.workspace_min_x + self.node.back_edge_margin_x
        lookahead = 0.0
        if self.node.use_velocity_lookahead and velocity_x > 0.0:
            lookahead = velocity_x * self.node.stop_delay_sec

        x_trigger = x_target + lookahead
        # Clamp to front of workspace
        return min(x_trigger, self.node.workspace_max_x)

    def check_back_edge_trigger(self) -> Tuple[bool, Optional[float], float]:
        """
        Evaluate if the oldest unremoved queued weed is near the back edge.

        Returns (should_stop, oldest_weed_x, trigger_x).
        """
        oldest = self.node.queue_mgr.get_oldest_queued_weed()
        vx = self.node.current_velocity[0]
        trig_x = self.compute_stop_trigger_x(vx)
        self.node.publish_stop_trigger_x(trig_x)

        if oldest is None:
            return False, None, trig_x

        weed_x = oldest.position.x
        self.node.publish_oldest_weed_x(weed_x)

        # In robot_base_link, robot drives in +X, so weed moves in -X.
        # Weed is near/past back-edge trigger when weed_x <= trig_x.
        should_stop = (weed_x <= trig_x)
        return should_stop, weed_x, trig_x

    def on_weed_detected(self) -> None:
        """Handle detection event while in IDLE."""
        if self.state != NavigationState.IDLE:
            return
        if self._cooldown_active:
            return

        should_stop, weed_x, trig_x = self.check_back_edge_trigger()
        if should_stop:
            x_str = f'{weed_x:.3f}' if weed_x is not None else 'N/A'
            self.node.get_logger().info(
                f'Oldest weed at x={x_str}m <= trigger {trig_x:.3f}m! '
                'Triggering pause near back edge...'
            )
            self._transition(NavigationState.PAUSE)
        else:
            oldest = self.node.queue_mgr.get_oldest_queued_weed()
            wid = oldest.weed_id if oldest else '?'
            x_str = f'{weed_x:.3f}' if weed_x is not None else 'N/A'
            self.node.get_logger().info(
                f'Oldest weed ID={wid} at x={x_str}m > trigger {trig_x:.3f}m. '
                f'Queue size={self.node.queue_mgr.queue_size}. Continuing navigation.'
            )

    def on_external_pause_requested(self) -> None:
        """Handle external pause request (e.g. from /start_stop_robot service)."""
        if self.state == NavigationState.IDLE:
            self.node.get_logger().info(
                'External pause requested while in IDLE. Transitioning to PAUSE.'
            )
            self._transition(NavigationState.PAUSE)
        else:
            self.node.get_logger().info(
                f'External pause requested while already in {self.state.value}.'
            )

    def on_external_resume_requested(self) -> None:
        """Handle external resume request (e.g. from /start_stop_robot service)."""
        if self.state in (NavigationState.PAUSE, NavigationState.TASK_EXE):
            self.node.get_logger().info(
                f'External resume requested while in {self.state.value}. '
                'Transitioning to RESUMING.'
            )
            self._transition(NavigationState.RESUMING)
        elif self.state == NavigationState.IDLE:
            self.node.get_logger().info(
                'External resume requested while already in IDLE.'
            )

    def on_odom_update(self) -> None:
        """Handle odometry update for cooldown and dynamic back-edge check."""
        if self.state != NavigationState.IDLE:
            return

        if self._cooldown_active:
            if self._cooldown_origin is None:
                self._cooldown_origin = self.node.current_position
                return

            dist = math.hypot(
                self.node.current_position[0] - self._cooldown_origin[0],
                self.node.current_position[1] - self._cooldown_origin[1],
            )
            if dist >= self.node.min_resume_distance_m:
                self.node.get_logger().info(
                    f'Traveled {dist:.2f}m >= {self.node.min_resume_distance_m:.2f}m. '
                    'Cooldown cleared.'
                )
                self._cooldown_active = False
                self.node.publish_cooldown_active(False)
                # Check if there are already weeds in queue needing removal
                self.on_weed_detected()
            return

        # Not in cooldown: check if moving has brought oldest weed to back edge
        if self.node.queue_mgr.queue_size > 0:
            self.on_weed_detected()

    # -- PAUSE ----------------------------------------------------------------
    def _enter_pause(self) -> None:
        """Enter PAUSE: halt velocity and call pause services."""
        self.node.start_hold_position()
        self.node.publish_nav_paused(True)
        self.node.publish_zero_cmd_vel()
        self.node.get_logger().info('Calling pause services...')
        self.node.call_pause_services()

    def on_pause_confirmed(self) -> None:
        """Handle successful Nav2 pause confirmation."""
        if self.state != NavigationState.PAUSE:
            return
        self.node.get_logger().info(
            'Pause confirmed. Robot stationary; transitioning to TASK_EXE.'
        )
        self._transition(NavigationState.TASK_EXE)

    def on_pause_failed(self, message: str) -> None:
        """Handle failed Nav2 pause request."""
        if self.state != NavigationState.PAUSE:
            return
        self.node.get_logger().error(f'Pause service returned failure: {message}')
        self._transition(NavigationState.ERROR)

    # -- TASK_EXE -------------------------------------------------------------
    def _enter_task_exe(self) -> None:
        """Enter TASK_EXE: signal robot_stopped to state_machine_node and start watchdog."""
        self.node.publish_zero_cmd_vel()
        self.node.publish_robot_stopped(True)
        self.node.publish_start_lasering(True)
        self.node.get_logger().info(
            'Published robot_stopped=True and start_lasering=True. '
            f'Awaiting weed removal (queue size={self.node.queue_mgr.queue_size})...'
        )
        self.node.start_task_watchdog()

    def on_task_done(self) -> None:
        """Handle completion of weed removal."""
        if self.state != NavigationState.TASK_EXE:
            return
        self.node.get_logger().info('Weed removal completed. Transitioning to RESUMING.')
        self.node.stop_task_watchdog()
        self._transition(NavigationState.RESUMING)

    def on_task_timeout(self) -> None:
        """Handle watchdog timeout during weed removal."""
        if self.state != NavigationState.TASK_EXE:
            return
        self.node.get_logger().error(
            f'Weed removal timed out after {self.node.task_timeout_sec:.1f}s!'
        )
        self._transition(NavigationState.ERROR)

    # -- RESUMING -------------------------------------------------------------
    def _enter_resuming(self) -> None:
        """Enter RESUMING: call resume services and mark cooldown pending."""
        self.node.stop_hold_position()
        self.node.publish_robot_stopped(False)
        self._cooldown_pending = True
        self.node.get_logger().info('Calling resume services...')
        self.node.call_resume_services()

    def on_resume_confirmed(self) -> None:
        """Handle successful Nav2 resume confirmation."""
        if self.state != NavigationState.RESUMING:
            return
        self.node.get_logger().info('Resume confirmed. Transitioning to IDLE.')
        self._transition(NavigationState.IDLE)

    def on_resume_failed(self, message: str) -> None:
        """Handle failed Nav2 resume request."""
        if self.state != NavigationState.RESUMING:
            return
        self.node.get_logger().error(f'Resume service returned failure: {message}')
        self._transition(NavigationState.ERROR)

    # -- ERROR ----------------------------------------------------------------
    def _enter_error(self) -> None:
        """Enter ERROR state."""
        self.node.stop_hold_position()
        self.node.get_logger().error('Entered ERROR state. Awaiting operator recovery.')

    def on_recovered(self) -> None:
        """Recover from ERROR state back to IDLE."""
        if self.state != NavigationState.ERROR:
            return
        self.node.get_logger().info('Recovered from ERROR state. Returning to IDLE.')
        self._transition(NavigationState.IDLE)


class NavigationCoordinatorNode(Node):
    """
    ROS 2 node coordinating Nav2 coverage navigation and weed removal.

    Gated by back-edge detection threshold to maximize weeds treated per stop.
    """

    def __init__(self, node_name: str = 'nav_integration_node') -> None:
        """Initialize parameters, subscribers, publishers, and state machine."""
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter('pause_service_name', '/navigate_complete_coverage/pause')
        self.declare_parameter('resume_service_name', '/navigate_complete_coverage/resume')
        self.declare_parameter('start_stop_service_name', '/start_stop_robot')
        self.declare_parameter('service_timeout_sec', 5.0)
        self.declare_parameter('approx_weed_topic', '/tracked_weeds')
        self.declare_parameter('tracked_weeds_topic', '')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('detection_bool_topic', '/weed_detection')
        self.declare_parameter('robot_stopped_topic', '/robot_stopped')
        self.declare_parameter('weeds_treated_topic', '/state_machine_node/all_weeds_treated')
        self.declare_parameter('task_done_topic', '/lasering_done')
        self.declare_parameter('weed_removed_topic', '/state_machine_node/weed_removed')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('workspace_min_x', 0.290)
        self.declare_parameter('workspace_max_x', 0.400)
        self.declare_parameter('back_edge_margin_x', 0.020)
        self.declare_parameter('stop_delay_sec', 0.20)
        self.declare_parameter('use_velocity_lookahead', True)
        self.declare_parameter('min_resume_distance_m', 0.30)
        self.declare_parameter('task_timeout_sec', 60.0)
        self.declare_parameter('mock_nav2', False)
        self.declare_parameter('provide_start_stop_service', True)
        self.declare_parameter('robot_base_frame', 'robot_base_link')
        self.declare_parameter('hold_position_on_pause', True)

        # Read parameters
        self.pause_service_name = str(self.get_parameter('pause_service_name').value)
        self.resume_service_name = str(self.get_parameter('resume_service_name').value)
        self.start_stop_service_name = str(
            self.get_parameter('start_stop_service_name').value
        )
        self.service_timeout_sec = float(self.get_parameter('service_timeout_sec').value)
        tracked_topic = str(self.get_parameter('tracked_weeds_topic').value)
        if tracked_topic:
            self.approx_weed_topic = tracked_topic
        else:
            self.approx_weed_topic = str(self.get_parameter('approx_weed_topic').value)

        self.detection_bool_topic = str(self.get_parameter('detection_bool_topic').value)
        self.robot_stopped_topic = str(self.get_parameter('robot_stopped_topic').value)
        self.weeds_treated_topic = str(self.get_parameter('weeds_treated_topic').value)
        self.task_done_topic = str(self.get_parameter('task_done_topic').value)
        self.weed_removed_topic = str(self.get_parameter('weed_removed_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.workspace_min_x = float(self.get_parameter('workspace_min_x').value)
        self.workspace_max_x = float(self.get_parameter('workspace_max_x').value)
        self.back_edge_margin_x = float(self.get_parameter('back_edge_margin_x').value)
        self.stop_delay_sec = float(self.get_parameter('stop_delay_sec').value)
        self.use_velocity_lookahead = bool(self.get_parameter('use_velocity_lookahead').value)
        self.min_resume_distance_m = float(self.get_parameter('min_resume_distance_m').value)
        self.task_timeout_sec = float(self.get_parameter('task_timeout_sec').value)
        self.mock_nav2 = bool(self.get_parameter('mock_nav2').value)
        self.provide_start_stop_service = bool(
            self.get_parameter('provide_start_stop_service').value
        )
        self.robot_base_frame = str(self.get_parameter('robot_base_frame').value)
        self.hold_position_on_pause = bool(
            self.get_parameter('hold_position_on_pause').value
        )

        # Odometry state
        self.current_position: Optional[Tuple[float, float]] = None
        self.current_velocity: Tuple[float, float] = (0.0, 0.0)

        # Local weed tracking queue
        self.queue_mgr = WeedQueueManager()
        self._watchdog_timer = None
        self._hold_timer = None

        # TF2 buffer & listener for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self._robot_stopped_pub = self.create_publisher(Bool, self.robot_stopped_topic, 10)
        self._start_lasering_pub = self.create_publisher(Bool, '/start_lasering', 10)
        self._state_pub = self.create_publisher(String, '~/state', 10)
        self._nav_paused_pub = self.create_publisher(Bool, '~/nav_paused', 10)
        self._cooldown_active_pub = self.create_publisher(Bool, '~/cooldown_active', 10)
        self._stop_trigger_x_pub = self.create_publisher(Float32, '~/stop_trigger_x', 10)
        self._oldest_weed_x_pub = self.create_publisher(Float32, '~/oldest_weed_x', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Subscriptions: Weed Detections
        if CUSTOM_MSGS_AVAILABLE:
            self._tracked_weeds_sub = self.create_subscription(
                WeedInfo,
                self.approx_weed_topic,
                self._on_tracked_weeds,
                10,
            )
        else:
            self._tracked_weeds_sub = None

        self._detection_bool_sub = self.create_subscription(
            Bool,
            self.detection_bool_topic,
            self._on_detection_bool,
            10,
        )

        # Subscriptions: Odometry
        self._odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odom,
            10,
        )

        # Subscriptions: Weeding Task Completion
        self._weeds_treated_sub = self.create_subscription(
            Bool,
            self.weeds_treated_topic,
            self._on_all_weeds_treated,
            10,
        )
        self._task_done_sub = self.create_subscription(
            Bool,
            self.task_done_topic,
            self._on_all_weeds_treated,
            10,
        )
        self._weed_removed_sub = self.create_subscription(
            Int32,
            self.weed_removed_topic,
            self._on_weed_removed,
            10,
        )

        # Service Clients
        self.pause_client = self.create_client(Trigger, self.pause_service_name)
        self.resume_client = self.create_client(Trigger, self.resume_service_name)
        self.start_stop_client = self.create_client(
            SetBool, self.start_stop_service_name
        )

        # Optional Service Server for UI start/stop compatibility
        if self.provide_start_stop_service:
            self._start_stop_srv = self.create_service(
                SetBool,
                self.start_stop_service_name,
                self._on_start_stop_service_request,
            )
            self.get_logger().info(
                f'Hosting service {self.start_stop_service_name} (SetBool)'
            )
        else:
            self._start_stop_srv = None

        # FSM instance
        self.sm = NavigationCoordinatorStateMachine(self)

        self.get_logger().info(
            f'Initialized {node_name} in state {self.sm.state.value}. '
            f'workspace_x=[{self.workspace_min_x:.3f}, {self.workspace_max_x:.3f}], '
            f'back_edge_margin={self.back_edge_margin_x:.3f}m, '
            f'min_resume_distance={self.min_resume_distance_m:.2f}m, '
            f'mock_nav2={self.mock_nav2}'
        )

    # -- Publisher helpers ----------------------------------------------------
    def publish_state(self, state_str: str) -> None:
        """Publish current coordinator state."""
        msg = String()
        msg.data = state_str
        self._state_pub.publish(msg)

    def publish_robot_stopped(self, value: bool) -> None:
        """Publish robot_stopped flag."""
        msg = Bool()
        msg.data = value
        self._robot_stopped_pub.publish(msg)

    def publish_start_lasering(self, value: bool) -> None:
        """Publish start_lasering flag."""
        msg = Bool()
        msg.data = value
        self._start_lasering_pub.publish(msg)

    def publish_nav_paused(self, value: bool) -> None:
        """Publish nav_paused flag."""
        msg = Bool()
        msg.data = value
        self._nav_paused_pub.publish(msg)

    def publish_cooldown_active(self, value: bool) -> None:
        """Publish cooldown_active flag."""
        msg = Bool()
        msg.data = value
        self._cooldown_active_pub.publish(msg)

    def publish_stop_trigger_x(self, value: float) -> None:
        """Publish calculated stop trigger X."""
        msg = Float32()
        msg.data = float(value)
        self._stop_trigger_x_pub.publish(msg)

    def publish_oldest_weed_x(self, value: float) -> None:
        """Publish current oldest weed X coordinate."""
        msg = Float32()
        msg.data = float(value)
        self._oldest_weed_x_pub.publish(msg)

    # -- Watchdog timer -------------------------------------------------------
    def start_task_watchdog(self) -> None:
        """Start task timeout watchdog timer."""
        self.stop_task_watchdog()
        self._watchdog_timer = self.create_timer(
            self.task_timeout_sec,
            self._on_watchdog_timeout,
        )

    def stop_task_watchdog(self) -> None:
        """Cancel task timeout watchdog timer if active."""
        if self._watchdog_timer is not None:
            self._watchdog_timer.cancel()
            self._watchdog_timer = None

    def _on_watchdog_timeout(self) -> None:
        """Handle watchdog timer trigger."""
        self.stop_task_watchdog()
        self.sm.on_task_timeout()

    # -- Motion Control & Service Helpers -------------------------------------
    def publish_zero_cmd_vel(self) -> None:
        """Publish zero velocity to cmd_vel to ensure robot halts immediately."""
        msg = Twist()
        self._cmd_vel_pub.publish(msg)

    def start_hold_position(self) -> None:
        """Start periodic zero-velocity publishing if hold_position_on_pause is enabled."""
        if not self.hold_position_on_pause or self._hold_timer is not None:
            return
        self._hold_timer = self.create_timer(0.1, self.publish_zero_cmd_vel)

    def stop_hold_position(self) -> None:
        """Stop periodic zero-velocity publishing."""
        if self._hold_timer is not None:
            self._hold_timer.cancel()
            self._hold_timer = None

    def _on_start_stop_service_request(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        """Handle incoming request on /start_stop_robot from UI or external caller."""
        self.get_logger().info(
            f'Received {self.start_stop_service_name} request: data={request.data}'
        )
        if request.data:
            if self.resume_client.service_is_ready():
                req = Trigger.Request()
                self.resume_client.call_async(req)
            self.sm.on_external_resume_requested()
            response.success = True
            response.message = 'Robot navigation resume requested'
        else:
            self.publish_zero_cmd_vel()
            if self.pause_client.service_is_ready():
                req = Trigger.Request()
                self.pause_client.call_async(req)
            self.sm.on_external_pause_requested()
            response.success = True
            response.message = 'Robot navigation pause requested'
        return response

    def transform_to_base_link(
        self,
        point: Point,
        source_frame: str,
    ) -> Point:
        """
        Transform a 3D Point from source_frame to robot_base_frame.

        Falls back to the input point if frames match or TF is unavailable.
        """
        if not source_frame or source_frame == self.robot_base_frame:
            return point

        try:
            pt_stamped = PointStamped()
            pt_stamped.header.frame_id = source_frame
            pt_stamped.header.stamp = rclpy.time.Time().to_msg()
            pt_stamped.point = point

            transformed = self.tf_buffer.transform(
                pt_stamped,
                self.robot_base_frame,
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            return transformed.point
        except Exception as ex:
            if 0.20 <= point.x <= 0.60:
                self.get_logger().debug(
                    f'TF transform from {source_frame} failed ({ex}); using raw point.'
                )
                return point
            self.get_logger().warning(
                f'Failed to transform weed from {source_frame} to '
                f'{self.robot_base_frame}: {ex}'
            )
            return point

    def call_pause_services(self) -> None:
        """Invoke available pause services (Trigger and SetBool) and halt robot."""
        self.publish_zero_cmd_vel()
        if self.mock_nav2:
            self.get_logger().info('[MOCK] Mocking pause service confirmation')
            self.sm.on_pause_confirmed()
            return

        pause_triggered = False

        # Try Trigger client (/navigate_complete_coverage/pause)
        if self.pause_client.service_is_ready():
            pause_triggered = True
            req = Trigger.Request()
            future = self.pause_client.call_async(req)
            future.add_done_callback(
                lambda f: self._on_service_response(f, 'pause')
            )
        else:
            self.get_logger().debug(
                f'Nav2 pause service {self.pause_service_name} not immediately available.'
            )

        # Try SetBool client (/start_stop_robot) only if not hosting it
        if not self.provide_start_stop_service and self.start_stop_client.service_is_ready():
            pause_triggered = True
            req_bool = SetBool.Request()
            req_bool.data = False
            future_bool = self.start_stop_client.call_async(req_bool)
            future_bool.add_done_callback(
                lambda f: self._on_set_bool_response(f, 'start_stop_pause')
            )

        if not pause_triggered:
            self.get_logger().info(
                'External navigation pause services not active; '
                'robot velocity halted via cmd_vel and proceeding with pause.'
            )
            self.sm.on_pause_confirmed()

    def call_resume_services(self) -> None:
        """Invoke available resume services (Trigger and SetBool)."""
        if self.mock_nav2:
            self.get_logger().info('[MOCK] Mocking resume service confirmation')
            self.sm.on_resume_confirmed()
            return

        resume_triggered = False

        # Try Trigger client (/navigate_complete_coverage/resume)
        if self.resume_client.service_is_ready():
            resume_triggered = True
            req = Trigger.Request()
            future = self.resume_client.call_async(req)
            future.add_done_callback(
                lambda f: self._on_service_response(f, 'resume')
            )
        else:
            self.get_logger().debug(
                f'Nav2 resume service {self.resume_service_name} not immediately available.'
            )

        # Try SetBool client (/start_stop_robot) only if not hosting it
        if not self.provide_start_stop_service and self.start_stop_client.service_is_ready():
            resume_triggered = True
            req_bool = SetBool.Request()
            req_bool.data = True
            future_bool = self.start_stop_client.call_async(req_bool)
            future_bool.add_done_callback(
                lambda f: self._on_set_bool_response(f, 'start_stop_resume')
            )

        if not resume_triggered:
            self.get_logger().info(
                'External navigation resume services not active; '
                'proceeding with resume confirmation.'
            )
            self.sm.on_resume_confirmed()

    def _on_service_response(self, future, label: str) -> None:
        """Handle asynchronous Trigger service response."""
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'{label} service call failed: {exc}')
            if label == 'pause':
                self.sm.on_pause_failed(str(exc))
            elif label == 'resume':
                self.sm.on_resume_failed(str(exc))
            return

        if not response or not response.success:
            err_msg = response.message if response else 'No response received'
            self.get_logger().error(f'{label} service returned failure: {err_msg}')
            if label == 'pause':
                self.sm.on_pause_failed(err_msg)
            elif label == 'resume':
                self.sm.on_resume_failed(err_msg)
            return

        self.get_logger().info(f'{label} service succeeded: {response.message}')
        if label == 'pause':
            self.sm.on_pause_confirmed()
        elif label == 'resume':
            self.sm.on_resume_confirmed()

    def _on_set_bool_response(self, future, label: str) -> None:
        """Handle SetBool service response."""
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'{label} service call failed: {exc}')
            return

        if not response or not response.success:
            err = response.message if response else 'No response'
            self.get_logger().warn(f'{label} returned: {err}')
            return

        self.get_logger().info(f'{label} service succeeded: {response.message}')
        if 'pause' in label:
            self.sm.on_pause_confirmed()
        elif 'resume' in label:
            self.sm.on_resume_confirmed()

    # -- Subscriber Callbacks -------------------------------------------------
    def _on_tracked_weeds(self, msg: 'WeedInfo') -> None:
        """Handle incoming WeedInfo message and update queue coordinates."""
        now_sec = time.time()
        for weed in msg.weeds:
            raw_pt = Point(
                x=float(weed.position_x),
                y=float(weed.position_y),
                z=float(weed.position_z),
            )
            pos = self.transform_to_base_link(raw_pt, msg.header.frame_id)
            self.queue_mgr.add_or_update(weed.id, pos, timestamp=now_sec)
            self.get_logger().info(
                f'Enqueued weed ID={weed.id} at base_link ({pos.x:.3f}, {pos.y:.3f}). '
                f'Total in queue: {self.queue_mgr.queue_size}'
            )

        self.sm.on_weed_detected()

    def _on_detection_bool(self, msg: Bool) -> None:
        """Handle boolean detection trigger (e.g. from test script or model)."""
        if msg.data:
            # If coordinates were not given via WeedInfo, synthesize mock weed at back edge
            if self.queue_mgr.queue_size == 0:
                mock_pos = Point(x=self.workspace_min_x, y=0.0, z=-0.10)
                self.queue_mgr.add_or_update(-1, mock_pos)
            self.sm.on_weed_detected()

    def _on_odom(self, msg: Odometry) -> None:
        """Handle Odometry update."""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        self.current_velocity = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
        )
        self.sm.on_odom_update()

    def _on_all_weeds_treated(self, msg: Bool) -> None:
        """Handle completion signal from weed removal state machine."""
        if msg.data:
            self.sm.on_task_done()

    def _on_weed_removed(self, msg: Int32) -> None:
        """Update local queue manager when a specific weed ID is removed."""
        self.queue_mgr.mark_removed(msg.data)


def main(args=None) -> None:
    """Entry point for nav_integration_node."""
    rclpy.init(args=args)
    node = NavigationCoordinatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        if 'rcl_shutdown already called' not in str(e):
            raise
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
