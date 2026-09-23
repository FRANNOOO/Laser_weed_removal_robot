#!/usr/bin/env python3
"""
Weed Removal State Machine Node.

Coordinates multi-stage weed removal according to the design diagram:
Idle -> Approx_loc -> Detect_Precise -> Move_to_precise -> Lasering -> Idle.

Integrates a higher-level WeedQueueManager for tracking detected weeds by ID,
preventing duplicate lasering, gating operations on robot stop status, and
iterating through all reachable weeds during a single stop.
"""

from enum import Enum
import math
from typing import List, Optional, Set, Tuple

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from state_machine.action_client_arm_control import CartesianActionClient
from state_machine.action_client_laser import ActionClientLaser
from state_machine.weed_queue_manager import QueuedWeed, WeedQueueManager
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import SetBool, Trigger
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformListener

try:
    from custom_msgs.msg import Weed, WeedInfo
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    CUSTOM_MSGS_AVAILABLE = False
    Weed = None
    WeedInfo = None


class State(str, Enum):
    """Execution states for weed removal state machine."""

    IDLE = 'Idle'
    APPROX_LOC = 'Approx_loc'
    DETECT_PRECISE = 'Detect_Precise'
    MOVE_TO_PRECISE = 'Move_to_precise'
    LASERING = 'Lasering'


class StateMachineNode(Node):
    """ROS 2 Node coordinating weed removal and multi-weed queue management."""

    def __init__(self, node_name: str = 'state_machine_node') -> None:
        """Initialize state machine node, parameters, and queue."""
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter('duration_sec', 2.0)
        self.declare_parameter('laser_duration_us', 500000)
        self.declare_parameter('enforce_workspace_limits', True)
        self.declare_parameter('workspace_min_x', 0.290)
        self.declare_parameter('workspace_max_x', 0.400)
        self.declare_parameter('workspace_min_y', -0.070)
        self.declare_parameter('workspace_max_y', 0.070)
        self.declare_parameter('workspace_min_z', -0.140)
        self.declare_parameter('workspace_max_z', -0.080)

        # High-level queue & stop coordination parameters
        self.declare_parameter('approx_weed_topic', '/tracked_weeds')
        self.declare_parameter('robot_stopped_topic', '/robot_stopped')
        self.declare_parameter('auto_start_weeding', False)
        self.declare_parameter('target_z_default', -0.100)
        self.declare_parameter('simulate_precise_camera', True)
        self.declare_parameter('precise_offset_x', 0.003)
        self.declare_parameter('precise_offset_y', -0.002)
        self.declare_parameter('robot_base_frame', 'robot_base_link')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('detect_timeout_sec', 3.0)
        self.declare_parameter('arm_timeout_padding_sec', 5.0)
        self.declare_parameter('laser_timeout_padding_sec', 3.0)
        self.declare_parameter('dedup_radius', 0.03)
        self.declare_parameter('weeding_enabled', True)

        # Read parameters
        self._duration_sec = float(self.get_parameter('duration_sec').value)
        self._laser_duration_us = int(
            self.get_parameter('laser_duration_us').value
        )
        self._approx_weed_topic = str(
            self.get_parameter('approx_weed_topic').value
        )
        self._robot_stopped_topic = str(
            self.get_parameter('robot_stopped_topic').value
        )
        self._auto_start_weeding = bool(
            self.get_parameter('auto_start_weeding').value
        )
        self._target_z_default = float(
            self.get_parameter('target_z_default').value
        )
        self._simulate_precise_camera = bool(
            self.get_parameter('simulate_precise_camera').value
        )
        self._precise_offset_x = float(
            self.get_parameter('precise_offset_x').value
        )
        self._precise_offset_y = float(
            self.get_parameter('precise_offset_y').value
        )
        self._robot_base_frame = str(
            self.get_parameter('robot_base_frame').value
        )
        self._odom_topic = str(
            self.get_parameter('odom_topic').value
        )
        self._detect_timeout_sec = float(
            self.get_parameter('detect_timeout_sec').value
        )
        self._arm_timeout_padding_sec = float(
            self.get_parameter('arm_timeout_padding_sec').value
        )
        self._laser_timeout_padding_sec = float(
            self.get_parameter('laser_timeout_padding_sec').value
        )
        self._dedup_radius = float(
            self.get_parameter('dedup_radius').value
        )
        self._weeding_enabled = bool(
            self.get_parameter('weeding_enabled').value
        )

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
        enforce_limits = bool(
            self.get_parameter('enforce_workspace_limits').value
        )

        # Action and service clients
        self.arm = CartesianActionClient(
            self,
            workspace_min=ws_min,
            workspace_max=ws_max,
            enforce_workspace_limits=enforce_limits,
        )
        self.laser = ActionClientLaser(self)

        # High-level weed queue manager
        self.queue_mgr = WeedQueueManager(dedup_radius=self._dedup_radius)
        self._robot_stopped = self._auto_start_weeding
        self._current_weed: Optional[QueuedWeed] = None
        self._failed_weed_ids_for_stop: Set[int] = set()
        self._mock_weed_id_counter = -1
        self._failed_count: int = 0

        # Odometry state for dynamic weed queue tracking
        self._current_position: Optional[Tuple[float, float]] = None
        self._current_yaw: float = 0.0

        # Internal state & target locations
        self._state = State.IDLE
        self._approx_loc: Optional[Point] = None
        self._precise_loc: Optional[Point] = None
        self._precise_sim_timer = None
        self._detect_watchdog_timer = None
        self._arm_watchdog_timer = None
        self._laser_watchdog_timer = None

        # TF2 listener for frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers for diagram state & control flags
        self._state_pub = self.create_publisher(String, '~/state', 10)
        self._ready_for_removal_pub = self.create_publisher(
            Bool, '~/ready_for_removal', 10
        )
        self._laser_trigger_pub = self.create_publisher(
            Bool, '~/laser_trigger', 10
        )
        self._arm_send_goal_pub = self.create_publisher(
            Bool, '~/arm_send_goal', 10
        )
        self._trigger_yolo_pub = self.create_publisher(
            Bool, '~/trigger_yolo', 10
        )
        self._removal_finished_pub = self.create_publisher(
            Bool, '~/removal_finished', 10
        )
        self._precise_loc_pub = self.create_publisher(
            Point, '~/precise_location', 10
        )

        # High-level coordination publishers
        self._weed_removed_pub = self.create_publisher(
            Int32, '~/weed_removed', 10
        )
        self._all_weeds_treated_pub = self.create_publisher(
            Bool, '~/all_weeds_treated', 10
        )
        self._queue_size_pub = self.create_publisher(
            Int32, '~/queue_size', 10
        )
        self._weeding_active_pub = self.create_publisher(
            Bool, '~/weeding_active', 10
        )
        self._removed_count_pub = self.create_publisher(
            Int32, '~/removed_count', 10
        )
        self._failed_count_pub = self.create_publisher(
            Int32, '~/failed_count', 10
        )

        # Service to dynamically enable/disable weeding
        self._start_stop_srv = self.create_service(
            SetBool,
            '/start_stop_weeding',
            self._start_stop_weeding_callback,
        )
        self.get_logger().info('Hosting service /start_stop_weeding (SetBool)')

        # Client for Depth Camera localization trigger service
        self._trigger_localization_client = self.create_client(
            Trigger,
            '/depth_camera_node/trigger_localization',
        )

        if CUSTOM_MSGS_AVAILABLE:
            self._weed_burned_pub = self.create_publisher(
                Weed, '/weed_burned', 10
            )
        else:
            self._weed_burned_pub = None

        # Subscriptions: First Camera / Tracked Weeds
        if CUSTOM_MSGS_AVAILABLE:
            self._tracked_weeds_sub = self.create_subscription(
                WeedInfo,
                self._approx_weed_topic,
                self._tracked_weeds_callback,
                10,
            )
            self.get_logger().info(
                f'Subscribed to approx weeds topic: {self._approx_weed_topic}'
            )
        else:
            self.get_logger().warn(
                'custom_msgs not available; /tracked_weeds disabled.'
            )

        # Subscriptions: Robot Stop Flag
        self._robot_stopped_sub = self.create_subscription(
            Bool,
            self._robot_stopped_topic,
            self._robot_stopped_callback,
            10,
        )
        self.get_logger().info(
            f'Subscribed to robot stopped topic: {self._robot_stopped_topic}'
        )

        # Subscriptions: Odometry for queue position tracking
        self._odom_sub = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._odom_callback,
            10,
        )
        self.get_logger().info(
            f'Subscribed to odometry topic: {self._odom_topic}'
        )

        # Backward-compatible manual injection subscriptions
        self._approx_loc_sub = self.create_subscription(
            Point,
            '~/approx_location',
            self._approx_location_callback,
            10,
        )
        self._approx_loc_pose_sub = self.create_subscription(
            PoseStamped,
            '~/approx_location_pose',
            self._approx_location_pose_callback,
            10,
        )
        self._target_pos_alias_sub = self.create_subscription(
            Point,
            '~/target_position',
            self._approx_location_callback,
            10,
        )

        # Second camera / precise location subscriptions
        self._precise_loc_sub = self.create_subscription(
            Point,
            '~/precise_location',
            self._precise_location_callback,
            10,
        )
        self._precise_loc_pose_sub = self.create_subscription(
            PoseStamped,
            '~/precise_location_pose',
            self._precise_location_pose_callback,
            10,
        )

        # Enter Idle state
        self._enter_idle()
        self.get_logger().info(
            f'Initialized {node_name} in state {self._state.value}. '
            f'auto_start_weeding={self._auto_start_weeding}, '
            f'robot_stopped={self._robot_stopped}'
        )

        # Activate controllers after startup
        self._check_controllers_timer = self.create_timer(
            1.0, self._check_controllers_timer_callback
        )

    @property
    def current_state(self) -> State:
        """Return current state of the state machine."""
        return self._state

    def _check_controllers_timer_callback(self) -> None:
        """Ensure controllers are activated on node start and retry until active."""
        self.arm.check_and_activate_controllers_async()
        if self.arm.is_controllers_ready:
            if self._check_controllers_timer:
                self._check_controllers_timer.cancel()
                self._check_controllers_timer = None

    def _start_stop_weeding_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """
        Handle /start_stop_weeding service request to enable or disable weeding.

        :param request: SetBool request containing enable/disable flag.
        :param response: SetBool response containing result.
        :return: Populated SetBool response.
        """
        self._weeding_enabled = bool(request.data)
        response.success = True
        response.message = f'Weeding enabled set to {self._weeding_enabled}'
        self.get_logger().info(response.message)
        if not self._weeding_enabled:
            if self._state != State.IDLE:
                self.get_logger().warn(
                    'Weeding disabled while state machine is active; returning to Idle.'
                )
                self._enter_idle()
        else:
            stopped_or_auto = self._robot_stopped or self._auto_start_weeding
            if stopped_or_auto and self._state == State.IDLE:
                self._check_and_start_next_weed()
        return response

    def _publish_flag(self, pub, value: bool) -> None:
        """Publish a boolean control flag."""
        msg = Bool()
        msg.data = value
        pub.publish(msg)

    def _publish_queue_size(self) -> None:
        """Publish the current number of queued weeds."""
        msg = Int32()
        msg.data = self.queue_mgr.queue_size
        self._queue_size_pub.publish(msg)

    def _set_state(self, new_state: State) -> None:
        """Update and publish machine state."""
        self._state = new_state
        msg = String()
        msg.data = new_state.value
        self._state_pub.publish(msg)

        is_active = (new_state != State.IDLE)
        self._publish_flag(self._weeding_active_pub, is_active)
        self.get_logger().info(f'State -> {new_state.value}')

    def _is_point_reachable(self, pt: Point) -> bool:
        """
        Check if coordinates fall within physical workspace boundaries.

        Uses target_z_default for the Z check because after TF transform
        from map to robot_base_link, weed Z is at ground level (~-0.5m)
        which is outside the arm's focal height range [-0.140, -0.080].
        The arm always operates at a fixed Z height, so only X and Y
        determine whether a weed is reachable.

        :param pt: Target 3D coordinates.
        :return: True if target is reachable by the arm, False otherwise.
        """
        z = self._target_z_default
        valid, _ = self.arm.check_workspace(pt.x, pt.y, z)
        return valid

    # --- Higher-Level Queue & Stop Coordination ---

    def _get_reachable_weeds_for_current_stop(self) -> List[QueuedWeed]:
        """Return active queued weeds reachable in workspace and not failed during current stop."""
        return [
            w for w in self.queue_mgr.get_all_active_weeds()
            if w.weed_id not in self._failed_weed_ids_for_stop
            and self._is_point_reachable(w.position)
        ]

    def _check_and_start_next_weed(self) -> bool:
        """
        Check queue for next reachable weed and begin removal sequence.

        :return: True if a weed removal was initiated, False otherwise.
        """
        if not self._weeding_enabled:
            self.get_logger().debug('Weeding is disabled via /start_stop_weeding.')
            return False

        if not (self._robot_stopped or self._auto_start_weeding):
            self.get_logger().info(
                'Robot moving (robot_stopped=False); waiting to stop.'
            )
            return False

        if self._state != State.IDLE:
            return False

        if self._current_position is not None:
            current_pose = (
                self._current_position[0],
                self._current_position[1],
                self._current_yaw,
            )
            self.queue_mgr.update_positions(None, current_pose)

        reachable = self._get_reachable_weeds_for_current_stop()
        if not reachable:
            if self.queue_mgr.queue_size > 0:
                coords = [
                    f'ID={w.weed_id}:({w.position.x:.3f},{w.position.y:.3f})'
                    for w in self.queue_mgr.get_all_active_weeds()
                ]
                self.get_logger().info(
                    f'{self.queue_mgr.queue_size} weed(s) in queue, '
                    f'none reachable in workspace. [{", ".join(coords)}]'
                )
            # If robot is stopped and no weeds are reachable (or queue
            # is empty), signal completion so nav_integration_node can
            # resume navigation instead of waiting for the watchdog
            # timeout.
            if self._robot_stopped:
                # Purge weeds that have permanently overshot the arm workspace
                purged = self.queue_mgr.purge_unreachable(self.arm.workspace_max[0])
                if purged > 0:
                    self.get_logger().info(
                        f'Purged {purged} overshot weed(s) with '
                        f'x > {self.arm.workspace_max[0]:.3f}m.'
                    )
                    self._publish_queue_size()
                self.get_logger().info(
                    'Robot stopped but no reachable weeds. '
                    'Publishing all_weeds_treated=True to resume nav.'
                )
                self._publish_flag(self._all_weeds_treated_pub, True)
            return False

        next_weed = reachable[0]
        self._current_weed = next_weed
        self.get_logger().info(
            f'Targeting weed ID={next_weed.weed_id} at '
            f'({next_weed.position.x:.4f}, {next_weed.position.y:.4f}, '
            f'{next_weed.position.z:.4f})'
        )
        self._enter_approx_loc(next_weed.position)
        return True

    def _on_weed_treatment_failed(self, reason: str) -> None:
        """
        Handle failure during weed treatment (motion aborted, rejected, or invalid).

        Marks the current weed as failed for the current stop, returns to IDLE,
        and either targets the next reachable weed or signals navigation to resume.
        """
        failed_weed = self._current_weed
        failed_id = failed_weed.weed_id if failed_weed else None
        self.get_logger().error(
            f'Weed treatment failed for weed ID={failed_id}: {reason}'
        )
        if failed_id is not None:
            self._failed_weed_ids_for_stop.add(failed_id)
        self._current_weed = None

        self._failed_count += 1
        msg_failed = Int32()
        msg_failed.data = self._failed_count
        self._failed_count_pub.publish(msg_failed)

        self._enter_idle()

        # Check if another reachable weed can be treated at this stop
        started = self._check_and_start_next_weed()
        if not started and self._robot_stopped:
            self.get_logger().info(
                'No remaining reachable weeds after failure; '
                'publishing all_weeds_treated=True to resume nav.'
            )
            self._publish_flag(self._all_weeds_treated_pub, True)

    def _robot_stopped_callback(self, msg: Bool) -> None:
        """
        Handle updates to the robot stopped flag.

        :param msg: Boolean flag indicating if robot has stopped.
        """
        prev_stopped = self._robot_stopped
        self._robot_stopped = msg.data
        self.get_logger().info(
            f'Robot stopped flag updated: {self._robot_stopped}'
        )

        if not self._robot_stopped:
            self._failed_weed_ids_for_stop.clear()

        if (self._robot_stopped and not prev_stopped and
                self._state == State.IDLE):
            self._failed_weed_ids_for_stop.clear()
            self.get_logger().info(
                'Robot stopped; checking queued weeds for removal.'
            )
            if self._current_position is not None:
                current_pose = (
                    self._current_position[0],
                    self._current_position[1],
                    self._current_yaw,
                )
                self.queue_mgr.update_positions(None, current_pose)
            started = self._check_and_start_next_weed()
            if not started:
                self.get_logger().info(
                    'No reachable weeds to treat upon robot stop; '
                    'publishing all_weeds_treated=True.'
                )
                self._publish_flag(self._all_weeds_treated_pub, True)

    def _transform_to_base_link(
        self,
        point: Point,
        source_frame: str,
    ) -> Point:
        """
        Transform a 3D Point from source_frame to robot_base_frame.

        Falls back to the input point if frames match or TF is unavailable.
        """
        if not source_frame or source_frame == self._robot_base_frame:
            # Still ensure Z is within operating limits
            ws_min_z = self.arm.workspace_min[2]
            ws_max_z = self.arm.workspace_max[2]
            z = point.z if (ws_min_z <= point.z <= ws_max_z) else self._target_z_default
            return Point(x=point.x, y=point.y, z=float(z))

        try:
            pt_stamped = PointStamped()
            pt_stamped.header.frame_id = source_frame
            pt_stamped.header.stamp = rclpy.time.Time().to_msg()
            pt_stamped.point = point

            transformed = self.tf_buffer.transform(
                pt_stamped,
                self._robot_base_frame,
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            pt_out = transformed.point
            ws_min_z = self.arm.workspace_min[2]
            ws_max_z = self.arm.workspace_max[2]
            if not (ws_min_z <= pt_out.z <= ws_max_z):
                pt_out.z = self._target_z_default
            return pt_out
        except Exception as ex:
            ws_min_z = self.arm.workspace_min[2]
            ws_max_z = self.arm.workspace_max[2]
            fallback_z = point.z if (ws_min_z <= point.z <= ws_max_z) else self._target_z_default
            fallback_pt = Point(x=point.x, y=point.y, z=float(fallback_z))
            if 0.20 <= point.x <= 0.60:
                self.get_logger().debug(
                    f'TF transform from {source_frame} to {self._robot_base_frame} '
                    f'failed ({ex}); using raw point within workspace.'
                )
                return fallback_pt
            self.get_logger().warning(
                f'Failed to transform weed from {source_frame} to '
                f'{self._robot_base_frame}: {ex}'
            )
            return fallback_pt

    def _odom_callback(self, msg: Odometry) -> None:
        """
        Handle odometry update to track vehicle position.

        :param msg: Odometry message with vehicle pose.
        """
        self._current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._current_yaw = math.atan2(siny_cosp, cosy_cosp)

        current_pose = (
            self._current_position[0],
            self._current_position[1],
            self._current_yaw,
        )
        self.queue_mgr.update_positions(
            None,
            current_pose,
        )
        self.queue_mgr.purge_unreachable(
            self.arm.workspace_max[0] + 0.15
        )

    def _tracked_weeds_callback(self, msg: 'WeedInfo') -> None:
        """
        Handle incoming 3D weed coordinates from first camera / YOLO tracker.

        :param msg: WeedInfo message with detected weeds and IDs.
        """
        if msg.header.stamp.sec > 0:
            now_sec = (
                float(msg.header.stamp.sec)
                + float(msg.header.stamp.nanosec) * 1e-9
            )
        else:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
        current_odom = None
        if self._current_position is not None:
            current_odom = (
                self._current_position[0],
                self._current_position[1],
                self._current_yaw,
            )

        new_count = 0
        for weed in msg.weeds:
            if self.queue_mgr.is_removed(weed.id):
                continue

            raw_pt = Point(
                x=float(weed.position_x),
                y=float(weed.position_y),
                z=float(weed.position_z),
            )
            pos = self._transform_to_base_link(raw_pt, msg.header.frame_id)

            if self.queue_mgr.add_or_update(
                weed.id,
                pos,
                timestamp=now_sec,
                raw_position=raw_pt,
                raw_frame_id=msg.header.frame_id,
                odom_pose=current_odom,
            ):
                new_count += 1

        if len(msg.weeds) > 0:
            if new_count > 0:
                self.get_logger().info(
                    f'Received {len(msg.weeds)} weed detection(s). '
                    f'New: {new_count}, Total active in queue: {self.queue_mgr.queue_size}'
                )
                self._publish_queue_size()
            else:
                self.get_logger().debug(
                    f'Received {len(msg.weeds)} weed detection updates (none new).'
                )

        stopped_or_auto = self._robot_stopped or self._auto_start_weeding
        if stopped_or_auto and self._state == State.IDLE:
            self._check_and_start_next_weed()

    # --- Watchdog Timers ---

    def _cancel_detect_watchdog(self) -> None:
        """Cancel the Camera 2 detection watchdog timer if active."""
        if self._detect_watchdog_timer is not None:
            self._detect_watchdog_timer.cancel()
            self._detect_watchdog_timer = None

    def _cancel_arm_watchdog(self) -> None:
        """Cancel the arm trajectory action watchdog timer if active."""
        if self._arm_watchdog_timer is not None:
            self._arm_watchdog_timer.cancel()
            self._arm_watchdog_timer = None

    def _cancel_laser_watchdog(self) -> None:
        """Cancel the laser service activation watchdog timer if active."""
        if self._laser_watchdog_timer is not None:
            self._laser_watchdog_timer.cancel()
            self._laser_watchdog_timer = None

    def _on_trigger_localization_response(self, future) -> None:
        """Handle response from /depth_camera_node/trigger_localization service."""
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f'Depth camera localization acknowledged: {res.message}')
            else:
                self.get_logger().warn(f'Depth camera localization returned false: {res.message}')
        except Exception as e:
            self.get_logger().warn(f'Error calling trigger_localization service: {e}')

    def _on_detect_precise_timeout(self) -> None:
        """Handle Camera 2 detection timeout by falling back to approx location."""
        self._cancel_detect_watchdog()
        self._publish_flag(self._trigger_yolo_pub, False)
        if self._state != State.DETECT_PRECISE:
            return
        if self._approx_loc is None:
            self._on_weed_treatment_failed('Camera 2 timeout and approx_loc is None')
            return
        self.get_logger().warn(
            f'Camera 2 detection timed out after {self._detect_timeout_sec:.1f}s; '
            'falling back to approximate coordinates for lasering.'
        )
        self._enter_move_to_precise(self._approx_loc)

    def _on_arm_timeout(self) -> None:
        """Handle arm motion watchdog timeout."""
        self._cancel_arm_watchdog()
        self._publish_flag(self._arm_send_goal_pub, False)
        self.get_logger().error(
            f'Arm motion action timed out in state {self._state.value}; cancelling goal.'
        )
        self.arm.cancel_current_goal()
        self._on_weed_treatment_failed('Arm motion action timed out')

    def _on_laser_timeout(self) -> None:
        """Handle laser activation service watchdog timeout."""
        self._cancel_laser_watchdog()
        self.get_logger().error('Laser activation service timed out.')
        self._exit_lasering(laser_success=False)

    # --- State Transitions ---

    def _enter_idle(self) -> None:
        """Enter Idle state and set diagram signals."""
        self._cancel_detect_watchdog()
        self._cancel_arm_watchdog()
        self._cancel_laser_watchdog()
        if self._precise_sim_timer is not None:
            self._precise_sim_timer.cancel()
            self._precise_sim_timer = None

        self._set_state(State.IDLE)
        self._publish_flag(self._ready_for_removal_pub, True)
        self._publish_flag(self._laser_trigger_pub, False)
        self._publish_flag(self._arm_send_goal_pub, False)
        self._publish_flag(self._trigger_yolo_pub, False)
        self._publish_flag(self._removal_finished_pub, False)
        self.get_logger().info(
            'Idle: ready_for_removal=True, laser_trigger=False, '
            'arm_send_goal=False'
        )

    def _enter_approx_loc(self, target: Point) -> None:
        """
        Enter Approx_loc state upon weed targeting.

        :param target: Approximate coordinates to position the arm.
        """
        self._cancel_arm_watchdog()
        self._cancel_detect_watchdog()
        self._cancel_laser_watchdog()

        # Clamp Z to valid arm workspace range. After TF transform from
        # map, weed Z is at ground level in robot_base_link (~-0.5m) but
        # the arm operates at a fixed focal height [-0.140, -0.080].
        ws_min_z = self.arm.workspace_min[2]
        ws_max_z = self.arm.workspace_max[2]
        clamped_z = target.z
        if not (ws_min_z <= clamped_z <= ws_max_z):
            clamped_z = self._target_z_default
        target = Point(x=target.x, y=target.y, z=float(clamped_z))

        self._approx_loc = target
        self._set_state(State.APPROX_LOC)
        self._publish_flag(self._ready_for_removal_pub, False)
        self._publish_flag(self._removal_finished_pub, False)

        valid, reason = self.arm.check_workspace(target.x, target.y, target.z)
        if not valid:
            self._on_weed_treatment_failed(
                f'Approx target ({target.x:.3f}, {target.y:.3f}, {target.z:.3f}) '
                f'outside workspace: {reason}'
            )
            return

        self._publish_flag(self._arm_send_goal_pub, True)
        self.get_logger().info(
            f'Approx_loc: arm_goal_pose=({target.x:.4f}, {target.y:.4f}, '
            f'{target.z:.4f}), arm_send_goal=True'
        )

        def on_approx_move_done(result):
            self._cancel_arm_watchdog()
            self._publish_flag(self._arm_send_goal_pub, False)
            if result is None or result.status != 4:
                status_code = result.status if result else 'REJECTED'
                msg_text = (
                    result.result.msg
                    if (result and hasattr(result, 'result') and result.result)
                    else ''
                )
                self._on_weed_treatment_failed(
                    f'Motion to approx_loc failed with status {status_code}: {msg_text}'
                )
                return

            self.get_logger().info(
                'Approx_loc: [arm_move_done] -> '
                'transitioning to Detect_Precise'
            )
            self._enter_detect_precise()

        sent = self.arm.move_to_position(
            target.x, target.y, target.z,
            duration_sec=self._duration_sec,
            result_callback=on_approx_move_done,
        )
        self._publish_flag(self._arm_send_goal_pub, False)
        if not sent:
            self._on_weed_treatment_failed('Failed to send goal to approx_loc')
            return

        arm_timeout = self._duration_sec + self._arm_timeout_padding_sec
        self._arm_watchdog_timer = self.create_timer(
            arm_timeout, self._on_arm_timeout
        )

    def _enter_detect_precise(self) -> None:
        """Enter Detect_Precise state upon arm arrival at approx position."""
        self._cancel_detect_watchdog()
        self._set_state(State.DETECT_PRECISE)
        self._publish_flag(self._trigger_yolo_pub, True)
        self.get_logger().info(
            'Detect_Precise: trigger_yolo=True (awaiting [yolo_done])'
        )

        # Trigger Camera 2 localization via direct service call if available
        client_ready = (
            hasattr(self, '_trigger_localization_client')
            and self._trigger_localization_client.service_is_ready()
        )
        if client_ready:
            self.get_logger().info('Calling /depth_camera_node/trigger_localization service...')
            future = self._trigger_localization_client.call_async(Trigger.Request())
            future.add_done_callback(self._on_trigger_localization_response)
        else:
            self.get_logger().debug(
                '/depth_camera_node/trigger_localization not ready; topic flag used.'
            )

        if self._precise_sim_timer is not None:
            self._precise_sim_timer.cancel()
            self._precise_sim_timer = None

        if self._simulate_precise_camera:
            self.get_logger().info(
                'Detect_Precise: simulate_precise_camera=True; '
                'scheduling dummy Camera 2 detection in 0.5s...'
            )
            self._precise_sim_timer = self.create_timer(
                0.5, self._on_simulated_precise_detection
            )
        else:
            self.get_logger().info(
                f'Detect_Precise: awaiting Camera 2 detection '
                f'(watchdog timeout: {self._detect_timeout_sec:.1f}s)...'
            )
            self._detect_watchdog_timer = self.create_timer(
                self._detect_timeout_sec, self._on_detect_precise_timeout
            )

    def _on_simulated_precise_detection(self) -> None:
        """Synthesize precise weed detection with spatial offset."""
        if self._precise_sim_timer is not None:
            self._precise_sim_timer.cancel()
            self._precise_sim_timer = None

        if self._state != State.DETECT_PRECISE or self._approx_loc is None:
            return

        # Compute precise coordinates with slight offset from approx location
        precise_x = self._approx_loc.x + self._precise_offset_x
        precise_y = self._approx_loc.y + self._precise_offset_y
        precise_z = self._approx_loc.z

        # Clamp to arm physical workspace limits
        ws_min = self.arm.workspace_min
        ws_max = self.arm.workspace_max
        precise_x = min(max(precise_x, ws_min[0]), ws_max[0])
        precise_y = min(max(precise_y, ws_min[1]), ws_max[1])
        precise_z = min(max(precise_z, ws_min[2]), ws_max[2])

        precise_pt = Point(x=precise_x, y=precise_y, z=precise_z)
        self.get_logger().info(
            f'[Simulated Camera 2] Detected precise weed location: '
            f'({precise_x:.4f}, {precise_y:.4f}, {precise_z:.4f}) '
            f'[offset: dx={self._precise_offset_x*1000.0:.1f}mm, '
            f'dy={self._precise_offset_y*1000.0:.1f}mm]'
        )
        self._precise_loc_pub.publish(precise_pt)
        self._enter_move_to_precise(precise_pt)

    def _enter_move_to_precise(self, target: Point) -> None:
        """
        Enter Move_to_precise state upon receiving precise coordinates.

        :param target: Precise coordinates aligned with laser aim.
        """
        self._cancel_detect_watchdog()
        self._cancel_arm_watchdog()
        self._publish_flag(self._trigger_yolo_pub, False)

        # Clamp Z to valid arm workspace range (same rationale as
        # _enter_approx_loc).
        ws_min_z = self.arm.workspace_min[2]
        ws_max_z = self.arm.workspace_max[2]
        clamped_z = target.z
        if not (ws_min_z <= clamped_z <= ws_max_z):
            clamped_z = self._target_z_default
        target = Point(x=target.x, y=target.y, z=float(clamped_z))

        self._precise_loc = target
        self._set_state(State.MOVE_TO_PRECISE)

        valid, reason = self.arm.check_workspace(target.x, target.y, target.z)
        if not valid:
            self._on_weed_treatment_failed(
                f'Precise target ({target.x:.3f}, {target.y:.3f}, {target.z:.3f}) '
                f'outside workspace: {reason}'
            )
            return

        self._publish_flag(self._arm_send_goal_pub, True)
        self.get_logger().info(
            f'Move_to_precise: arm_goal_pose=({target.x:.4f}, '
            f'{target.y:.4f}, {target.z:.4f}), arm_send_goal=True'
        )

        def on_precise_move_done(result):
            self._cancel_arm_watchdog()
            self._publish_flag(self._arm_send_goal_pub, False)
            if result is None or result.status != 4:
                status_code = result.status if result else 'REJECTED'
                msg_text = (
                    result.result.msg
                    if (result and hasattr(result, 'result') and result.result)
                    else ''
                )
                self._on_weed_treatment_failed(
                    f'Motion to precise_loc failed with status {status_code}: {msg_text}'
                )
                return

            self.get_logger().info(
                'Move_to_precise: [arm_move_done] -> transitioning to Lasering'
            )
            self._enter_lasering()

        sent = self.arm.move_to_position(
            target.x, target.y, target.z,
            duration_sec=self._duration_sec,
            result_callback=on_precise_move_done,
        )
        self._publish_flag(self._arm_send_goal_pub, False)
        if not sent:
            self._on_weed_treatment_failed('Failed to send goal to precise_loc')
            return

        arm_timeout = self._duration_sec + self._arm_timeout_padding_sec
        self._arm_watchdog_timer = self.create_timer(
            arm_timeout, self._on_arm_timeout
        )

    def _enter_lasering(self) -> None:
        """Enter Lasering state and activate laser for specified duration."""
        self._cancel_arm_watchdog()
        self._cancel_laser_watchdog()
        self._set_state(State.LASERING)
        self._publish_flag(self._laser_trigger_pub, True)
        self.get_logger().info(
            f'Lasering: laser_trigger=True, '
            f'firing for {self._laser_duration_us} us...'
        )

        future = self.laser.trigger_laser(self._laser_duration_us)
        if future is None:
            self.get_logger().error(
                'Laser service unavailable. Returning to Idle.'
            )
            self._exit_lasering(laser_success=False)
            return

        laser_timeout = (
            (self._laser_duration_us / 1e6) + self._laser_timeout_padding_sec
        )
        self._laser_watchdog_timer = self.create_timer(
            laser_timeout, self._on_laser_timeout
        )

        def on_laser_done(res_future):
            self._cancel_laser_watchdog()
            laser_success = False
            try:
                response = res_future.result()
                if response and response.success:
                    laser_success = True
                    self.get_logger().info(
                        f'Lasering succeeded: {response.message}'
                    )
                else:
                    msg = response.message if response else 'No response'
                    self.get_logger().error(f'Lasering failed: {msg}')
            except Exception as e:
                self.get_logger().error(f'Laser call exception: {e}')

            self._exit_lasering(laser_success=laser_success)

        future.add_done_callback(on_laser_done)

    def _exit_lasering(self, laser_success: bool = True) -> None:
        """
        Exit Lasering state and proceed with queue or Idle.

        :param laser_success: Whether the laser activation was successful.
        """
        self._cancel_laser_watchdog()
        self._publish_flag(self._laser_trigger_pub, False)
        self._publish_flag(self._removal_finished_pub, True)

        # Mark weed as removed if lasering succeeded
        if self._current_weed is not None:
            wid = self._current_weed.weed_id
            pos = self._current_weed.position

            if laser_success:
                self.queue_mgr.mark_removed(wid)
                self.get_logger().info(
                    f'Weed ID={wid} successfully marked as REMOVED. '
                    f'Total removed: {self.queue_mgr.removed_count}, '
                    f'Remaining in queue: {self.queue_mgr.queue_size}'
                )

                # Publish weed removed ID
                msg_id = Int32()
                msg_id.data = wid
                self._weed_removed_pub.publish(msg_id)

                # Publish cumulative removed count
                msg_rem = Int32()
                msg_rem.data = self.queue_mgr.removed_count
                self._removed_count_pub.publish(msg_rem)

                # Publish /weed_burned for field recorder compatibility
                if self._weed_burned_pub is not None:
                    w_msg = Weed()
                    w_msg.id = wid
                    w_msg.position_x = pos.x
                    w_msg.position_y = pos.y
                    w_msg.position_z = pos.z
                    self._weed_burned_pub.publish(w_msg)

                self._publish_queue_size()
            else:
                self.get_logger().warn(
                    f'Weed ID={wid} was NOT marked removed due to laser error.'
                )
                self._failed_weed_ids_for_stop.add(wid)

            self._current_weed = None

        # Check if more reachable weeds exist during this stop
        can_continue = (self._robot_stopped or self._auto_start_weeding)
        has_more = len(self._get_reachable_weeds_for_current_stop()) > 0

        if can_continue and has_more:
            self.get_logger().info(
                'More reachable weeds in queue. '
                'Continuing weeding during this stop...'
            )
            self._enter_idle()
            self._check_and_start_next_weed()
        else:
            if can_continue and not has_more:
                self.get_logger().info(
                    'No more reachable weeds in queue for this stop. '
                    'Weeding complete.'
                )
                self._publish_flag(self._all_weeds_treated_pub, True)
            self._enter_idle()

    # --- Topic Callback Handlers for Testing / Injections ---

    def _approx_location_callback(self, msg: Point) -> None:
        """
        Handle manual or simulated approximate weed coordinates.

        :param msg: 3D Point coordinates.
        """
        z = msg.z if msg.z != 0.0 else self._target_z_default
        ws_min_z = self.arm.workspace_min[2]
        ws_max_z = self.arm.workspace_max[2]
        clamped_z = min(max(z, ws_min_z), ws_max_z)
        pos = Point(x=float(msg.x), y=float(msg.y), z=float(clamped_z))

        weed_id = self._mock_weed_id_counter
        self._mock_weed_id_counter -= 1

        self.queue_mgr.add_or_update(weed_id, pos)
        self.get_logger().info(
            f'Manually enqueued weed ID={weed_id} at '
            f'({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}). '
            f'Queue size: {self.queue_mgr.queue_size}'
        )
        self._publish_queue_size()

        stopped_or_auto = self._robot_stopped or self._auto_start_weeding
        if stopped_or_auto and self._state == State.IDLE:
            self._check_and_start_next_weed()

    def _approx_location_pose_callback(self, msg: PoseStamped) -> None:
        """
        Handle incoming approximate weed pose.

        :param msg: PoseStamped containing weed position.
        """
        self._approx_location_callback(msg.pose.position)

    def _precise_location_callback(self, msg: Point) -> None:
        """
        Handle incoming precise coordinates [yolo_done].

        :param msg: Precise Point coordinates.
        """
        if self._state != State.DETECT_PRECISE:
            self.get_logger().warn(
                f'Ignoring yolo_done: machine in state {self._state.value} '
                '(expected Detect_Precise)'
            )
            return

        self._cancel_detect_watchdog()
        self.get_logger().info(
            f'[yolo_done]: precise_loc=({msg.x:.4f}, {msg.y:.4f}, {msg.z:.4f})'
        )
        self._enter_move_to_precise(msg)

    def _precise_location_pose_callback(self, msg: PoseStamped) -> None:
        """
        Handle incoming precise weed pose.

        :param msg: PoseStamped containing precise position.
        """
        self._precise_location_callback(msg.pose.position)


def main(args: Optional[list] = None) -> None:
    """Run the state machine node."""
    rclpy.init(args=args)

    node = StateMachineNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        if 'rcl_shutdown already called' not in str(e):
            raise

    try:
        executor.shutdown()
        node.destroy_node()
    except Exception:
        pass

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
