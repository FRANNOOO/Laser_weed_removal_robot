#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from enum import Enum, auto
import math

from enum import Enum
from typing import Optional

from geometry_msgs.msg import Point, PointStamped, PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from state_machine.action_client_arm_control import CartesianActionClient
from state_machine.action_client_laser import ActionClientLaser
from state_machine.weed_queue_manager import QueuedWeed, WeedQueueManager
from std_msgs.msg import Bool, Int32, String
from tf2_ros import Buffer, TransformListener

try:
    from custom_msgs.msg import Weed, WeedInfo
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    CUSTOM_MSGS_AVAILABLE = False

class State(Enum):
    """
    Full mission states (for reference):
      1. HOMING      - automatic homing of the robot
      2. IDLE        - waiting for a command to start the task
      3. NAVIGATING  - navigating to the target location
      4. PAUSE       - pausing the navigation task on weed detection
      5. TASK_EXE    - lasering the detected weed
      6. RESUMING    - resuming the navigation task after lasering
      7. ERROR       - error state (robot stuck, unexpected situation, etc.)
      8. COMPLETED   - task completed successfully

    HOMING/NAVIGATING/COMPLETED are already handled by the behavior tree +
    start/pause/resume buttons. This FSM only owns what happens once a weed
    is detected mid-navigation: IDLE (=navigating normally) -> PAUSE ->
    TASK_EXE -> RESUMING -> back to IDLE, with ERROR as a catch-all.
    """
    IDLE = auto()       # robot navigating normally, watching for detections
    PAUSE = auto()      # holding position before starting the task
    TASK_EXE = auto()   # lasering the detected weed
    RESUMING = auto()   # signaling navigation to resume
    ERROR = auto()


class StateMachine:
    """
    Event-driven FSM. 
    """

    def __init__(self, node: Node):
        self.node = node
        self.state = State.IDLE

        # Re-detection cooldown (distance-based only for now, see _arm_cooldown)
        self._cooldown_pending = False
        self._cooldown_active = False
        self._cooldown_origin = None
        self.task_fsm = None

        self._enter_fns = {
            State.IDLE: self._enter_idle,
            State.PAUSE: self._enter_pause,
            State.TASK_EXE: self._enter_task_exe,
            State.RESUMING: self._enter_resuming,
            State.ERROR: self._enter_error,
        }

        self._enter_fns[self.state]()

    # -- transition plumbing -------------------------------------------------
    def _transition(self, next_state: State):
        self.node.get_logger().info(f'{self.state.name} -> {next_state.name}')
        self.state = next_state
        enter_fn = self._enter_fns.get(next_state)
        if enter_fn:
            enter_fn()


    # -- IDLE (robot navigating, watching for weed detections) -------------
    def _enter_idle(self):
        if self._cooldown_pending:
            self._cooldown_pending = False
            self._arm_cooldown()
        else:
            self._cooldown_active = False

    def _arm_cooldown(self):
        # NOTE: timer-based cooldown mode is temporarily removed along with
        # the other timers (see note above __init__). Distance-based
        # cooldown doesn't need a timer, so it still works as-is.
        self._cooldown_active = True
        self._cooldown_origin = self.node.current_position
        if self._cooldown_origin is None:
            # No odometry yet; stay armed and let on_odom() clear it
            # once position data starts arriving and the distance
            # threshold is met.
            self.node.get_logger().warn(
                'Entering cooldown with no odometry yet; will clear once odom arrives')

    def _maybe_pause_on_pending_detection(self):
        """Call after cooldown clears, in case a detection is already latched."""
        if self.state == State.IDLE and not self._cooldown_active and self.node.weed_detected:
            self._transition(State.PAUSE)

    def on_weed_detected(self):
        """Called from the detection subscription callback."""
        if self.state != State.IDLE:
            return
        if self._cooldown_active:
            return  # ignore detections until cooldown clears
        self._transition(State.PAUSE)

    def on_odom_update(self):
        """Called from the odometry subscription callback."""
        if self.state != State.IDLE or not self._cooldown_active:
            return

        if self._cooldown_origin is None:
            # First odom sample since entering cooldown; use it as the origin.
            self._cooldown_origin = self.node.current_position
            return

        distance = math.hypot(
            self.node.current_position[0] - self._cooldown_origin[0],
            self.node.current_position[1] - self._cooldown_origin[1],
        )
        if distance >= self.node.min_resume_distance_m:
            self._cooldown_active = False
            self._maybe_pause_on_pending_detection()

    # -- PAUSE (call pause service, transition as soon as it confirms) -----
    def _enter_pause(self):
        self.node.pause_confirmed = False
        logger = self.node.get_logger()
        logger.info('Calling pause service, waiting for confirmation...')
        self.node.call_service(self.node.pause_client, 'pause')
        logger.info('Calling pause complete (request sent, awaiting response)')


    def on_pause_confirmed(self):
        """Called from the pause service response callback."""
        if self.state != State.PAUSE:
            return
        self.node.get_logger().info('Pause confirmed, moving to TASK_EXE')
        self._transition(State.TASK_EXE)

    def on_pause_failed(self, message: str):
        if self.state != State.PAUSE:
            return
        self.node.get_logger().error(f'pause service returned failure: {message}')
        self._transition(State.ERROR)

    # -- TASK_EXE (send lasering signal, wait for completion) --------------
    def _enter_task_exe(self):
        self.node.task_done = False  # reset before sending, avoid stale True
        #send a signal for laser state machine to start lasering
        self.task_fsm = LaserStateMachine(self.node.arm_client)
        self.task_fsm._enter_idle(self.node.tracked_weeds_msg)
        self.node.get_logger().info('Published start_lasering, waiting for lasering_done')
        self.node._enter_resuming()

    def on_task_done(self):
        """Called from the lasering_done subscription callback."""
        if self.state != State.TASK_EXE:
            return
        self._transition(State.RESUMING)

    # -- RESUMING (call resume service, wait for confirmation) -------------
    def _enter_resuming(self):
        self.node.resume_confirmed = False
        self.node.weed_detected = False  # clear so IDLE doesn't re-trigger immediately
        self._cooldown_pending = True    # arm the cooldown once back in IDLE
        logger = self.node.get_logger()
        logger.info('Calling resume service, waiting for confirmation...')
        self.node.call_service(self.node.resume_client, 'resume')
        logger.info('Calling resume complete (request sent, awaiting response)')

    def on_resume_confirmed(self):
        """Called from the resume service response callback."""
        if self.state != State.RESUMING:
            return
        self.node.get_logger().info('Resume confirmed, moving to IDLE')
        self._transition(State.IDLE)

    def on_resume_failed(self, message: str):
        if self.state != State.RESUMING:
            return
        self.node.get_logger().error(f'resume service returned failure: {message}')
        self._transition(State.ERROR)

    # -- ERROR ---------------------------------------------------------------
    def _enter_error(self):
        self.node.get_logger().warn('Entered ERROR state')

    def on_recovered(self):
        """Call this (e.g. from an operator-triggered recovery topic/service)
        to leave ERROR. Wire it up to whatever your actual recovery
        condition/trigger ends up being."""
        if self.state != State.ERROR:
            return
        self._transition(State.IDLE)


class WeedStateMachineNode(Node):
    def __init__(self):
        super().__init__('weed_fsm_node')

        # -- parameters / shared data read by the FSM -----------------------
        # NOTE: pause_duration_sec / task_timeout_sec / service_timeout_sec /
        # cooldown_mode='timer' are temporarily unused - all timers were
        # stripped out to isolate the pause/resume service-call bug. Add
        # them back once that's confirmed fixed.
        self.pause_duration_sec = 5.0
        self.weed_detected = False
        self.task_done = False
        self.pause_confirmed = False
        self.resume_confirmed = False
        self.min_resume_distance_m = 1.0      # TODO: tune to camera FOV/overlap
        self.current_position = None          # (x, y), updated from odometry
        self.current_velocity = (0.0, 0.0)     # (vx, vy), updated from odometry

        # -- detection input --------------------------------------------------
        self._detection_sub = self.create_subscription(
            Bool,
            'tracked_weeds',
            self._on_detection,
            10,
        )

        # -- odometry (for distance-based re-detection cooldown) ----------------
        self._odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self._on_odom,
            10,
        )

        # -- pause/resume service clients ---------------------------------------
        self.pause_client = self.create_client(
            Trigger, '/navigate_complete_coverage/pause')
        self.resume_client = self.create_client(
            Trigger, '/navigate_complete_coverage/resume')

        for client, name in [(self.pause_client, 'pause'),
                              (self.resume_client, 'resume')]:
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'Waiting for {name} service...')

        # -- FSM ------------------------------------------------------------
        self.sm = StateMachine(self)

    def call_service(self, client, label):
        request = Trigger.Request()
        future = client.call_async(request)
        future.add_done_callback(
            lambda f, label=label: self._on_response(f, label))

    def _on_response(self, future, label):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'{label} service call failed: {exc}')
            if label == 'pause':
                self.sm.on_pause_failed(str(exc))
            elif label == 'resume':
                self.sm.on_resume_failed(str(exc))
            return

        if not response.success:
            if label == 'pause':
                self.sm.on_pause_failed(response.message)
            elif label == 'resume':
                self.sm.on_resume_failed(response.message)
            return

        if label == 'pause':
            self.pause_confirmed = True
            self.sm.on_pause_confirmed()
        elif label == 'resume':
            self.resume_confirmed = True
            self.sm.on_resume_confirmed()

    def _on_detection(self, msg: Bool):
        self.weed_detected = msg.data
        if msg.data:
            self.get_logger().info('Weed detected')
            self.sm.on_weed_detected()

    def _on_task_done(self, msg: Bool):
        self.task_done = msg.data
        if msg.data:
            self.sm.on_task_done()

    def _on_odom(self, msg: Odometry):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        self.current_velocity = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
        )   
        self.sm.on_odom_update()

#!/usr/bin/env python3
"""
Weed Removal State Machine Node.

Coordinates multi-stage weed removal according to the design diagram:
Idle -> Approx_loc -> Detect_Precise -> Move_to_precise -> Lasering -> Idle.

Integrates a higher-level WeedQueueManager for tracking detected weeds by ID,
preventing duplicate lasering, gating operations on robot stop status, and
iterating through all reachable weeds during a single stop.
"""




class LaserState(str, Enum):
    """Execution states for weed removal state machine."""

    IDLE = 'Idle'
    APPROX_LOC = 'Approx_loc'
    DETECT_PRECISE = 'Detect_Precise'
    MOVE_TO_PRECISE = 'Move_to_precise'
    LASERING = 'Lasering'


class LaserStateMachine():
    """ROS 2 Node coordinating weed removal and multi-weed queue management."""

    def __init__(self, node: Node):
        self.node = node
        self.state = LaserState.IDLE


        # Action and service clients
        self.arm = CartesianActionClient(
            self,
            workspace_min=ws_min,
            workspace_max=ws_max,
            enforce_workspace_limits=enforce_limits,
        ) 
        # Configure the ws_min, ws_max, and enforce_limits parameters as needed for your robot's workspace.
        
        self.laser = ActionClientLaser(self)

        # High-level weed queue manager
        self.queue_mgr = WeedQueueManager()
        self._robot_stopped = self._auto_start_weeding
        self._current_weed: Optional[QueuedWeed] = None
        self._mock_weed_id_counter = -1

        # Internal state & target locations
        self._state = LaserState.IDLE
        self._approx_loc: Optional[Point] = None
        self._precise_loc: Optional[Point] = None
        self._precise_sim_timer = None

        # TF2 listener for frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
    def current_state(self) -> LaserState:
        """Return current state of the state machine."""
        return self._state

    def _check_controllers_timer_callback(self) -> None:
        """Ensure controllers are activated on node start."""
        if self._check_controllers_timer:
            self._check_controllers_timer.cancel()
            self._check_controllers_timer = None
        self.arm.check_and_activate_controllers_async()

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

    def _set_state(self, new_state: LaserState) -> None:
        """Update and publish machine state."""
        self._state = new_state
        msg = String()
        msg.data = new_state.value
        self._state_pub.publish(msg)

        is_active = (new_state != LaserState.IDLE)
        self._publish_flag(self._weeding_active_pub, is_active)
        self.get_logger().info(f'State -> {new_state.value}')

    def transform_point(
        self,
        point: Point,
        source_frame: str,
        target_frame: str
    ) -> Point:
        """
        Transform a 3D Point from source_frame to target_frame.

        Falls back to the input point if frames match or TF is unavailable.
        """
        if not source_frame or source_frame == target_frame:
            return point
        try:
            pt_stamped = PointStamped()
            pt_stamped.header.frame_id = source_frame
            pt_stamped.header.stamp = rclpy.time.Time().to_msg()
            pt_stamped.point = point

            transformed = self.tf_buffer.transform(
                pt_stamped,
                target_frame,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            return transformed.point
        except Exception as ex:
            if 0.20 <= point.x <= 0.60:
                self.get_logger().debug(
                    f'TF transform from {source_frame} to {target_frame} '
                    f'failed ({ex}); using raw point within workspace.'
                )
                return point
            self.get_logger().warning(
                f'Failed to transform weed from {source_frame} to '
                f'{target_frame}: {ex}'
            )
            return point

    def _is_point_reachable(self, pt_odom: Point) -> bool:
        """
        Check if coordinates fall within physical workspace boundaries.

        :param pt_odom: Target 3D coordinates in odom frame.
        :return: True if target is reachable by the arm, False otherwise.
        """
        pt_base = self.transform_point(pt_odom, 'odom', self._robot_base_frame)
        valid, _ = self.arm.check_workspace(pt_base.x, pt_base.y, pt_base.z)
        return valid

    # --- Higher-Level Queue & Stop Coordination ---

    def _check_and_start_next_weed(self) -> bool:
        """
        Check queue for next reachable weed and begin removal sequence.

        :return: True if a weed removal was initiated, False otherwise.
        """
        if not (self._robot_stopped or self._auto_start_weeding):
            self.get_logger().info(
                'Robot moving (robot_stopped=False); waiting to stop.'
            )
            return False

        if self._state != State.IDLE:
            return False

        next_weed = self.queue_mgr.get_next_reachable(self._is_point_reachable)
        if next_weed is None:
            if self.queue_mgr.queue_size > 0:
                self.get_logger().info(
                    f'{self.queue_mgr.queue_size} weed(s) in queue, '
                    'none reachable in workspace.'
                )
            return False

        self._current_weed = next_weed
        
        pt_base = self.transform_point(
            next_weed.position, 'odom', self._robot_base_frame
        )
        
        self.get_logger().info(
            f'Targeting weed ID={next_weed.weed_id} at base_link '
            f'({pt_base.x:.4f}, {pt_base.y:.4f}, {pt_base.z:.4f})'
        )
        self._enter_approx_loc(pt_base)
        return True

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

        if (self._robot_stopped and not prev_stopped and
                self._state == State.IDLE):
            self.get_logger().info(
                'Robot stopped; checking queued weeds for removal.'
            )
            self._check_and_start_next_weed()

    def tracked_weeds_callback(self, msg: WeedInfo) -> None:
        """
        Handle incoming 3D weed coordinates from first camera / YOLO tracker.

        :param msg: WeedInfo message with detected weeds and IDs.
        """
        new_count = 0
        for weed in msg.weeds:
            if self.queue_mgr.is_removed(weed.id):
                continue

            # Assign default z height if 0.0 or outside valid workspace
            z = weed.position_z
            ws_min_z = self.arm.workspace_min[2]
            ws_max_z = self.arm.workspace_max[2]
            if abs(z) < 1e-3 or not (ws_min_z <= z <= ws_max_z):
                z = self._target_z_default

            raw_pt = Point(
                x=float(weed.position_x),
                y=float(weed.position_y),
                z=float(z),
            )
            pos_odom = self.transform_point(raw_pt, msg.header.frame_id, 'odom')

            if self.queue_mgr.add_or_update(weed.id, pos_odom):
                new_count += 1

        if new_count > 0:
            self.get_logger().info(
                f'Enqueued {new_count} new weed(s). '
                f'Total in queue: {self.queue_mgr.queue_size}'
            )
            self._publish_queue_size()

        stopped_or_auto = self._robot_stopped or self._auto_start_weeding
        if stopped_or_auto and self._state == State.IDLE:
            self._check_and_start_next_weed()

    # --- State Transitions ---

    def _enter_idle(self) -> None:
        """Enter Idle state and set diagram signals."""

        self._set_state(State.IDLE)
        self.tracked_weeds_callback(self.tracked_weeds_msg) 

    def _enter_approx_loc(self, target: Point) -> None:
        """
        Enter Approx_loc state upon weed targeting.

        :param target: Approximate coordinates to position the arm.
        """
        self._approx_loc = target
        self._set_state(State.APPROX_LOC)
        self._publish_flag(self._ready_for_removal_pub, False)

        valid, reason = self.arm.check_workspace(target.x, target.y, target.z)
        if not valid:
            self.get_logger().error(
                f'Approx target ({target.x:.3f}, {target.y:.3f}, '
                f'{target.z:.3f}) outside workspace: {reason}'
            )
            self._current_weed = None
            self._enter_idle()
            return

        self._publish_flag(self._arm_send_goal_pub, True)
        self.get_logger().info(
            f'Approx_loc: arm_goal_pose=({target.x:.4f}, {target.y:.4f}, '
            f'{target.z:.4f}), arm_send_goal=True'
        )

        def on_approx_move_done(result):
            self._publish_flag(self._arm_send_goal_pub, False)
            if result is None or result.status != 4:
                status_code = result.status if result else 'REJECTED'
                self.get_logger().error(
                    f'Motion to approx_loc failed with status {status_code}. '
                    'Returning to Idle.'
                )
                self._current_weed = None
                self._enter_idle()
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
            self.get_logger().error(
                'Failed to send goal to approx_loc. Returning to Idle.'
            )
            self._current_weed = None
            self._enter_idle()

    def _enter_detect_precise(self) -> None:
        """Enter Detect_Precise state upon arm arrival at approx position."""
        self._set_state(State.DETECT_PRECISE)
        self._publish_flag(self._trigger_yolo_pub, True)
        self.get_logger().info(
            'Detect_Precise: trigger_yolo=True (awaiting [yolo_done])'
        )
        self._publish_flag(self._trigger_yolo_pub, False)

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
        self._precise_loc = target
        self._set_state(State.MOVE_TO_PRECISE)

        valid, reason = self.arm.check_workspace(target.x, target.y, target.z)
        if not valid:
            self.get_logger().error(
                f'Precise target ({target.x:.3f}, {target.y:.3f}, '
                f'{target.z:.3f}) outside workspace: {reason}'
            )
            self._current_weed = None
            self._enter_idle()
            return

        self._publish_flag(self._arm_send_goal_pub, True)
        self.get_logger().info(
            f'Move_to_precise: arm_goal_pose=({target.x:.4f}, '
            f'{target.y:.4f}, {target.z:.4f}), arm_send_goal=True'
        )

        def on_precise_move_done(result):
            self._publish_flag(self._arm_send_goal_pub, False)
            if result is None or result.status != 4:
                status_code = result.status if result else 'REJECTED'
                self.get_logger().error(
                    f'Motion to precise_loc failed with status {status_code}. '
                    'Returning to Idle.'
                )
                self._current_weed = None
                self._enter_idle()
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
            self.get_logger().error(
                'Failed to send goal to precise_loc. Returning to Idle.'
            )
            self._current_weed = None
            self._enter_idle()

    def _enter_lasering(self) -> None:
        """Enter Lasering state and activate laser for specified duration."""
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

        def on_laser_done(res_future):
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

            self._current_weed = None

        # Check if more reachable weeds exist during this stop
        can_continue = (self._robot_stopped or self._auto_start_weeding)
        has_more = self.queue_mgr.has_reachable_weeds(self._is_point_reachable)

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

        # Transform manual input (assumed base_link) to odom
        pos_odom = self.transform_point(pos, self._robot_base_frame, 'odom')

        weed_id = self._mock_weed_id_counter
        self._mock_weed_id_counter -= 1

        self.queue_mgr.add_or_update(weed_id, pos_odom)
        self.get_logger().info(
            f'Manually enqueued weed ID={weed_id} at odom '
            f'({pos_odom.x:.3f}, {pos_odom.y:.3f}, {pos_odom.z:.3f}). '
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







def main(args=None):
    rclpy.init(args=args)
    node = WeedStateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()