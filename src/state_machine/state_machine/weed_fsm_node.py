#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from enum import Enum, auto
import math


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
    Event-driven FSM. There is no polling tick and, for now, no timers at
    all: every transition is triggered directly by the node callback that
    produced the relevant event (a subscription message or a service
    response). Timers (watchdogs, pause settle delay) were removed to
    isolate a pause/resume-only-works-once bug - see the note below.
    """

    def __init__(self, node: Node):
        self.node = node
        self.state = State.IDLE

        # Re-detection cooldown (distance-based only for now, see _arm_cooldown)
        self._cooldown_pending = False
        self._cooldown_active = False
        self._cooldown_origin = None

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

    # NOTE: all timers (watchdogs, pause settle delay, timer-based cooldown)
    # have been stripped out for now to isolate whether the pause/resume
    # service call logic itself works correctly on repeat cycles. Once
    # that's confirmed, we can reintroduce them one at a time.

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
        self.node.task_start_pub.publish(Bool(data=True))
        self.node.get_logger().info('Published start_lasering, waiting for lasering_done')

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

        # -- task (lasering) start/done topics ---------------------------------
        self.task_start_pub = self.create_publisher(Bool, 'start_lasering', 10)
        self._task_done_sub = self.create_subscription(
            Bool, 'lasering_done', self._on_task_done, 10
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