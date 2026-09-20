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
    Tick-driven FSM. `tick()` is called once per control-loop iteration
    (from a ROS timer). Each state has:
      - an `_update_*` function, called every tick while in that state,
        responsible for checking exit conditions and returning the next
        State (or None to stay).
      - an optional `_enter_*` function, called exactly once when the FSM
        transitions INTO that state - used for one-time side effects
        (publishing a signal, recording a timestamp), so they don't fire
        repeatedly every tick.
    """

    def __init__(self, node: Node):
        self.node = node
        self.state = State.IDLE

        # Re-detection cooldown, distance- or timer-based (see _enter_idle/_update_idle)
        self._cooldown_pending = False
        self._cooldown_active = False
        self._cooldown_origin = None       # used when cooldown_mode == 'distance'
        self._cooldown_start_time = None   # used when cooldown_mode == 'timer'

        self._update_fns = {
            State.IDLE: self._update_idle,
            State.PAUSE: self._update_pause,
            State.TASK_EXE: self._update_task_exe,
            State.RESUMING: self._update_resuming,
            State.ERROR: self._update_error,
        }

        self._enter_fns = {
            State.IDLE: self._enter_idle,
            State.PAUSE: self._enter_pause,
            State.TASK_EXE: self._enter_task_exe,
            State.RESUMING: self._enter_resuming,
            State.ERROR: self._enter_error,
        }

    def tick(self):
        """Call this once per control-loop iteration (e.g. from a timer)."""
        next_state = self._update_fns[self.state]()
        if next_state is not None and next_state != self.state:
            self._transition(next_state)

    def _transition(self, next_state: State):
        self.node.get_logger().info(f'{self.state.name} -> {next_state.name}')
        self.state = next_state
        enter_fn = self._enter_fns.get(next_state)
        if enter_fn:
            enter_fn()

    # -- IDLE (robot navigating, watching for weed detections) -------------
    def _enter_idle(self):
        # If we just came from RESUMING, arm a cooldown so a weed that's
        # still in frame (or a near-neighbor) right after lasering doesn't
        # immediately re-trigger a pause. Any other path into IDLE (startup,
        # ERROR recovery) starts with no cooldown. Mode is set on the node
        # via cooldown_mode: 'distance' or 'timer'.
        if self._cooldown_pending:
            self._cooldown_active = True
            self._cooldown_origin = self.node.current_position       # distance mode
            self._cooldown_start_time = self.node.get_clock().now()  # timer mode
            self._cooldown_pending = False
        else:
            self._cooldown_active = False

    def _update_idle(self):
        if self._cooldown_active:
            if not self._cooldown_elapsed():
                return None  # ignore detections until cooldown clears
            self._cooldown_active = False

        if self.node.weed_detected:
            return State.PAUSE
        return None

    def _cooldown_elapsed(self) -> bool:
        """Returns True once the configured cooldown condition is satisfied."""
        mode = self.node.cooldown_mode

        if mode == 'distance':
            if self._cooldown_origin is None or self.node.current_position is None:
                # No odometry yet - can't measure distance, so don't clear
                # the cooldown blindly. Wait until odom data is available.
                return False
            distance = math.hypot(
                self.node.current_position[0] - self._cooldown_origin[0],
                self.node.current_position[1] - self._cooldown_origin[1],
            )
            return distance >= self.node.min_resume_distance_m

        elif mode == 'timer':
            elapsed = (self.node.get_clock().now() - self._cooldown_start_time).nanoseconds / 1e9
            return elapsed >= self.node.cooldown_duration_sec

        else:
            self.node.get_logger().error(
                f"Unknown cooldown_mode '{mode}', defaulting to no cooldown")
            return True

    # -- PAUSE (call pause service once, then hold for a settle time) ------
    def _enter_pause(self):
        self.node.pause_confirmed = False
        self._pause_start_time = None
        self.node.call_service(self.node.pause_client, 'pause')

    def _update_pause(self):
        if not self.node.pause_confirmed:
            return None  # still waiting on the pause service response

        # Service confirmed pause; start the settle timer once it lands.
        if self._pause_start_time is None:
            self._pause_start_time = self.node.get_clock().now()
            return None

        elapsed = (self.node.get_clock().now() - self._pause_start_time).nanoseconds / 1e9
        if elapsed >= self.node.pause_duration_sec:
            return State.TASK_EXE
        return None

    # -- TASK_EXE (send lasering signal, wait for completion) --------------
    def _enter_task_exe(self):
        self.node.task_done = False  # reset before sending, avoid stale True
        self.node.task_start_pub.publish(Bool(data=True))
        self._task_start_time = self.node.get_clock().now()

    def _update_task_exe(self):
        elapsed = (self.node.get_clock().now() - self._task_start_time).nanoseconds / 1e9
        if self.node.task_done:
            return State.RESUMING
        if elapsed >= self.node.task_timeout_sec:
            self.node.get_logger().error('TASK_EXE timed out waiting for lasering_done')
            return State.ERROR
        return None

    # -- RESUMING (call resume service, wait for confirmation) -------------
    def _enter_resuming(self):
        self.node.resume_confirmed = False
        self.node.weed_detected = False  # clear so IDLE doesn't re-trigger immediately
        self._cooldown_pending = True    # arm the distance cooldown once back in IDLE
        self.node.call_service(self.node.resume_client, 'resume')

    def _update_resuming(self):
        if self.node.resume_confirmed:
            return State.IDLE
        return None  # still waiting on the resume service response

    # -- ERROR ---------------------------------------------------------------
    def _enter_error(self):
        self.node.get_logger().warn('Entered ERROR state')

    def _update_error(self):
        # Replace with your actual recovery condition/logic
        recovered = False
        if recovered:
            return State.IDLE
        return None


class WeedStateMachineNode(Node):
    def __init__(self):
        super().__init__('weed_fsm_node')

        # -- parameters / shared data read by the FSM -----------------------
        self.weed_detected = False
        self.pause_duration_sec = 5.0
        self.task_timeout_sec = 30.0
        self.task_done = False
        self.pause_confirmed = False
        self.resume_confirmed = False
        self.cooldown_mode = 'distance'       # 'distance' or 'timer'
        self.min_resume_distance_m = 1.0      # used when cooldown_mode == 'distance'; TODO: tune to camera FOV/overlap
        self.cooldown_duration_sec = 3.0      # used when cooldown_mode == 'timer'; TODO: tune
        self.current_position = None          # (x, y), updated from odometry

        # -- detection input --------------------------------------------------
        self._detection_sub = self.create_subscription(
            Bool,                   # TODO: replace with your actual detection msg type
            'weed_detection',
            self._on_detection,
            10,
        )

        # -- odometry (for distance-based re-detection cooldown) ----------------
        self._odom_sub = self.create_subscription(
            Odometry,              # TODO: confirm this matches your odom source
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

        # -- FSM + timer --------------------------------------------------------
        self.sm = StateMachine(self)
        timer_period_sec = 0.5
        self.timer = self.create_timer(timer_period_sec, self._on_timer)

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
            return

        if not response.success:
            self.get_logger().error(f'{label} service returned failure: {response.message}')
            return

        if label == 'pause':
            self.pause_confirmed = True
        elif label == 'resume':
            self.resume_confirmed = True

    def _on_timer(self):
        self.sm.tick()

    def _on_detection(self, msg: Bool):
        # Only latch new detections; the FSM's _update_idle() decides
        # whether/when to act on it, and only reads it while in IDLE.
        self.weed_detected = msg.data

    def _on_task_done(self, msg: Bool):
        self.task_done = msg.data

    def _on_odom(self, msg: Odometry):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )


def main(args=None):
    rclpy.init(args=args)
    node = WeedStateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()