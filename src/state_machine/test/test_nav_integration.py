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

"""Unit tests for NavigationCoordinatorStateMachine and trigger logic."""

from unittest.mock import MagicMock

from geometry_msgs.msg import Point

import pytest

from state_machine.nav_integration_node import (
    NavigationCoordinatorStateMachine,
    NavigationState,
)
from state_machine.weed_queue_manager import WeedQueueManager


class MockLogger:
    """Mock logger recording messages."""

    def __init__(self) -> None:
        """Initialize empty message records."""
        self.infos = []
        self.warns = []
        self.errors = []
        self.debugs = []

    def info(self, msg: str) -> None:
        """Record info message."""
        self.infos.append(msg)

    def warn(self, msg: str) -> None:
        """Record warn message."""
        self.warns.append(msg)

    def error(self, msg: str) -> None:
        """Record error message."""
        self.errors.append(msg)

    def debug(self, msg: str) -> None:
        """Record debug message."""
        self.debugs.append(msg)


class MockCoordinatorNode:
    """Mock coordinator node for testing the state machine in isolation."""

    def __init__(self) -> None:
        """Initialize mock node attributes and publishers."""
        self.logger = MockLogger()
        self.queue_mgr = WeedQueueManager()

        # Geometry and thresholds
        self.workspace_min_x = 0.290
        self.workspace_max_x = 0.400
        self.workspace_min_y = -0.070
        self.workspace_max_y = 0.070
        self.back_edge_margin_x = 0.040
        self.use_velocity_lookahead = True
        self.stop_delay_sec = 0.80
        self.min_resume_distance_m = 0.05
        self.task_timeout_sec = 120.0

        # Robot state
        self.current_position = (0.0, 0.0)
        self.current_velocity = (0.0, 0.0)
        self.current_yaw = 0.0
        self.transform_to_base_link = MagicMock(side_effect=lambda pt, frame: pt)

        # Service clients
        self.pause_client = MagicMock()
        self.resume_client = MagicMock()

        # Recorded published values
        self.published_states = []
        self.published_nav_paused = []
        self.published_robot_stopped = []
        self.published_cooldown = []
        self.published_trigger_x = []
        self.published_oldest_weed_x = []
        self.published_start_lasering = []

        # Watchdog mocks
        self.watchdog_started = False
        self.watchdog_stopped = False

    def get_logger(self) -> MockLogger:
        """Return the mock logger."""
        return self.logger

    def publish_state(self, state: str) -> None:
        """Record published state string."""
        self.published_states.append(state)

    def publish_nav_paused(self, paused: bool) -> None:
        """Record published nav_paused boolean."""
        self.published_nav_paused.append(paused)

    def publish_robot_stopped(self, stopped: bool) -> None:
        """Record published robot_stopped boolean."""
        self.published_robot_stopped.append(stopped)

    def publish_cooldown_active(self, active: bool) -> None:
        """Record published cooldown boolean."""
        self.published_cooldown.append(active)

    def publish_stop_trigger_x(self, x: float) -> None:
        """Record published stop_trigger_x float."""
        self.published_trigger_x.append(x)

    def publish_oldest_weed_x(self, x: float) -> None:
        """Record published oldest_weed_x float."""
        self.published_oldest_weed_x.append(x)

    def publish_start_lasering(self, start: bool) -> None:
        """Record published start_lasering boolean."""
        self.published_start_lasering.append(start)

    def publish_zero_cmd_vel(self) -> None:
        """Simulate zero velocity command."""
        pass

    def call_trigger_service(self, client, name: str) -> None:
        """Simulate service call recording."""
        pass

    def call_pause_services(self) -> None:
        """Simulate pause service call."""
        pass

    def call_resume_services(self) -> None:
        """Simulate resume service call."""
        pass

    def start_hold_position(self) -> None:
        """Simulate starting hold position."""
        pass

    def stop_hold_position(self) -> None:
        """Simulate stopping hold position."""
        pass

    def start_task_watchdog(self) -> None:
        """Mark watchdog as started."""
        self.watchdog_started = True

    def stop_task_watchdog(self) -> None:
        """Mark watchdog as stopped."""
        self.watchdog_stopped = True

    def create_timer(self, period: float, callback) -> MagicMock:
        """Return a mock timer (does not fire automatically in tests)."""
        timer = MagicMock()
        timer.cancel = MagicMock()
        return timer


@pytest.fixture
def mock_node():
    """Fixture providing a configured MockCoordinatorNode."""
    return MockCoordinatorNode()


@pytest.fixture
def coordinator_sm(mock_node):
    """Fixture providing an initialized NavigationCoordinatorStateMachine."""
    return NavigationCoordinatorStateMachine(mock_node)


def test_compute_stop_trigger_x(coordinator_sm, mock_node):
    """Test stop trigger X calculation with various velocities."""
    # Stationary: max_x - margin = 0.400 - 0.040 = 0.360
    assert coordinator_sm.compute_stop_trigger_x(0.0) == pytest.approx(0.360)

    # Moving forward at 0.1 m/s: 0.360 - (0.1 * 0.8) = 0.280
    assert coordinator_sm.compute_stop_trigger_x(0.10) == pytest.approx(0.280)

    # Moving backward or zero velocity: no negative lookahead
    assert coordinator_sm.compute_stop_trigger_x(-0.05) == pytest.approx(0.360)

    # Very fast forward: clamped to min_trigger_x (0.150)
    assert coordinator_sm.compute_stop_trigger_x(1.0) == pytest.approx(0.150)

    # Disabled lookahead
    mock_node.use_velocity_lookahead = False
    assert coordinator_sm.compute_stop_trigger_x(0.10) == pytest.approx(0.360)


def test_back_edge_trigger_empty_queue(coordinator_sm):
    """Test back-edge check when queue is empty."""
    should_stop, weed_x, trig_x = coordinator_sm.check_back_edge_trigger()
    assert should_stop is False
    assert weed_x is None
    assert trig_x == pytest.approx(0.360)


def test_back_edge_trigger_weed_at_front_edge(coordinator_sm, mock_node):
    """Test that a weed at front edge (0.150m) does NOT trigger stopping."""
    mock_node.queue_mgr.add_or_update(1, Point(x=0.150, y=0.0, z=-0.10))

    should_stop, weed_x, trig_x = coordinator_sm.check_back_edge_trigger()
    assert should_stop is False
    assert weed_x == pytest.approx(0.150)
    assert trig_x == pytest.approx(0.360)

    # Triggering on_weed_detected should remain in IDLE
    coordinator_sm.on_weed_detected()
    assert coordinator_sm.state == NavigationState.IDLE


def test_back_edge_trigger_ignores_weeds_past_workspace(coordinator_sm, mock_node):
    """Test that weeds that have already moved past workspace_max_x do NOT trigger stopping."""
    # Weed 1 is at 0.430m (beyond workspace_max_x = 0.415)
    mock_node.queue_mgr.add_or_update(1, Point(x=0.430, y=0.0, z=-0.10))

    should_stop, weed_x, trig_x = coordinator_sm.check_back_edge_trigger()
    assert should_stop is False
    assert weed_x is None

    coordinator_sm.on_weed_detected()
    assert coordinator_sm.state == NavigationState.IDLE


def test_multi_weed_queue_and_back_edge_stop(coordinator_sm, mock_node):
    """Test queueing weeds and stopping only when oldest reaches back edge."""
    # Weed 1 enters at 0.15m (camera view)
    mock_node.queue_mgr.add_or_update(10, Point(x=0.150, y=0.01, z=-0.10))
    coordinator_sm.on_weed_detected()
    assert coordinator_sm.state == NavigationState.IDLE

    # Weed 2 enters behind it at 0.12m
    mock_node.queue_mgr.add_or_update(11, Point(x=0.120, y=-0.02, z=-0.10))
    coordinator_sm.on_weed_detected()
    assert coordinator_sm.state == NavigationState.IDLE
    assert mock_node.queue_mgr.queue_size == 2

    # Oldest weed (10) approaches back edge (x=0.365m >= trig_x=0.360m)
    mock_node.queue_mgr.add_or_update(10, Point(x=0.365, y=0.01, z=-0.10))
    coordinator_sm.on_weed_detected()

    # Should transition to PAUSE to stop robot
    assert coordinator_sm.state == NavigationState.PAUSE
    assert mock_node.published_nav_paused[-1] is True


def test_full_fsm_cycle(coordinator_sm, mock_node):
    """Test full cycle: IDLE -> PAUSE -> TASK_EXE -> RESUMING -> IDLE."""
    # 1. Enqueue weed at back edge
    mock_node.queue_mgr.add_or_update(1, Point(x=0.365, y=0.0, z=-0.10))
    coordinator_sm.on_weed_detected()
    assert coordinator_sm.state == NavigationState.PAUSE

    # 2. Pause confirmed by Nav2
    coordinator_sm.on_pause_confirmed()
    assert coordinator_sm.state == NavigationState.TASK_EXE
    assert mock_node.published_robot_stopped[-1] is True
    assert mock_node.published_start_lasering[-1] is True
    assert mock_node.watchdog_started is True

    # 3. Weeds treated, task done
    coordinator_sm.on_task_done()
    assert coordinator_sm.state == NavigationState.RESUMING
    assert mock_node.watchdog_stopped is True
    assert mock_node.published_robot_stopped[-1] is False

    # 4. Resume confirmed by Nav2
    coordinator_sm.on_resume_confirmed()
    assert coordinator_sm.state == NavigationState.IDLE
    assert coordinator_sm.is_cooldown_active is True
    assert mock_node.published_cooldown[-1] is True


def test_odometry_cooldown_distance(coordinator_sm, mock_node):
    """Test cooldown prevents re-triggering until min distance traveled."""
    # Simulate completion and entry into cooldown at position (1.0, 2.0)
    mock_node.current_position = (1.0, 2.0)
    coordinator_sm._transition(NavigationState.PAUSE)
    coordinator_sm.on_pause_confirmed()
    coordinator_sm.on_task_done()
    coordinator_sm.on_resume_confirmed()

    assert coordinator_sm.state == NavigationState.IDLE
    assert coordinator_sm.is_cooldown_active is True

    # Add a weed near back edge during cooldown (>= 0.360m)
    mock_node.queue_mgr.add_or_update(20, Point(x=0.365, y=0.0, z=-0.10))
    coordinator_sm.on_weed_detected()
    # Must NOT stop during cooldown
    assert coordinator_sm.state == NavigationState.IDLE

    # Robot moves 0.02m (< 0.05m required)
    mock_node.current_position = (1.02, 2.0)
    coordinator_sm.on_odom_update()
    assert coordinator_sm.is_cooldown_active is True
    assert coordinator_sm.state == NavigationState.IDLE

    # Robot moves past 0.05m (total 0.06m from origin)
    mock_node.current_position = (1.06, 2.0)
    coordinator_sm.on_odom_update()

    # Cooldown cleared and pending weed evaluated -> PAUSE triggered!
    assert coordinator_sm.is_cooldown_active is False
    assert coordinator_sm.state == NavigationState.PAUSE


def test_watchdog_timeout_leads_to_error(coordinator_sm, mock_node):
    """Test that weed removal watchdog timeout transitions to ERROR state."""
    mock_node.queue_mgr.add_or_update(1, Point(x=0.385, y=0.0, z=-0.10))
    coordinator_sm.on_weed_detected()
    coordinator_sm.on_pause_confirmed()
    assert coordinator_sm.state == NavigationState.TASK_EXE

    coordinator_sm.on_task_timeout()
    assert coordinator_sm.state == NavigationState.ERROR
    assert 'ERROR' in mock_node.published_states


def test_service_failures_lead_to_error(coordinator_sm, mock_node):
    """Test that Nav2 service failures transition to ERROR state."""
    mock_node.queue_mgr.add_or_update(1, Point(x=0.385, y=0.0, z=-0.10))
    coordinator_sm.on_weed_detected()
    assert coordinator_sm.state == NavigationState.PAUSE

    # Pause service failed
    coordinator_sm.on_pause_failed('Navigation service timeout')
    assert coordinator_sm.state == NavigationState.ERROR

    # Reset and test resume service failure
    coordinator_sm.state = NavigationState.RESUMING
    coordinator_sm.on_resume_failed('Resume rejected')
    assert coordinator_sm.state == NavigationState.ERROR


def test_lateral_unreachable_weeds_do_not_trigger_stop(coordinator_sm, mock_node):
    """Test that weeds outside lateral workspace bounds [min_y, max_y] do not trigger stop."""
    # Weed at x=0.385 (near back edge), but lateral y=0.150 is outside [-0.070, 0.070]
    mock_node.queue_mgr.add_or_update(1, Point(x=0.385, y=0.150, z=-0.10))
    coordinator_sm.on_weed_detected()
    # Must remain IDLE because candidate is laterally unreachable
    assert coordinator_sm.state == NavigationState.IDLE

    # Now add weed within lateral reach [-0.070, 0.070]
    mock_node.queue_mgr.add_or_update(2, Point(x=0.385, y=0.020, z=-0.10))
    coordinator_sm.on_weed_detected()
    # Must transition to PAUSE
    assert coordinator_sm.state == NavigationState.PAUSE
