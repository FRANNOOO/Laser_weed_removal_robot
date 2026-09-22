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

"""Unit tests for StateMachineNode dummy camera simulation and TF handling."""

from unittest.mock import MagicMock

from geometry_msgs.msg import Point
import pytest
import rclpy
from state_machine.state_machine_node import State, StateMachineNode


@pytest.fixture(scope='module')
def rclpy_context():
    """Initialize and teardown rclpy context for test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def sm_node(rclpy_context):
    """Create a StateMachineNode instance for testing."""
    node = StateMachineNode()
    # Mock publishers and action clients to avoid live network side-effects
    node._precise_loc_pub = MagicMock()
    node._state_pub = MagicMock()
    node._weeding_active_pub = MagicMock()
    node._ready_for_removal_pub = MagicMock()
    node._laser_trigger_pub = MagicMock()
    node._arm_send_goal_pub = MagicMock()
    node._trigger_yolo_pub = MagicMock()
    node._removal_finished_pub = MagicMock()
    node._all_weeds_treated_pub = MagicMock()
    node.arm.move_to_position = MagicMock(return_value=True)
    yield node
    node.destroy_node()


def test_simulated_camera_default_parameters(sm_node):
    """Verify default parameters for simulated Camera 2."""
    assert sm_node._simulate_precise_camera is True
    assert sm_node._precise_offset_x == pytest.approx(0.003)
    assert sm_node._precise_offset_y == pytest.approx(-0.002)


def test_simulated_precise_detection_applies_offset(sm_node):
    """Verify offset is added and state transitions to MOVE_TO_PRECISE."""
    sm_node._state = State.DETECT_PRECISE
    sm_node._approx_loc = Point(x=0.350, y=0.010, z=-0.100)

    sm_node._on_simulated_precise_detection()

    assert sm_node._state == State.MOVE_TO_PRECISE
    assert sm_node._precise_loc is not None
    assert sm_node._precise_loc.x == pytest.approx(0.353, abs=1e-4)
    assert sm_node._precise_loc.y == pytest.approx(0.008, abs=1e-4)
    assert sm_node._precise_loc.z == pytest.approx(-0.100, abs=1e-4)
    sm_node._precise_loc_pub.publish.assert_called_once()


def test_simulated_precise_detection_clamps_to_workspace(sm_node):
    """Verify precise coordinates are clamped within arm workspace limits."""
    sm_node._state = State.DETECT_PRECISE
    # Place at upper x boundary and test clamping against workspace_max
    sm_node._approx_loc = Point(x=0.400, y=0.070, z=-0.100)
    sm_node._precise_offset_x = 0.050  # 450mm exceeds 400mm max
    sm_node._precise_offset_y = -0.010

    sm_node._on_simulated_precise_detection()

    assert sm_node._precise_loc.x == pytest.approx(0.400, abs=1e-4)
    assert sm_node._precise_loc.y == pytest.approx(0.060, abs=1e-4)


def test_simulated_precise_detection_ignored_when_not_in_detect_state(sm_node):
    """Verify simulation is ignored if state changed before timer fires."""
    sm_node._state = State.IDLE
    sm_node._approx_loc = Point(x=0.350, y=0.0, z=-0.100)

    sm_node._on_simulated_precise_detection()

    assert sm_node._state == State.IDLE
    assert sm_node._precise_loc is None
    sm_node._precise_loc_pub.publish.assert_not_called()


def test_enter_idle_cancels_sim_timer(sm_node):
    """Verify entering Idle cancels active simulated detection timer."""
    mock_timer = MagicMock()
    sm_node._precise_sim_timer = mock_timer

    sm_node._enter_idle()

    mock_timer.cancel.assert_called_once()
    assert sm_node._precise_sim_timer is None


def test_transform_to_base_link_fallback(sm_node):
    """Verify fallback behavior when transforming coordinates to base link."""
    pt = Point(x=0.350, y=0.020, z=-0.100)

    # Matching frame returns same point directly
    same = sm_node._transform_to_base_link(pt, 'robot_base_link')
    assert same.x == pytest.approx(0.350)
    assert same.y == pytest.approx(0.020)

    # Empty source frame returns same point
    empty = sm_node._transform_to_base_link(pt, '')
    assert empty.x == pytest.approx(0.350)

    # Frame with no TF returns point within workspace fallback
    fallback = sm_node._transform_to_base_link(pt, 'unknown_optical_frame')
    assert fallback.x == pytest.approx(0.350)


def test_weed_treatment_failure_publishes_all_weeds_treated(sm_node):
    """Verify failure during motion handles error and signals navigation to resume."""
    sm_node._robot_stopped = True
    sm_node._state = State.IDLE
    weed_pt = Point(x=0.350, y=0.010, z=-0.100)
    sm_node.queue_mgr.add_or_update(42, weed_pt)

    # Start weed treatment
    started = sm_node._check_and_start_next_weed()
    assert started is True
    assert sm_node._current_weed.weed_id == 42

    # Simulate failure (e.g. motion aborted)
    sm_node._on_weed_treatment_failed('Motion aborted by server')

    assert 42 in sm_node._failed_weed_ids_for_stop
    assert sm_node._state == State.IDLE
    assert sm_node._current_weed is None
    # No more reachable weeds -> publishes all_weeds_treated=True
    sm_node._all_weeds_treated_pub.publish.assert_called()


def test_robot_resumed_clears_failed_weeds_for_stop(sm_node):
    """Verify resuming navigation clears the failed weeds set for the next stop."""
    from std_msgs.msg import Bool
    sm_node._failed_weed_ids_for_stop.add(42)

    msg = Bool(data=False)
    sm_node._robot_stopped_callback(msg)

    assert len(sm_node._failed_weed_ids_for_stop) == 0


def test_detect_precise_watchdog_timeout_fallback(sm_node):
    """Verify Camera 2 detection timeout falls back to approx location."""
    sm_node._simulate_precise_camera = False
    sm_node._approx_loc = Point(x=0.350, y=0.010, z=-0.100)
    sm_node._enter_detect_precise()

    assert sm_node._state == State.DETECT_PRECISE
    assert sm_node._detect_watchdog_timer is not None

    # Simulate timeout firing
    sm_node._on_detect_precise_timeout()

    assert sm_node._state == State.MOVE_TO_PRECISE
    assert sm_node._precise_loc.x == pytest.approx(0.350)
    assert sm_node._precise_loc.y == pytest.approx(0.010)
    assert sm_node._detect_watchdog_timer is None


def test_arm_watchdog_timeout_cancels_goal_and_fails_treatment(sm_node):
    """Verify arm motion timeout cancels goal handle and marks treatment as failed."""
    sm_node.arm.cancel_current_goal = MagicMock()
    sm_node._robot_stopped = True
    weed_pt = Point(x=0.350, y=0.010, z=-0.100)
    sm_node.queue_mgr.add_or_update(77, weed_pt)
    sm_node._check_and_start_next_weed()

    assert sm_node._state == State.APPROX_LOC
    assert sm_node._arm_watchdog_timer is not None

    # Simulate arm watchdog timeout
    sm_node._on_arm_timeout()

    sm_node.arm.cancel_current_goal.assert_called_once()
    assert sm_node._arm_watchdog_timer is None
    assert 77 in sm_node._failed_weed_ids_for_stop
    assert sm_node._state == State.IDLE


def test_laser_watchdog_timeout_exits_lasering_as_failure(sm_node):
    """Verify laser timeout exits lasering without marking weed as removed."""
    from rclpy.task import Future
    pending_future = Future()
    sm_node.laser.trigger_laser = MagicMock(return_value=pending_future)

    weed_pt = Point(x=0.350, y=0.010, z=-0.100)
    sm_node.queue_mgr.add_or_update(88, weed_pt)
    sm_node._current_weed = sm_node.queue_mgr.get_weed(88)
    sm_node._enter_lasering()

    assert sm_node._state == State.LASERING
    assert sm_node._laser_watchdog_timer is not None

    # Simulate laser watchdog timeout
    sm_node._on_laser_timeout()

    assert sm_node._laser_watchdog_timer is None
    assert sm_node.queue_mgr.is_removed(88) is False
    assert 88 in sm_node._failed_weed_ids_for_stop
    assert sm_node._state == State.IDLE


def test_enter_idle_cancels_all_watchdogs(sm_node):
    """Verify entering Idle cancels all watchdog timers."""
    t_detect = MagicMock()
    t_arm = MagicMock()
    t_laser = MagicMock()
    t_sim = MagicMock()

    sm_node._detect_watchdog_timer = t_detect
    sm_node._arm_watchdog_timer = t_arm
    sm_node._laser_watchdog_timer = t_laser
    sm_node._precise_sim_timer = t_sim

    sm_node._enter_idle()

    t_detect.cancel.assert_called_once()
    t_arm.cancel.assert_called_once()
    t_laser.cancel.assert_called_once()
    t_sim.cancel.assert_called_once()
    assert sm_node._detect_watchdog_timer is None
    assert sm_node._arm_watchdog_timer is None
    assert sm_node._laser_watchdog_timer is None
    assert sm_node._precise_sim_timer is None


def test_start_stop_weeding_service(sm_node):
    """Verify /start_stop_weeding service callback toggles weeding execution."""
    from std_srvs.srv import SetBool
    req = SetBool.Request()
    res = SetBool.Response()

    # Disable weeding
    req.data = False
    out_res = sm_node._start_stop_weeding_callback(req, res)
    assert out_res.success is True
    assert sm_node._weeding_enabled is False

    # Attempt to start weed when robot is stopped
    sm_node._robot_stopped = True
    sm_node.queue_mgr.add_or_update(99, Point(x=0.35, y=0.0, z=-0.10))
    started = sm_node._check_and_start_next_weed()
    assert started is False
    assert sm_node._state == State.IDLE

    # Enable weeding
    req.data = True
    out_res = sm_node._start_stop_weeding_callback(req, res)
    assert out_res.success is True
    assert sm_node._weeding_enabled is True
    # Enabled callback starts next weed if stopped
    assert sm_node._state == State.APPROX_LOC


def test_telemetry_counters(sm_node):
    """Verify removed_count and failed_count telemetry publishing."""
    sm_node._removed_count_pub = MagicMock()
    sm_node._failed_count_pub = MagicMock()

    # Treatment failure publishes incremented failed_count
    sm_node._current_weed = None
    sm_node._on_weed_treatment_failed('Test failure')
    assert sm_node._failed_count == 1
    sm_node._failed_count_pub.publish.assert_called_once()
    assert sm_node._failed_count_pub.publish.call_args[0][0].data == 1

    # Successful removal publishes updated removed_count
    sm_node.queue_mgr.add_or_update(123, Point(x=0.35, y=0.0, z=-0.10))
    sm_node._current_weed = sm_node.queue_mgr.get_weed(123)
    sm_node._exit_lasering(laser_success=True)
    assert sm_node.queue_mgr.removed_count == 1
    sm_node._removed_count_pub.publish.assert_called_once()
    assert sm_node._removed_count_pub.publish.call_args[0][0].data == 1
