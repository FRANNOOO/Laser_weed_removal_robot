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
    # Place at upper x boundary (0.400) and upper y boundary (0.070)
    sm_node._approx_loc = Point(x=0.400, y=0.070, z=-0.100)
    sm_node._precise_offset_x = 0.050  # 450mm would exceed 400mm max
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
