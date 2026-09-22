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

"""Unit tests for WeedQueueManager."""

from geometry_msgs.msg import Point

import pytest

from state_machine.weed_queue_manager import WeedQueueManager


@pytest.fixture
def queue_mgr():
    """Create a fresh WeedQueueManager instance."""
    return WeedQueueManager()


def test_enqueue_and_size(queue_mgr):
    """Test enqueuing new weeds and checking queue size."""
    p1 = Point(x=0.32, y=0.01, z=-0.10)
    p2 = Point(x=0.35, y=-0.02, z=-0.11)

    assert queue_mgr.add_or_update(101, p1) is True
    assert queue_mgr.add_or_update(102, p2) is True
    assert queue_mgr.queue_size == 2
    assert queue_mgr.is_queued(101) is True
    assert queue_mgr.is_queued(102) is True
    assert queue_mgr.is_queued(999) is False


def test_update_existing_weed(queue_mgr):
    """Test updating position of already queued weed."""
    p1 = Point(x=0.32, y=0.01, z=-0.10)
    queue_mgr.add_or_update(101, p1)

    p1_updated = Point(x=0.33, y=0.015, z=-0.10)
    # Updating returns False (not new)
    assert queue_mgr.add_or_update(101, p1_updated) is False
    assert queue_mgr.queue_size == 1
    assert queue_mgr.get_weed(101).position.x == pytest.approx(0.33)


def test_mark_removed_and_prevent_duplicates(queue_mgr):
    """Test marking weed as removed and ensuring it cannot be re-enqueued."""
    p1 = Point(x=0.32, y=0.01, z=-0.10)
    queue_mgr.add_or_update(101, p1)

    # Mark as removed
    assert queue_mgr.mark_removed(101) is True
    assert queue_mgr.queue_size == 0
    assert queue_mgr.removed_count == 1
    assert queue_mgr.is_removed(101) is True

    # Attempt to re-enqueue the same weed ID (simulate repeated detections)
    assert queue_mgr.add_or_update(101, p1) is False
    assert queue_mgr.queue_size == 0
    assert queue_mgr.is_removed(101) is True


def test_get_next_reachable(queue_mgr):
    """Test filtering next reachable weed based on workspace limits."""
    # Weed 1: outside workspace (x=0.20, min is 0.29)
    # Weed 2: inside workspace (x=0.32)
    # Weed 3: inside workspace (x=0.36)
    p_unreachable = Point(x=0.20, y=0.0, z=-0.10)
    p_reachable_1 = Point(x=0.32, y=0.01, z=-0.10)
    p_reachable_2 = Point(x=0.36, y=-0.01, z=-0.11)

    queue_mgr.add_or_update(1, p_unreachable)
    queue_mgr.add_or_update(2, p_reachable_1)
    queue_mgr.add_or_update(3, p_reachable_2)

    def is_in_ws(pt):
        return 0.29 <= pt.x <= 0.40 and -0.07 <= pt.y <= 0.07

    assert queue_mgr.has_reachable_weeds(is_in_ws) is True
    next_weed = queue_mgr.get_next_reachable(is_in_ws)
    assert next_weed is not None
    assert next_weed.weed_id == 2

    # Remove weed 2 and check that next reachable is weed 3
    queue_mgr.mark_removed(2)
    next_weed = queue_mgr.get_next_reachable(is_in_ws)
    assert next_weed is not None
    assert next_weed.weed_id == 3

    # Remove weed 3
    queue_mgr.mark_removed(3)
    # Only unreachable weed 1 remains
    assert queue_mgr.queue_size == 1
    assert queue_mgr.has_reachable_weeds(is_in_ws) is False
    assert queue_mgr.get_next_reachable(is_in_ws) is None


def test_get_oldest_queued_weed(queue_mgr):
    """Test retrieving oldest queued weed in FIFO order."""
    assert queue_mgr.get_oldest_queued_weed() is None
    assert queue_mgr.get_all_active_weeds() == []

    p1 = Point(x=0.38, y=0.01, z=-0.10)
    p2 = Point(x=0.35, y=-0.02, z=-0.10)
    p3 = Point(x=0.32, y=0.03, z=-0.10)

    queue_mgr.add_or_update(101, p1)
    queue_mgr.add_or_update(102, p2)
    queue_mgr.add_or_update(103, p3)

    oldest = queue_mgr.get_oldest_queued_weed()
    assert oldest is not None
    assert oldest.weed_id == 101
    assert oldest.position.x == 0.38

    # Mark 101 removed, oldest should become 102
    queue_mgr.mark_removed(101)
    oldest = queue_mgr.get_oldest_queued_weed()
    assert oldest is not None
    assert oldest.weed_id == 102

    active = queue_mgr.get_all_active_weeds()
    assert len(active) == 2
    assert [w.weed_id for w in active] == [102, 103]


def test_update_positions_tf_and_dead_reckoning(queue_mgr):
    """Test dynamic coordinate updating via TF and dead-reckoning fallback."""
    raw_pt = Point(x=10.0, y=5.0, z=-0.10)
    base_pos = Point(x=0.42, y=0.01, z=-0.10)

    # Enqueue weed with raw_position, raw_frame_id, and odom_pose
    queue_mgr.add_or_update(
        55,
        base_pos,
        raw_position=raw_pt,
        raw_frame_id='map',
        odom_pose=(0.0, 0.0, 0.0),
    )

    # 1. TF transform function succeeds
    def mock_tf(pt, frame):
        if frame == 'map':
            return Point(x=0.36, y=0.01, z=-0.10)
        return None

    queue_mgr.update_positions(
        tf_transform_fn=mock_tf, current_odom_pose=(0.05, 0.0, 0.0)
    )
    assert queue_mgr.get_weed(55).position.x == pytest.approx(0.36)

    # 2. TF transform fails, dead-reckoning fallback is used
    # Vehicle has advanced from (0.05, 0, 0) to (0.10, 0, 0)
    # In robot_base_link, advancing forward (+dx_robot) increases weed X:
    # 0.36 + (0.10 - 0.05) = 0.41
    queue_mgr.update_positions(
        tf_transform_fn=lambda pt, frame: None,
        current_odom_pose=(0.10, 0.0, 0.0),
    )
    assert queue_mgr.get_weed(55).position.x == pytest.approx(0.41)


def test_purge_unreachable(queue_mgr):
    """Test purging weeds that moved past max workspace threshold."""
    queue_mgr.add_or_update(1, Point(x=0.10, y=0.0, z=-0.10))
    queue_mgr.add_or_update(2, Point(x=0.45, y=0.0, z=-0.10))

    purged = queue_mgr.purge_unreachable(max_x_threshold=0.42)
    assert purged == 1
    assert queue_mgr.is_queued(1) is True
    assert queue_mgr.is_queued(2) is False
    # Purged weeds are marked removed so they cannot be resurrected
    assert queue_mgr.is_removed(2) is True
    assert queue_mgr.add_or_update(2, Point(x=0.45, y=0.0, z=-0.10)) is False
    assert queue_mgr.is_queued(2) is False


def test_update_existing_weed_anchor_with_odom(queue_mgr):
    """Test that re-detecting a weed updates initial_position anchor for dead-reckoning."""
    # First detection at robot_base_link x=0.20 when odom=(0.0, 0.0, 0.0)
    queue_mgr.add_or_update(
        10, Point(x=0.20, y=0.0, z=-0.10), odom_pose=(0.0, 0.0, 0.0)
    )

    # Robot moves forward 0.10m -> weed position advances to 0.30m
    queue_mgr.update_positions(None, current_odom_pose=(0.10, 0.0, 0.0))
    assert queue_mgr.get_weed(10).position.x == pytest.approx(0.30)

    # New perception measurement arrives at x=0.31m when robot is at odom=(0.10, 0.0, 0.0)
    queue_mgr.add_or_update(
        10, Point(x=0.31, y=0.0, z=-0.10), odom_pose=(0.10, 0.0, 0.0)
    )
    assert queue_mgr.get_weed(10).position.x == pytest.approx(0.31)
    assert queue_mgr.get_weed(10).initial_position.x == pytest.approx(0.31)

    # Robot moves forward another 0.05m (to odom=0.15m)
    # Displacement from new anchor is (0.15 - 0.10) = 0.05m
    # Weed position should now be 0.31 + 0.05 = 0.36m (NOT jumping back to 0.20 + 0.15 = 0.35m)
    queue_mgr.update_positions(None, current_odom_pose=(0.15, 0.0, 0.0))
    assert queue_mgr.get_weed(10).position.x == pytest.approx(0.36)


def test_get_oldest_reachable_candidate(queue_mgr):
    """Test retrieving oldest candidate that has not passed max threshold."""
    # Weed 1 passed past back edge
    queue_mgr.add_or_update(1, Point(x=0.46, y=0.0, z=-0.10))
    # Weed 2 is reachable near back edge
    queue_mgr.add_or_update(2, Point(x=0.38, y=0.0, z=-0.10))
    # Weed 3 is approaching front edge
    queue_mgr.add_or_update(3, Point(x=0.15, y=0.0, z=-0.10))

    candidate = queue_mgr.get_oldest_reachable_candidate(max_x_threshold=0.42)
    assert candidate is not None
    assert candidate.weed_id == 2


def test_get_oldest_reachable_candidate_with_lateral_bounds(queue_mgr):
    """Test retrieving oldest reachable candidate with lateral [min_y, max_y] bounds."""
    # Weed 1 is within X threshold but outside positive lateral bound
    queue_mgr.add_or_update(1, Point(x=0.38, y=0.12, z=-0.10))
    # Weed 2 is within X threshold and within lateral bounds
    queue_mgr.add_or_update(2, Point(x=0.36, y=-0.03, z=-0.10))
    # Weed 3 is within X threshold but outside negative lateral bound
    queue_mgr.add_or_update(3, Point(x=0.35, y=-0.15, z=-0.10))

    # Without Y filtering, weed 1 would be selected
    assert queue_mgr.get_oldest_reachable_candidate(0.40).weed_id == 1

    # With Y filtering [-0.07, 0.07], weed 2 should be selected
    cand = queue_mgr.get_oldest_reachable_candidate(
        0.40, min_y=-0.07, max_y=0.07
    )
    assert cand is not None
    assert cand.weed_id == 2


def test_spatial_deduplication():
    """Test spatial deduplication against nearby existing queued weeds."""
    mgr = WeedQueueManager(dedup_radius=0.03)

    # First weed detection with ID 1 at (0.35, 0.01)
    is_new = mgr.add_or_update(1, Point(x=0.35, y=0.01, z=-0.10))
    assert is_new is True
    assert mgr.queue_size == 1

    # Second detection with different ID 2 at (0.355, 0.015) (distance ~0.007m < 0.03m)
    # Should update existing weed 1 and NOT create a new entry
    is_new = mgr.add_or_update(2, Point(x=0.355, y=0.015, z=-0.10))
    assert is_new is False
    assert mgr.queue_size == 1
    assert mgr.get_weed(1).position.x == pytest.approx(0.355)

    # Third detection with ID 3 at (0.35, 0.08) (distance ~0.065m > 0.03m)
    # Should create a separate queued weed
    is_new = mgr.add_or_update(3, Point(x=0.35, y=0.08, z=-0.10))
    assert is_new is True
    assert mgr.queue_size == 2
