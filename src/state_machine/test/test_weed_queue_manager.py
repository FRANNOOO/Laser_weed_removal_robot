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

    # Attempt to re-enqueue the same weed ID (simulate repeated YOLO detections)
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
