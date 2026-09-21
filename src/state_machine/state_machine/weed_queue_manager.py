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
Weed Queue Manager for multi-weed laser removal coordination.

Maintains a queue of detected weeds, prevents duplicate lasering by tracking
removed weed IDs, and coordinates sequential targeting of reachable weeds.
"""

from dataclasses import dataclass
import time
from typing import Callable, Dict, List, Optional, Set

from geometry_msgs.msg import Point


@dataclass
class QueuedWeed:
    """Represents a weed awaiting removal in the queue."""

    weed_id: int
    position: Point
    timestamp: float


class WeedQueueManager:
    """
    Higher-level manager for detected weed queue and removal state.

    Ensures weeds are only processed once, tracks already-lasered IDs,
    and queries weeds reachable within the arm's workspace.
    """

    def __init__(self) -> None:
        """Initialize empty queue and removal history."""
        self._queue: Dict[int, QueuedWeed] = {}
        self._removed_ids: Set[int] = set()

    @property
    def queue_size(self) -> int:
        """Return the number of weeds currently waiting in the queue."""
        return len(self._queue)

    @property
    def removed_count(self) -> int:
        """Return the number of weeds marked as removed."""
        return len(self._removed_ids)

    def is_removed(self, weed_id: int) -> bool:
        """
        Check if a weed ID has already been removed.

        :param weed_id: Unique integer identifier of the weed.
        :return: True if the weed has already been lasered/removed.
        """
        return weed_id in self._removed_ids

    def is_queued(self, weed_id: int) -> bool:
        """
        Check if a weed ID is currently queued for removal.

        :param weed_id: Unique integer identifier of the weed.
        :return: True if the weed is currently in queue, False otherwise.
        """
        return weed_id in self._queue

    def add_or_update(
        self,
        weed_id: int,
        position: Point,
        timestamp: Optional[float] = None
    ) -> bool:
        """
        Add newly detected weed or update position of existing weed.

        If the weed has already been marked as removed, it is ignored.

        :param weed_id: Unique integer identifier of the weed.
        :param position: 3D coordinates (Point) in the robot base frame.
        :param timestamp: Detection timestamp (defaults to system time).
        :return: True if new weed added to queue, False if ignored or updated.
        """
        if self.is_removed(weed_id):
            return False

        ts = timestamp if timestamp is not None else time.time()

        if weed_id in self._queue:
            # Update existing weed position and timestamp
            self._queue[weed_id].position = position
            self._queue[weed_id].timestamp = ts
            return False

        self._queue[weed_id] = QueuedWeed(
            weed_id=weed_id,
            position=position,
            timestamp=ts,
        )
        return True

    def mark_removed(self, weed_id: int) -> bool:
        """
        Mark a weed as removed.

        Removes weed from active queue and adds ID to removed set,
        guaranteeing it will never be lasered again.

        :param weed_id: Unique integer identifier of the weed.
        :return: True if the weed was recorded as removed.
        """
        self._queue.pop(weed_id, None)
        self._removed_ids.add(weed_id)
        return True

    def get_weed(self, weed_id: int) -> Optional[QueuedWeed]:
        """
        Retrieve a queued weed by its ID without removing it.

        :param weed_id: Unique integer identifier of the weed.
        :return: QueuedWeed instance if present, None otherwise.
        """
        return self._queue.get(weed_id)

    def get_oldest_queued_weed(self) -> Optional[QueuedWeed]:
        """
        Return the oldest unremoved weed in the queue (FIFO order).

        :return: Oldest QueuedWeed instance if not empty, None otherwise.
        """
        if not self._queue:
            return None
        return next(iter(self._queue.values()))

    def get_all_active_weeds(self) -> List[QueuedWeed]:
        """
        Return list of all active unremoved weeds currently in queue.

        :return: List of QueuedWeed instances in FIFO order.
        """
        return list(self._queue.values())

    def get_next_reachable(
        self,
        is_reachable_fn: Callable[[Point], bool]
    ) -> Optional[QueuedWeed]:
        """
        Find the next queued weed reachable by the arm workspace.

        :param is_reachable_fn: Callable returning True if Point is in bounds.
        :return: First reachable QueuedWeed (FIFO), or None if none reachable.
        """
        for weed in self._queue.values():
            if is_reachable_fn(weed.position):
                return weed
        return None

    def pop_next_reachable(
        self,
        is_reachable_fn: Callable[[Point], bool]
    ) -> Optional[QueuedWeed]:
        """
        Pop and return the next queued weed reachable by the arm workspace.

        :param is_reachable_fn: Callable returning True if Point is in bounds.
        :return: The popped QueuedWeed, or None if none reachable.
        """
        target_id: Optional[int] = None
        for weed in self._queue.values():
            if is_reachable_fn(weed.position):
                target_id = weed.weed_id
                break

        if target_id is not None:
            return self._queue.pop(target_id)
        return None

    def has_reachable_weeds(
        self,
        is_reachable_fn: Callable[[Point], bool]
    ) -> bool:
        """
        Check if any currently queued weeds are within reachable bounds.

        :param is_reachable_fn: Callable returning True if Point is in bounds.
        :return: True if at least one weed is reachable, False otherwise.
        """
        return any(is_reachable_fn(w.position) for w in self._queue.values())

    def get_all_queued_ids(self) -> List[int]:
        """Return list of all IDs currently in the queue."""
        return list(self._queue.keys())

    def get_all_removed_ids(self) -> List[int]:
        """Return list of all IDs marked as removed."""
        return list(self._removed_ids)

    def clear_queue(self) -> None:
        """Clear all active weeds in the queue (retains removed history)."""
        self._queue.clear()
