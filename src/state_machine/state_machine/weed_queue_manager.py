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
import math
import time
from typing import Callable, Dict, List, Optional, Set, Tuple

from geometry_msgs.msg import Point


@dataclass
class QueuedWeed:
    """Represents a weed awaiting removal in the queue."""

    weed_id: int
    position: Point
    timestamp: float
    initial_position: Optional[Point] = None
    raw_position: Optional[Point] = None
    raw_frame_id: str = ''
    odom_pose_at_detection: Optional[Tuple[float, float, float]] = None


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
        timestamp: Optional[float] = None,
        raw_position: Optional[Point] = None,
        raw_frame_id: str = '',
        odom_pose: Optional[Tuple[float, float, float]] = None,
    ) -> bool:
        """
        Add newly detected weed or update position of existing weed.

        If the weed has already been marked as removed, it is ignored.

        :param weed_id: Unique integer identifier of the weed.
        :param position: 3D coordinates (Point) in the robot base frame.
        :param timestamp: Detection timestamp (defaults to system time).
        :param raw_position: Original Point before any transformation.
        :param raw_frame_id: Source frame ID of the detection message.
        :param odom_pose: Vehicle odometry pose (x, y, yaw) at detection time.
        :return: True if new weed added to queue, False if ignored or updated.
        """
        if self.is_removed(weed_id):
            return False

        ts = timestamp if timestamp is not None else time.time()
        orig_pt = raw_position if raw_position is not None else Point(
            x=position.x, y=position.y, z=position.z
        )

        if weed_id in self._queue:
            # Update existing weed position and timestamp
            self._queue[weed_id].position = position
            self._queue[weed_id].timestamp = ts
            if self._queue[weed_id].initial_position is None:
                self._queue[weed_id].initial_position = Point(
                    x=position.x, y=position.y, z=position.z
                )
            if raw_position is not None:
                self._queue[weed_id].raw_position = raw_position
            if raw_frame_id:
                self._queue[weed_id].raw_frame_id = raw_frame_id
            if odom_pose is not None:
                self._queue[weed_id].odom_pose_at_detection = odom_pose
            return False

        self._queue[weed_id] = QueuedWeed(
            weed_id=weed_id,
            position=position,
            timestamp=ts,
            initial_position=Point(x=position.x, y=position.y, z=position.z),
            raw_position=orig_pt,
            raw_frame_id=raw_frame_id,
            odom_pose_at_detection=odom_pose,
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

    def get_oldest_reachable_candidate(
        self,
        max_x_threshold: float
    ) -> Optional[QueuedWeed]:
        """
        Return the oldest queued weed that has not passed beyond max_x_threshold.

        :param max_x_threshold: Maximum allowable X coordinate before weed is past workspace.
        :return: Oldest candidate QueuedWeed, or None if none found.
        """
        for weed in self._queue.values():
            if weed.position.x <= max_x_threshold:
                return weed
        return None

    def update_positions(
        self,
        tf_transform_fn: Optional[Callable[[Point, str], Optional[Point]]] = None,
        current_odom_pose: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        """
        Update estimated base_link positions for all active queued weeds.

        Tries TF transform first if a transform function and raw_frame_id are
        available. Falls back to odometry dead-reckoning displacement when TF
        fails or is unavailable.

        :param tf_transform_fn: Optional callable transforming (Point, source_frame) -> Point.
        :param current_odom_pose: Current vehicle odometry pose (x, y, yaw).
        """
        for weed in self._queue.values():
            updated = False
            raw_pt = weed.raw_position or weed.position

            # 1. Try TF transform if frame is known
            if tf_transform_fn is not None and weed.raw_frame_id:
                try:
                    tf_pos = tf_transform_fn(raw_pt, weed.raw_frame_id)
                    if tf_pos is not None:
                        weed.position = tf_pos
                        weed.initial_position = Point(
                            x=tf_pos.x, y=tf_pos.y, z=tf_pos.z
                        )
                        if current_odom_pose is not None:
                            weed.odom_pose_at_detection = current_odom_pose
                        updated = True
                except Exception:
                    updated = False

            # 2. Odometry dead-reckoning displacement fallback
            if (
                not updated
                and current_odom_pose is not None
                and weed.odom_pose_at_detection is not None
            ):
                x_now, y_now, _ = current_odom_pose
                x_det, y_det, yaw_det = weed.odom_pose_at_detection

                dx_world = x_now - x_det
                dy_world = y_now - y_det

                # Forward displacement along vehicle heading at detection
                dx_robot = dx_world * math.cos(yaw_det) + dy_world * math.sin(yaw_det)
                dy_robot = -dx_world * math.sin(yaw_det) + dy_world * math.cos(yaw_det)

                # In robot_base_link (rotated 180 deg about Z relative to base_link):
                # Vehicle forward motion (+dx_robot) causes stationary ground points
                # to advance in +X (towards the manipulator arm).
                init_pt = weed.initial_position or weed.position
                weed.position = Point(
                    x=init_pt.x + dx_robot,
                    y=init_pt.y + dy_robot,
                    z=init_pt.z,
                )

    def purge_unreachable(self, max_x_threshold: float) -> int:
        """
        Remove weeds that have moved completely past the reachable workspace.

        :param max_x_threshold: X coordinate above which weeds are permanently unreachable.
        :return: Count of purged weeds.
        """
        to_remove = [
            wid for wid, w in self._queue.items()
            if w.position.x > max_x_threshold
        ]
        for wid in to_remove:
            self._queue.pop(wid, None)
        return len(to_remove)

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
