"""
YOLO Client and Adapter for End-Effector Detection Requests.

Handles triggering YOLO inference via the 'yolo_detect' ROS 2 Action Server
(yolo_jazzy_od_interfaces/action/Detection), topic subscription fallback,
or a simulated test detector.
"""

from typing import Callable, List, NamedTuple, Optional

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

try:
    from yolo_jazzy_od_interfaces.action import Detection
    YOLO_ACTION_AVAILABLE = True
except ImportError:
    YOLO_ACTION_AVAILABLE = False
    Detection = None

try:
    from vision_msgs.msg import Detection2DArray
    VISION_MSGS_AVAILABLE = True
except ImportError:
    VISION_MSGS_AVAILABLE = False
    Detection2DArray = None


class WeedDetection(NamedTuple):
    """Represents a 2D weed detection in image space."""

    u: float
    v: float
    width: float
    height: float
    confidence: float
    class_id: str


class YoloClient:
    """Coordinates detection requests to the YOLO model."""

    def __init__(
        self,
        node: Node,
        action_name: str = 'yolo_detect',
        detection_topic: Optional[str] = None,
        mock_mode: bool = False,
        classes_filter: Optional[List[str]] = None,
        callback_group: Optional[object] = None,
    ) -> None:
        """
        Initialize the YOLO client adapter.

        :param node: Parent ROS 2 node.
        :param action_name: Name of the Detection action server.
        :param detection_topic: Optional topic to listen for detections.
        :param mock_mode: If true, generates simulated detections for testing.
        :param classes_filter: List of class IDs/labels to accept (None accepts all).
        :param callback_group: Optional ROS 2 callback group for action client.
        """
        self._node = node
        self._logger = node.get_logger()
        self._action_name = action_name
        self._mock_mode = mock_mode

        if classes_filter:
            expanded = set()
            for c in classes_filter:
                s = str(c).strip().lower()
                expanded.add(s)
                # Transparently alias common weed and meristem labels and IDs
                if s in ('weed', 'meristem', '0', '4'):
                    expanded.update({'0', '4', 'weed', 'meristem'})
            self._classes_filter = expanded
        else:
            self._classes_filter = None

        self._action_client: Optional[ActionClient] = None
        self._active_callback: Optional[Callable[[List[WeedDetection]], None]] = None
        self._topic_sub = None

        if not self._mock_mode and YOLO_ACTION_AVAILABLE and Detection is not None:
            self._action_client = ActionClient(
                node, Detection, action_name, callback_group=callback_group
            )
            self._logger.info(
                f'YoloClient initialized with action server: "{action_name}"'
            )
        elif not self._mock_mode:
            self._logger.warn(
                'yolo_jazzy_od_interfaces not available; falling back to topic or mock mode.'
            )

        if detection_topic and VISION_MSGS_AVAILABLE:
            self._topic_sub = node.create_subscription(
                Detection2DArray,
                detection_topic,
                self._detection_topic_callback,
                10,
            )
            self._logger.info(
                f'YoloClient subscribed to detection topic: "{detection_topic}"'
            )

    @property
    def is_action_available(self) -> bool:
        """Check if action server interface is configured and ready."""
        if self._action_client is None:
            return False
        return self._action_client.server_is_ready()

    def request_detection(
        self,
        callback: Callable[[List[WeedDetection]], None],
        timeout_sec: float = 3.0,
        mock_weed_uv: Optional[tuple] = None,
    ) -> bool:
        """
        Trigger YOLO weed detection.

        :param callback: Function called with List[WeedDetection] once inference completes.
        :param timeout_sec: Maximum time to wait for action server if connecting.
        :param mock_weed_uv: Optional (u, v) tuple to return in mock mode.
        :return: True if detection request was dispatched, False otherwise.
        """
        self._active_callback = callback

        if self._mock_mode or self._action_client is None:
            self._logger.info('YoloClient running in mock/offline detection mode.')
            u = 320.0 if mock_weed_uv is None else float(mock_weed_uv[0])
            v = 240.0 if mock_weed_uv is None else float(mock_weed_uv[1])
            detections = [
                WeedDetection(
                    u=u,
                    v=v,
                    width=40.0,
                    height=40.0,
                    confidence=0.95,
                    class_id='weed',
                )
            ]
            self._node.create_timer(
                0.05,
                lambda: self._dispatch_results(detections),
            )
            return True

        if not self._action_client.wait_for_server(timeout_sec=timeout_sec):
            self._logger.error(
                f'YOLO action server "{self._action_name}" unavailable after {timeout_sec}s.'
            )
            return False

        goal_msg = Detection.Goal()
        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)
        return True

    def _on_goal_response(self, future) -> None:
        """Handle goal response from action server."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._logger.warning('YOLO detection goal was rejected by server.')
                self._dispatch_results([])
                return

            res_future = goal_handle.get_result_async()
            res_future.add_done_callback(self._on_action_result)
        except Exception as e:
            self._logger.error(f'Error receiving goal response: {e}')
            self._dispatch_results([])

    def _on_action_result(self, future) -> None:
        """Process result from YOLO detection action."""
        try:
            goal_result = future.result()
            if goal_result is None or goal_result.status != GoalStatus.STATUS_SUCCEEDED:
                status = goal_result.status if goal_result else 'None'
                self._logger.error(f'YOLO action finished with status {status}')
                self._dispatch_results([])
                return

            result = goal_result.result
            detections = self._parse_detection_array(result.detections)
            self._dispatch_results(detections)
        except Exception as e:
            self._logger.error(f'Exception parsing YOLO action result: {e}')
            self._dispatch_results([])

    def _detection_topic_callback(self, msg) -> None:
        """Process asynchronous detections published on topic."""
        if self._active_callback is not None:
            detections = self._parse_detection_array(msg)
            self._dispatch_results(detections)

    def _parse_detection_array(self, detection_array) -> List[WeedDetection]:
        """Convert a vision_msgs/Detection2DArray into a list of WeedDetection."""
        parsed: List[WeedDetection] = []
        if detection_array is None or not hasattr(detection_array, 'detections'):
            return parsed

        for det in detection_array.detections:
            u = float(det.bbox.center.position.x)
            v = float(det.bbox.center.position.y)
            w = float(det.bbox.size_x)
            h = float(det.bbox.size_y)

            conf = 0.0
            class_id = ''
            if hasattr(det, 'results') and len(det.results) > 0:
                conf = float(det.results[0].hypothesis.score)
                class_id = str(det.results[0].hypothesis.class_id)

            if (
                self._classes_filter is not None
                and class_id.strip().lower() not in self._classes_filter
            ):
                continue

            parsed.append(
                WeedDetection(
                    u=u,
                    v=v,
                    width=w,
                    height=h,
                    confidence=conf,
                    class_id=class_id,
                )
            )

        return parsed

    def _dispatch_results(self, detections: List[WeedDetection]) -> None:
        """Send detections to the registered callback."""
        cb = self._active_callback
        self._active_callback = None
        if cb is not None:
            cb(detections)
