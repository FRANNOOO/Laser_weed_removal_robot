#!/usr/bin/env python3
"""
Depth Camera Processing Node for Intel RealSense D405.

Performs:
1. Subscribing to flat RGB and depth streams from RealSense D405 mounted on end-effector.
2. Waiting for trigger signal from state machine (/state_machine_node/trigger_yolo)
   when arm is positioned over approximate weed location.
3. Forwarding flat RGB image to YOLO model (/camera_endeffector/image_raw).
4. Invoking YOLO detection to get weed pixel coordinate (u, v).
5. Querying 2D depth image at (u, v) using O(1) pixel lookup with local median filtering.
6. Projecting (u, v, Z) into 3D optical frame using pinhole camera intrinsics.
7. Transforming 3D coordinates into robot_base_link via TF2.
8. Publishing precise 3D position to /state_machine_node/precise_location.
"""

from typing import List, Optional, Tuple

from depth_camera_processing.depth_image_processor import (
    decode_depth_image,
    extract_intrinsics_from_camera_info,
    get_depth_at_pixel,
    pixel_to_3d_camera_frame,
)
from depth_camera_processing.yolo_client import WeedDetection, YoloClient
from geometry_msgs.msg import Point, PointStamped, PoseStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformException, TransformListener


class DepthCameraNode(Node):
    """ROS 2 Node coordinating depth camera processing and weed 3D localization."""

    def __init__(self, node_name: str = 'depth_camera_node') -> None:
        """Initialize depth camera node, parameters, and subscriptions."""
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter('color_image_topic', '/camera_endeffector/color/image_raw')
        self.declare_parameter(
            'depth_image_topic',
            '/camera_endeffector/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter('camera_info_topic', '/camera_endeffector/color/camera_info')
        self.declare_parameter('yolo_image_forward_topic', '/camera_endeffector/image_raw')
        self.declare_parameter('trigger_topic', '/state_machine_node/trigger_yolo')
        self.declare_parameter('precise_location_topic', '/state_machine_node/precise_location')
        self.declare_parameter(
            'precise_location_pose_topic',
            '/state_machine_node/precise_location_pose',
        )
        self.declare_parameter('camera_optical_frame', 'camera_endeffector_optical_link')
        self.declare_parameter('target_frame', 'robot_base_link')
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('depth_window_size', 3)
        self.declare_parameter('min_valid_depth', 0.02)
        self.declare_parameter('max_valid_depth', 1.20)
        self.declare_parameter('yolo_action_name', 'yolo_detect')
        self.declare_parameter('yolo_detection_topic', '')
        self.declare_parameter('mock_yolo', False)
        self.declare_parameter('classes_filter', ['weed'])
        self.declare_parameter('workspace_min_x', 0.290)
        self.declare_parameter('workspace_max_x', 0.400)
        self.declare_parameter('workspace_min_y', -0.070)
        self.declare_parameter('workspace_max_y', 0.070)

        # Fallback camera intrinsics in case CameraInfo is delayed or missing
        self.declare_parameter('default_fx', 380.0)
        self.declare_parameter('default_fy', 380.0)
        self.declare_parameter('default_cx', 320.0)
        self.declare_parameter('default_cy', 240.0)

        # Read parameters
        self._color_topic = str(self.get_parameter('color_image_topic').value)
        self._depth_topic = str(self.get_parameter('depth_image_topic').value)
        self._info_topic = str(self.get_parameter('camera_info_topic').value)
        self._forward_topic = str(self.get_parameter('yolo_image_forward_topic').value)
        self._trigger_topic = str(self.get_parameter('trigger_topic').value)
        self._precise_loc_topic = str(self.get_parameter('precise_location_topic').value)
        self._precise_pose_topic = str(self.get_parameter('precise_location_pose_topic').value)
        self._camera_frame = str(self.get_parameter('camera_optical_frame').value)
        self._target_frame = str(self.get_parameter('target_frame').value)
        self._depth_scale = float(self.get_parameter('depth_scale').value)
        self._window_size = int(self.get_parameter('depth_window_size').value)
        self._min_depth = float(self.get_parameter('min_valid_depth').value)
        self._max_depth = float(self.get_parameter('max_valid_depth').value)
        self._mock_yolo = bool(self.get_parameter('mock_yolo').value)
        self._ws_min_x = float(self.get_parameter('workspace_min_x').value)
        self._ws_max_x = float(self.get_parameter('workspace_max_x').value)
        self._ws_min_y = float(self.get_parameter('workspace_min_y').value)
        self._ws_max_y = float(self.get_parameter('workspace_max_y').value)

        # Image and intrinsics caching
        self._latest_color_msg: Optional[Image] = None
        self._latest_depth_msg: Optional[Image] = None
        self._intrinsics: Optional[Tuple[float, float, float, float]] = None
        self._is_processing: bool = False

        # Fallback intrinsics
        self._default_intrinsics = (
            float(self.get_parameter('default_fx').value),
            float(self.get_parameter('default_fy').value),
            float(self.get_parameter('default_cx').value),
            float(self.get_parameter('default_cy').value),
        )

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Multi-threaded callback group to prevent deadlocks between
        # image streams, trigger signals, and action client responses.
        self._cb_group = ReentrantCallbackGroup()

        # YOLO client adapter
        filter_param = self.get_parameter('classes_filter').value
        classes_filter = list(filter_param) if filter_param else None
        yolo_det_topic = str(self.get_parameter('yolo_detection_topic').value)
        self.yolo = YoloClient(
            self,
            action_name=str(self.get_parameter('yolo_action_name').value),
            detection_topic=yolo_det_topic if yolo_det_topic else None,
            mock_mode=self._mock_yolo,
            classes_filter=classes_filter,
            callback_group=self._cb_group,
        )

        # Publishers
        self._forward_image_pub = self.create_publisher(Image, self._forward_topic, 10)
        self._precise_loc_pub = self.create_publisher(Point, self._precise_loc_topic, 10)
        self._precise_pose_pub = self.create_publisher(PoseStamped, self._precise_pose_topic, 10)
        self._active_pub = self.create_publisher(Bool, '~/localization_active', 10)
        self._detected_uv_pub = self.create_publisher(Point, '~/last_detected_pixel', 10)

        # QoS for sensor data - keep only latest frame (depth=1) to prevent queuing
        # backlog and minimize memory and Zenoh network transfer pressure.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Subscriptions
        self._color_sub = self.create_subscription(
            Image,
            self._color_topic,
            self._color_image_callback,
            sensor_qos,
            callback_group=self._cb_group,
        )
        self._depth_sub = self.create_subscription(
            Image,
            self._depth_topic,
            self._depth_image_callback,
            sensor_qos,
            callback_group=self._cb_group,
        )
        self._info_sub = self.create_subscription(
            CameraInfo,
            self._info_topic,
            self._camera_info_callback,
            10,
            callback_group=self._cb_group,
        )
        self._trigger_sub = self.create_subscription(
            Bool,
            self._trigger_topic,
            self._trigger_callback,
            10,
            callback_group=self._cb_group,
        )

        # Manual / diagnostic service trigger
        self._trigger_service = self.create_service(
            Trigger,
            '~/trigger_localization',
            self._service_trigger_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f'DepthCameraNode initialized. Subscribing: color="{self._color_topic}", '
            f'depth="{self._depth_topic}", trigger="{self._trigger_topic}". '
            f'Forwarding to "{self._forward_topic}", publishing to "{self._precise_loc_topic}".'
        )

    def _color_image_callback(self, msg: Image) -> None:
        """Cache latest flat RGB image."""
        self._latest_color_msg = msg

    def _depth_image_callback(self, msg: Image) -> None:
        """Cache latest depth image."""
        self._latest_depth_msg = msg

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Extract and cache camera intrinsics."""
        try:
            self._intrinsics = extract_intrinsics_from_camera_info(msg)
        except ValueError as e:
            self.get_logger().debug(f'Waiting for initialized CameraInfo: {e}')

    def _trigger_callback(self, msg: Bool) -> None:
        """Handle trigger signal from state machine."""
        if not msg.data:
            return

        if self._is_processing:
            self.get_logger().debug(
                'Received trigger_yolo=True while localization already in progress; skipping.'
            )
            return

        self.get_logger().info(
            f'Received trigger_yolo=True from {self._trigger_topic}. '
            'Activating depth camera weed localization...'
        )
        self.execute_weed_localization()

    def _service_trigger_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle manual trigger via service call."""
        success = self.execute_weed_localization()
        response.success = success
        response.message = (
            'Weed localization initiated.'
            if success
            else 'Failed to initiate localization (busy or images missing).'
        )
        return response

    def execute_weed_localization(self) -> bool:
        """
        Execute the full weed localization pipeline.

        1. Check image availability (instant cached dispatch).
        2. Dispatch detection request to YOLO with goal-acceptance image forwarding.
        3. In callback, lookup depth and select best candidate within workspace.
        4. Transform to base frame and publish precise location.

        :return: True if process was initiated, False otherwise.
        """
        if self._is_processing:
            self.get_logger().debug(
                'Localization already in progress; skipping duplicate trigger.'
            )
            return False

        # If cached frames exist, proceed immediately without waiting.
        # Only if either frame is missing, wait briefly (up to 0.5s).
        if self._latest_color_msg is None or self._latest_depth_msg is None:
            import time
            wait_deadline = time.time() + 0.5
            while (
                (self._latest_color_msg is None or self._latest_depth_msg is None)
                and time.time() < wait_deadline
            ):
                time.sleep(0.01)

        if self._latest_color_msg is None:
            self.get_logger().error(
                f'Cannot localize weed: No RGB image received on "{self._color_topic}".'
            )
            return False

        if self._latest_depth_msg is None:
            self.get_logger().error(
                f'Cannot localize weed: No depth image received on "{self._depth_topic}".'
            )
            return False

        self._is_processing = True
        self._publish_active(True)

        color_msg = self._latest_color_msg

        def forward_rgb_image() -> None:
            """Publish image to YOLO input topic."""
            self._forward_image_pub.publish(color_msg)
            self.get_logger().info(
                f'Forwarded RGB image ({color_msg.width}x{color_msg.height}, '
                f'encoding={color_msg.encoding}) to "{self._forward_topic}".'
            )

        # Publish once immediately
        forward_rgb_image()

        # Trigger YOLO inference, and publish again immediately when goal is accepted by server
        dispatched = self.yolo.request_detection(
            callback=self._on_yolo_detections_received,
            timeout_sec=5.0,
            on_goal_accepted=forward_rgb_image,
        )

        if not dispatched:
            self.get_logger().error('Failed to dispatch YOLO detection request.')
            self._is_processing = False
            self._publish_active(False)
            return False

        return True

    def _on_yolo_detections_received(self, detections: List[WeedDetection]) -> None:
        """
        Process returned detections, inspect depth image, and calculate 3D coordinates.

        :param detections: List of 2D detections from YOLO.
        """
        try:
            if not detections:
                self.get_logger().warn('YOLO returned 0 detections. Target weed not detected.')
                return

            self.get_logger().info(
                f'YOLO returned {len(detections)} detection(s). Resolving precise coordinates...'
            )

            # Resolve camera intrinsics
            fx, fy, cx, cy = (
                self._intrinsics if self._intrinsics is not None else self._default_intrinsics
            )

            depth_msg = self._latest_depth_msg
            if depth_msg is None:
                self.get_logger().error('Depth image was lost during processing.')
                return

            depth_array = decode_depth_image(depth_msg, depth_scale=self._depth_scale)
            optical_frame = depth_msg.header.frame_id or self._camera_frame

            # Evaluate candidate detections
            candidates = []
            for det in detections:
                depth_val = get_depth_at_pixel(
                    depth_array,
                    u=det.u,
                    v=det.v,
                    window_size=self._window_size,
                    min_depth=self._min_depth,
                    max_depth=self._max_depth,
                )
                if depth_val is None:
                    continue

                xc, yc, zc = pixel_to_3d_camera_frame(
                    det.u, det.v, depth_val, fx, fy, cx, cy
                )
                pt, pose = self._transform_to_target_frame(
                    xc, yc, zc, source_frame=optical_frame, stamp=depth_msg.header.stamp
                )
                in_ws = (
                    self._ws_min_x <= pt.x <= self._ws_max_x
                    and self._ws_min_y <= pt.y <= self._ws_max_y
                )
                center_dist_sq = (det.u - cx) ** 2 + (det.v - cy) ** 2
                candidates.append((in_ws, center_dist_sq, det, depth_val, xc, yc, zc, pt, pose))

            if not candidates:
                self.get_logger().error(
                    f'Could not retrieve valid depth for any of {len(detections)} candidate(s).'
                )
                return

            # Prioritize candidates within the workspace, then closest to center
            candidates.sort(key=lambda c: (not c[0], c[1]))
            _, _, best_det, depth_m, x_cam, y_cam, z_cam, precise_pt, precise_pose = candidates[0]

            self.get_logger().info(
                f'Selected weed candidate at pixel ({best_det.u:.1f}, {best_det.v:.1f}) '
                f'[size: {best_det.width:.0f}x{best_det.height:.0f}, '
                f'conf={best_det.confidence:.2f}, depth={depth_m:.4f}m].'
            )

            # Publish pixel + depth debug
            uv_msg = Point(x=float(best_det.u), y=float(best_det.v), z=float(depth_m))
            self._detected_uv_pub.publish(uv_msg)

            self.get_logger().info(
                f'Projected 3D camera coordinates: ({x_cam:.4f}, {y_cam:.4f}, {z_cam:.4f}) m.'
            )

            # Publish precise location to State Machine
            self._precise_loc_pub.publish(precise_pt)
            self._precise_pose_pub.publish(precise_pose)

            self.get_logger().info(
                f'Successfully published precise weed location in {self._target_frame}: '
                f'x={precise_pt.x:.4f}, y={precise_pt.y:.4f}, z={precise_pt.z:.4f}'
            )

        except Exception as e:
            self.get_logger().error(f'Error during weed depth localization: {e}')
        finally:
            self._is_processing = False
            self._publish_active(False)

    def _transform_to_target_frame(
        self,
        x: float,
        y: float,
        z: float,
        source_frame: str,
        stamp,
    ) -> Tuple[Point, PoseStamped]:
        """
        Transform 3D point from source_frame to target_frame using TF2.

        Falls back to camera frame coordinates if transform is unavailable.

        :param x: X in camera frame.
        :param y: Y in camera frame.
        :param z: Z in camera frame.
        :param source_frame: Source TF frame ID.
        :param stamp: Image timestamp.
        :return: Tuple of (Point, PoseStamped) in target_frame.
        """
        pt_stamped = PointStamped()
        pt_stamped.header.frame_id = source_frame
        pt_stamped.header.stamp = stamp
        pt_stamped.point.x = float(x)
        pt_stamped.point.y = float(y)
        pt_stamped.point.z = float(z)

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = source_frame
        pose_stamped.header.stamp = stamp
        pose_stamped.pose.position.x = float(x)
        pose_stamped.pose.position.y = float(y)
        pose_stamped.pose.position.z = float(z)
        pose_stamped.pose.orientation.w = 1.0

        if not source_frame or source_frame == self._target_frame:
            return pt_stamped.point, pose_stamped

        try:
            transformed = self.tf_buffer.transform(
                pt_stamped,
                self._target_frame,
                timeout=Duration(seconds=0.1),
            )
            pose_stamped.header.frame_id = self._target_frame
            pose_stamped.pose.position = transformed.point
            return transformed.point, pose_stamped
        except TransformException as ex:
            # Fallback to latest available transform (Time(0)) to absorb network/clock jitter
            try:
                pt_stamped.header.stamp = rclpy.time.Time().to_msg()
                transformed = self.tf_buffer.transform(
                    pt_stamped,
                    self._target_frame,
                    timeout=Duration(seconds=0.1),
                )
                pose_stamped.header.frame_id = self._target_frame
                pose_stamped.header.stamp = pt_stamped.header.stamp
                pose_stamped.pose.position = transformed.point
                return transformed.point, pose_stamped
            except TransformException as ex2:
                self.get_logger().warning(
                    f'TF transform from "{source_frame}" to "{self._target_frame}" failed '
                    f'({ex}; fallback {ex2}). Falling back to raw camera frame coordinates.'
                )
                return pt_stamped.point, pose_stamped

    def _publish_active(self, active: bool) -> None:
        """Publish boolean indicating whether localization is currently in progress."""
        msg = Bool()
        msg.data = active
        self._active_pub.publish(msg)


def main(args: Optional[list] = None) -> None:
    """Run the depth camera processing node."""
    rclpy.init(args=args)
    node = DepthCameraNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        if 'rcl_shutdown already called' not in str(e):
            raise
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
