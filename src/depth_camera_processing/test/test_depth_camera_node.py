"""Integration tests for DepthCameraNode."""

from depth_camera_processing.depth_camera_node import DepthCameraNode
from geometry_msgs.msg import Point
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool


@pytest.fixture
def rclpy_init():
    """Initialize and teardown rclpy for testing."""
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_depth_camera_node_lifecycle(rclpy_init):
    """Test instantiating and shutting down DepthCameraNode."""
    node = DepthCameraNode('test_depth_camera_node')
    assert node.get_name() == 'test_depth_camera_node'
    node.destroy_node()


def test_depth_camera_node_forward_and_localization(rclpy_init):
    """
    Test end-to-end forward and localization flow.

    1. Node receives RGB and Depth frames.
    2. Node receives trigger signal.
    3. Node forwards RGB frame to /camera_endeffector/image_raw.
    4. Node calculates depth at detected weed and publishes precise_location.
    """
    node = DepthCameraNode(
        'test_depth_node_pipeline',
    )
    # Enable mock YOLO mode for testing
    node.yolo._mock_mode = True

    # Track published outputs
    forwarded_images = []
    precise_locations = []

    def on_forward_image(msg):
        forwarded_images.append(msg)

    def on_precise_loc(msg):
        precise_locations.append(msg)

    sub_forward = node.create_subscription(
        Image, node._forward_topic, on_forward_image, 10
    )
    sub_loc = node.create_subscription(
        Point, node._precise_loc_topic, on_precise_loc, 10
    )

    # 1. Create simulated RGB image (640x480)
    rgb_data = np.zeros((480, 640, 3), dtype=np.uint8)
    rgb_msg = Image()
    rgb_msg.height, rgb_msg.width = 480, 640
    rgb_msg.encoding = 'rgb8'
    rgb_msg.step = 640 * 3
    rgb_msg.data = rgb_data.tobytes()
    rgb_msg.header.frame_id = 'camera_endeffector_optical_link'

    # 2. Create simulated 16UC1 Depth image (250mm at center)
    depth_data = np.full((480, 640), 250, dtype=np.uint16)
    depth_msg = Image()
    depth_msg.height, depth_msg.width = 480, 640
    depth_msg.encoding = '16UC1'
    depth_msg.step = 640 * 2
    depth_msg.data = depth_data.tobytes()
    depth_msg.header.frame_id = 'camera_endeffector_optical_link'

    # 3. Create CameraInfo
    ci_msg = CameraInfo()
    ci_msg.k = [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]

    # Inject messages into callbacks
    node._color_image_callback(rgb_msg)
    node._depth_image_callback(depth_msg)
    node._camera_info_callback(ci_msg)

    # 4. Send trigger signal
    trigger_msg = Bool()
    trigger_msg.data = True
    node._trigger_callback(trigger_msg)

    # Spin briefly to allow timers/callbacks to process
    for _ in range(25):
        rclpy.spin_once(node, timeout_sec=0.05)
        if len(precise_locations) > 0 and len(forwarded_images) > 0:
            break

    # Verify that flat RGB image was forwarded
    assert len(forwarded_images) >= 1
    assert forwarded_images[0].width == 640
    assert forwarded_images[0].height == 480

    # Verify that precise location was calculated and published
    assert len(precise_locations) >= 1
    pt = precise_locations[0]
    # Mock YOLO returns (u=320, v=240). Depth is 250mm = 0.250m.
    # At optical center (320, 240): X=0.0, Y=0.0, Z=0.250m
    assert np.isclose(pt.x, 0.0, atol=0.01)
    assert np.isclose(pt.y, 0.0, atol=0.01)
    assert np.isclose(pt.z, 0.250, atol=0.01)

    node.destroy_subscription(sub_forward)
    node.destroy_subscription(sub_loc)
    node.destroy_node()
