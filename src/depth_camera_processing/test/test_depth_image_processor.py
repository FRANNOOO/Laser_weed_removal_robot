"""Unit tests for depth_image_processor."""

from depth_camera_processing.depth_image_processor import (
    decode_depth_image,
    extract_intrinsics_from_camera_info,
    get_depth_at_pixel,
    pixel_to_3d_camera_frame,
)
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image


def create_mock_depth_image(
    array: np.ndarray,
    encoding: str = '16UC1',
    is_bigendian: bool = False,
) -> Image:
    """Create a mock sensor_msgs/Image from numpy array."""
    msg = Image()
    msg.height, msg.width = array.shape
    msg.encoding = encoding
    msg.is_bigendian = is_bigendian
    msg.step = msg.width * array.itemsize
    msg.data = array.tobytes()
    return msg


def test_decode_depth_image_16uc1():
    """Test decoding 16UC1 depth image (mm to meters)."""
    raw = np.array([
        [100, 200, 500],
        [1000, 0, 350],
    ], dtype=np.uint16)

    img_msg = create_mock_depth_image(raw, encoding='16UC1')
    decoded = decode_depth_image(img_msg, depth_scale=0.001)

    assert decoded.shape == (2, 3)
    assert decoded.dtype == np.float32
    assert np.isclose(decoded[0, 0], 0.100)
    assert np.isclose(decoded[0, 1], 0.200)
    assert np.isclose(decoded[0, 2], 0.500)
    assert np.isclose(decoded[1, 0], 1.000)
    assert np.isclose(decoded[1, 1], 0.000)
    assert np.isclose(decoded[1, 2], 0.350)


def test_decode_depth_image_32fc1():
    """Test decoding 32FC1 depth image (already in meters)."""
    raw = np.array([
        [0.15, 0.25],
        [0.35, 0.45],
    ], dtype=np.float32)

    img_msg = create_mock_depth_image(raw, encoding='32FC1')
    decoded = decode_depth_image(img_msg)

    assert decoded.shape == (2, 2)
    assert np.isclose(decoded[0, 0], 0.15)
    assert np.isclose(decoded[1, 1], 0.45)


def test_decode_depth_image_invalid_encoding():
    """Test error handling for unsupported encodings."""
    raw = np.zeros((2, 2), dtype=np.uint8)
    img_msg = create_mock_depth_image(raw, encoding='rgb8')

    with pytest.raises(ValueError, match='Unsupported depth encoding'):
        decode_depth_image(img_msg)


def test_get_depth_at_pixel_valid():
    """Test direct lookup at pixel with valid depth."""
    depth_array = np.full((10, 10), 0.25, dtype=np.float32)
    depth = get_depth_at_pixel(depth_array, u=5.0, v=4.0)
    assert depth is not None
    assert np.isclose(depth, 0.25)


def test_get_depth_at_pixel_median_filtering_on_dropout():
    """Test median window filtering when center pixel is 0 (dropout hole)."""
    depth_array = np.full((5, 5), 0.20, dtype=np.float32)
    depth_array[2, 2] = 0.0  # Dropout hole
    depth_array[2, 3] = 0.22
    depth_array[2, 1] = 0.24

    depth = get_depth_at_pixel(
        depth_array, u=2.0, v=2.0, window_size=3, min_depth=0.05, max_depth=1.0
    )
    assert depth is not None
    assert 0.19 <= depth <= 0.25


def test_get_depth_at_pixel_all_invalid():
    """Test returning None when entire neighborhood is invalid/zero."""
    depth_array = np.zeros((5, 5), dtype=np.float32)
    depth = get_depth_at_pixel(
        depth_array, u=2.0, v=2.0, window_size=3, min_depth=0.05
    )
    assert depth is None


def test_get_depth_at_pixel_out_of_bounds():
    """Test returning None for out-of-bounds pixel queries."""
    depth_array = np.ones((5, 5), dtype=np.float32)
    assert get_depth_at_pixel(depth_array, u=-1.0, v=2.0) is None
    assert get_depth_at_pixel(depth_array, u=5.0, v=2.0) is None
    assert get_depth_at_pixel(depth_array, u=2.0, v=10.0) is None


def test_pixel_to_3d_camera_frame():
    """Test 3D pinhole projection from pixel and depth."""
    fx, fy = 400.0, 400.0
    cx, cy = 320.0, 240.0
    depth_m = 0.200

    x, y, z = pixel_to_3d_camera_frame(320.0, 240.0, depth_m, fx, fy, cx, cy)
    assert np.isclose(x, 0.0)
    assert np.isclose(y, 0.0)
    assert np.isclose(z, 0.200)

    x, y, z = pixel_to_3d_camera_frame(400.0, 200.0, depth_m, fx, fy, cx, cy)
    assert np.isclose(x, 0.04)
    assert np.isclose(y, -0.02)
    assert np.isclose(z, 0.200)


def test_extract_intrinsics_from_camera_info():
    """Test extracting intrinsics from CameraInfo K matrix."""
    ci = CameraInfo()
    ci.k = [385.0, 0.0, 321.5, 0.0, 386.0, 242.0, 0.0, 0.0, 1.0]

    fx, fy, cx, cy = extract_intrinsics_from_camera_info(ci)
    assert np.isclose(fx, 385.0)
    assert np.isclose(fy, 386.0)
    assert np.isclose(cx, 321.5)
    assert np.isclose(cy, 242.0)


def test_extract_intrinsics_uninitialized():
    """Test exception when CameraInfo has zero focal length."""
    ci = CameraInfo()
    ci.k = [0.0] * 9

    with pytest.raises(ValueError, match='zero or uninitialized'):
        extract_intrinsics_from_camera_info(ci)
