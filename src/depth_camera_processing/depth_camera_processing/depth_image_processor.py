"""
Depth Image Processing Utilities for RealSense D405.

Provides high-performance, point-cloud-free 2D depth image decoding,
sub-pixel/median depth lookup, and pinhole camera 3D projection.
"""

from typing import Optional, Tuple

import numpy as np
from sensor_msgs.msg import CameraInfo, Image


def decode_depth_image(
    msg: Image,
    depth_scale: float = 0.001,
) -> np.ndarray:
    """
    Decode a sensor_msgs/Image depth image into a 2D float array in meters.

    Supports '16UC1', 'mono16', and '32FC1' encodings without external
    vision dependencies (e.g. OpenCV / cv_bridge).

    :param msg: ROS 2 sensor_msgs/Image message.
    :param depth_scale: Multiplier to convert raw integer depth to meters
                        (default 0.001 for 16UC1 mm -> m).
    :return: 2D numpy array of shape (height, width) with depth values in meters.
    :raises ValueError: If encoding is unsupported or byte buffer size mismatch.
    """
    height = msg.height
    width = msg.width
    encoding = msg.encoding.lower()

    if encoding in ('16uc1', 'mono16'):
        dtype = '>u2' if msg.is_bigendian else '<u2'
        raw = np.frombuffer(msg.data, dtype=dtype)
        if raw.size != height * width:
            raise ValueError(
                f'Buffer size {raw.size} does not match dimensions {width}x{height}'
            )
        depth_m = raw.reshape((height, width)).astype(np.float32) * float(depth_scale)
        return depth_m

    elif encoding in ('32fc1',):
        dtype = '>f4' if msg.is_bigendian else '<f4'
        raw = np.frombuffer(msg.data, dtype=dtype)
        if raw.size != height * width:
            raise ValueError(
                f'Buffer size {raw.size} does not match dimensions {width}x{height}'
            )
        depth_m = raw.reshape((height, width)).astype(np.float32)
        return depth_m

    else:
        raise ValueError(
            f'Unsupported depth encoding "{msg.encoding}". Expected 16UC1 or 32FC1.'
        )


def get_depth_at_pixel(
    depth_array: np.ndarray,
    u: float,
    v: float,
    window_size: int = 3,
    min_depth: float = 0.02,
    max_depth: float = 1.0,
) -> Optional[float]:
    """
    Query depth at pixel coordinate (u, v) with local median filter fallback.

    If the center pixel is invalid (0, NaN, or out of depth bounds),
    samples an NxN neighborhood window to calculate the median of valid depths,
    avoiding dropout errors common near stems or shadowed soil.

    :param depth_array: 2D numpy float array of depth in meters.
    :param u: Horizontal pixel coordinate (column).
    :param v: Vertical pixel coordinate (row).
    :param window_size: Neighborhood window width (odd integer >= 1).
    :param min_depth: Minimum acceptable depth in meters.
    :param max_depth: Maximum acceptable depth in meters.
    :return: Depth in meters, or None if no valid depth found.
    """
    if depth_array is None or depth_array.ndim != 2:
        return None

    height, width = depth_array.shape
    col = int(round(u))
    row = int(round(v))

    if not (0 <= col < width and 0 <= row < height):
        return None

    center_val = float(depth_array[row, col])
    if not np.isnan(center_val) and min_depth <= center_val <= max_depth:
        return center_val

    # Center pixel is invalid or out of range; use local neighborhood window
    if window_size <= 1:
        return None

    half = window_size // 2
    r_min = max(0, row - half)
    r_max = min(height, row + half + 1)
    c_min = max(0, col - half)
    c_max = min(width, col + half + 1)

    window = depth_array[r_min:r_max, c_min:c_max].flatten()
    valid_mask = (~np.isnan(window)) & (window >= min_depth) & (window <= max_depth)
    valid_vals = window[valid_mask]

    if valid_vals.size == 0:
        return None

    return float(np.median(valid_vals))


def extract_intrinsics_from_camera_info(
    camera_info: CameraInfo,
) -> Tuple[float, float, float, float]:
    """
    Extract (fx, fy, cx, cy) from CameraInfo message.

    :param camera_info: ROS 2 sensor_msgs/CameraInfo message.
    :return: Tuple of (fx, fy, cx, cy).
    :raises ValueError: If camera info does not contain valid intrinsic parameters.
    """
    # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    if len(camera_info.k) >= 9 and camera_info.k[0] > 0.0 and camera_info.k[4] > 0.0:
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        return fx, fy, cx, cy

    # Projection matrix P = [fx', 0, cx', Tx, 0, fy', cy', Ty, 0, 0, 1, 0]
    if len(camera_info.p) >= 12 and camera_info.p[0] > 0.0 and camera_info.p[5] > 0.0:
        fx = float(camera_info.p[0])
        fy = float(camera_info.p[5])
        cx = float(camera_info.p[2])
        cy = float(camera_info.p[6])
        return fx, fy, cx, cy

    raise ValueError(
        'CameraInfo contains zero or uninitialized focal lengths in K and P matrices.'
    )


def pixel_to_3d_camera_frame(
    u: float,
    v: float,
    depth_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> Tuple[float, float, float]:
    """
    Project 2D pixel (u, v) and depth Z to 3D point in camera optical frame.

    Standard pinhole camera model:
        X_cam = (u - cx) * depth_m / fx
        Y_cam = (v - cy) * depth_m / fy
        Z_cam = depth_m

    :param u: Horizontal pixel coordinate.
    :param v: Vertical pixel coordinate.
    :param depth_m: Measured depth along camera optical Z axis (meters).
    :param fx: Focal length in X (pixels).
    :param fy: Focal length in Y (pixels).
    :param cx: Principal point X (pixels).
    :param cy: Principal point Y (pixels).
    :return: (X_cam, Y_cam, Z_cam) in camera optical coordinate frame.
    """
    x_cam = (float(u) - float(cx)) * float(depth_m) / float(fx)
    y_cam = (float(v) - float(cy)) * float(depth_m) / float(fy)
    z_cam = float(depth_m)
    return x_cam, y_cam, z_cam
