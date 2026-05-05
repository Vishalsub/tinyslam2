#!/usr/bin/env python3
"""
EOS-SLAM Dataset Player Node

EuRoC (MAV) dataset replay to ROS2 topics.

Publishes:
  - /camera/left/image_raw   (sensor_msgs/Image, mono8)
  - /camera/right/image_raw  (sensor_msgs/Image, mono8)
  - /imu/data                 (sensor_msgs/Imu) [if available]

Data pipeline only — no SLAM, estimation, or fusion logic.
"""

from __future__ import annotations

import csv
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image, Imu


class DatasetPlayerNode(Node):
    """
    ROS2 node that replays EuRoC MAV dataset to stereo and IMU topics.
    """

    TOPIC_LEFT_IMAGE = '/camera/left/image_raw'
    TOPIC_RIGHT_IMAGE = '/camera/right/image_raw'
    TOPIC_IMU = '/imu/data'

    DEFAULT_REPLAY_RATE = 30.0

    def __init__(self) -> None:
        super().__init__('dataset_player')

        self._bridge = CvBridge()
        self._frame_index = 0
        self._imu_index = 0

        # (timestamp_ns, path_left, path_right)
        self._image_entries: list[tuple[int, str, str]] = []
        # (timestamp_ns, [omega_x, omega_y, omega_z, acc_x, acc_y, acc_z])
        self._imu_entries: list[tuple[int, list[float]]] = []

        self._replay_rate = self.DEFAULT_REPLAY_RATE
        self._timer = None

        self._declare_parameters()
        self.load_image_list()
        self._load_imu_list()
        if not self._image_entries:
            raise RuntimeError('No stereo images loaded. Cannot start replay.')

        self._create_publishers()
        self._start_replay_timer()

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            'dataset_path',
            value='',
            descriptor=ParameterDescriptor(description='EuRoC dataset root'),
        )
        self.declare_parameter(
            'replay_rate',
            value=self.DEFAULT_REPLAY_RATE,
            descriptor=ParameterDescriptor(description='Replay rate in Hz (default: 30)'),
        )
        self.declare_parameter(
            'frame_id',
            value='camera_optical_frame',
            descriptor=ParameterDescriptor(description='Frame ID for published messages'),
        )

        dataset_path = self.get_parameter('dataset_path').get_parameter_value().string_value
        if not dataset_path:
            raise ValueError(
                'dataset_path parameter is required, e.g. '
                'ros2 run eos_dataset_player dataset_player --ros-args -p dataset_path:=/path/to/dataset'
            )

        self._dataset_path = Path(dataset_path).expanduser().resolve()
        self._replay_rate = self.get_parameter('replay_rate').get_parameter_value().double_value
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.get_logger().info(
            f'Dataset path: {self._dataset_path}, replay rate: {self._replay_rate} Hz'
        )

    def _resolve_euroc_paths(self) -> tuple[Path, Path, Path]:
        """
        Resolve paths for cam0, cam1, and imu0.

        Supports both:
          - EuRoC standard: <root>/mav0/cam0/data, <root>/mav0/cam1/data, <root>/mav0/imu0/data.csv
          - Flat: <root>/cam0/data, <root>/cam1/data, <root>/imu0/data.csv
        """
        mav0 = self._dataset_path / 'mav0'
        if mav0.exists():
            cam0_data = mav0 / 'cam0' / 'data'
            cam1_data = mav0 / 'cam1' / 'data'
            imu_csv = mav0 / 'imu0' / 'data.csv'
        else:
            cam0_data = self._dataset_path / 'cam0' / 'data'
            cam1_data = self._dataset_path / 'cam1' / 'data'
            imu_csv = self._dataset_path / 'imu0' / 'data.csv'
        return cam0_data, cam1_data, imu_csv

    def load_image_list(self) -> None:
        """
        Load sorted list of stereo image pairs with timestamps.

        Left/right images are matched by filename stem (timestamp in ns).
        """
        cam0_data, cam1_data, _ = self._resolve_euroc_paths()

        if not cam0_data.exists():
            raise FileNotFoundError(f'Left camera data directory not found: {cam0_data}')
        if not cam1_data.exists():
            raise FileNotFoundError(f'Right camera data directory not found: {cam1_data}')

        valid_extensions = {'.png', '.jpg', '.jpeg', '.bmp'}
        entries: dict[int, tuple[str, str]] = {}

        for f in cam0_data.iterdir():
            if f.suffix.lower() not in valid_extensions:
                continue
            try:
                ts = int(f.stem)  # EuRoC timestamps are in nanoseconds
            except ValueError:
                continue

            right_path = cam1_data / f.name
            if right_path.exists():
                entries[ts] = (str(f), str(right_path))

        if not entries:
            raise RuntimeError(
                f'No valid stereo image pairs found in {cam0_data} and {cam1_data}'
            )

        self._image_entries = [(ts, left, right) for ts, (left, right) in sorted(entries.items())]
        self.get_logger().info(f'Loaded {len(self._image_entries)} stereo image pairs')

    def _load_imu_list(self) -> None:
        """
        Load EuRoC IMU data from imu0/data.csv.

        EuRoC format: #timestamp [ns], omega_x, omega_y, omega_z, acc_x, acc_y, acc_z
        """
        _, _, imu_csv = self._resolve_euroc_paths()
        if not imu_csv.exists():
            self.get_logger().warn(f'IMU data file not found: {imu_csv}. IMU publishing disabled.')
            return

        self._imu_entries = []
        with open(imu_csv, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0].strip().startswith('#'):
                    continue
                if len(row) < 7:
                    continue
                try:
                    ts = int(row[0])
                    values = [float(row[i]) for i in range(1, 7)]
                    self._imu_entries.append((ts, values))
                except (ValueError, IndexError):
                    continue

        if self._imu_entries:
            self.get_logger().info(f'Loaded {len(self._imu_entries)} IMU measurements')
        else:
            self.get_logger().warn('No IMU data loaded. IMU publishing disabled.')

    def _create_publishers(self) -> None:
        self._pub_left = self.create_publisher(Image, self.TOPIC_LEFT_IMAGE, 10)
        self._pub_right = self.create_publisher(Image, self.TOPIC_RIGHT_IMAGE, 10)
        self._pub_imu = self.create_publisher(Imu, self.TOPIC_IMU, 10)

        self.get_logger().info(
            f'Publishing: {self.TOPIC_LEFT_IMAGE}, {self.TOPIC_RIGHT_IMAGE}, {self.TOPIC_IMU}'
        )

    def _start_replay_timer(self) -> None:
        self._timer = self.create_timer(1.0 / self._replay_rate, self._replay_callback)
        self.get_logger().info(f'Replay started at {self._replay_rate} Hz')

    def publish_images(self, timestamp_ns: int, left_path: str, right_path: str) -> None:
        """Load stereo images from disk and publish with correct timestamps."""
        left_img = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
        right_img = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)

        if left_img is None:
            self.get_logger().warn(f'Failed to load left image: {left_path}')
            return
        if right_img is None:
            self.get_logger().warn(f'Failed to load right image: {right_path}')
            return

        sec = int(timestamp_ns // 1_000_000_000)
        nsec = int(timestamp_ns % 1_000_000_000)

        left_msg = self._bridge.cv2_to_imgmsg(left_img, encoding='mono8')
        right_msg = self._bridge.cv2_to_imgmsg(right_img, encoding='mono8')

        for msg in (left_msg, right_msg):
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nsec
            msg.header.frame_id = self._frame_id

        self._pub_left.publish(left_msg)
        self._pub_right.publish(right_msg)

    def publish_imu(self, image_timestamp_ns: int) -> None:
        """
        Publish all IMU messages with timestamps <= image_timestamp_ns.

        Keeps temporal alignment between images and IMU.
        """
        if not self._imu_entries:
            return

        while self._imu_index < len(self._imu_entries):
            ts_ns, values = self._imu_entries[self._imu_index]
            if ts_ns > image_timestamp_ns:
                break

            msg = Imu()
            msg.header.stamp.sec = int(ts_ns // 1_000_000_000)
            msg.header.stamp.nanosec = int(ts_ns % 1_000_000_000)
            msg.header.frame_id = self._frame_id

            # omega_x, omega_y, omega_z, acc_x, acc_y, acc_z
            msg.angular_velocity.x = float(values[0])
            msg.angular_velocity.y = float(values[1])
            msg.angular_velocity.z = float(values[2])
            msg.linear_acceleration.x = float(values[3])
            msg.linear_acceleration.y = float(values[4])
            msg.linear_acceleration.z = float(values[5])

            # Covariance: -1 means unknown/unspecified
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0

            self._pub_imu.publish(msg)
            self._imu_index += 1

    def _replay_callback(self) -> None:
        """Timer callback: publish next stereo pair and IMU messages up to current time."""
        if self._frame_index >= len(self._image_entries):
            self.get_logger().info(
                'End of dataset reached. Stopping replay. '
                f'Published {len(self._image_entries)} stereo pairs.'
            )
            assert self._timer is not None
            self._timer.cancel()
            return

        ts_ns, left_path, right_path = self._image_entries[self._frame_index]
        self.publish_images(ts_ns, left_path, right_path)
        self.publish_imu(ts_ns)
        self._frame_index += 1


def main(args=None) -> None:
    """Entry point for dataset_player node."""
    rclpy.init(args=args)
    try:
        node = DatasetPlayerNode()
        rclpy.spin(node)
    except (ValueError, RuntimeError, FileNotFoundError) as e:
        import sys

        print(f'Error: {e}', file=sys.stderr)
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
