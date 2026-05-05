#!/usr/bin/env python3
"""
EOS-SLAM Bag Player Node

Reads ROS1 or ROS2 bags (using rosbags library) and republishes to EOS-SLAM topic
conventions. Bypasses ros2 bag play for compatibility with bags that have
metadata parsing issues (e.g. yaml-cpp bad conversion with large timestamps).

Topic mapping:
  /cam0/image_raw  -> /camera/left/image_raw
  /cam1/image_raw  -> /camera/right/image_raw
  /imu0            -> /imu/data
"""

from __future__ import annotations

import time
from pathlib import Path

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu


# Topic mapping: bag topic -> EOS-SLAM topic
TOPIC_MAP = {
    '/cam0/image_raw': '/camera/left/image_raw',
    '/cam1/image_raw': '/camera/right/image_raw',
    '/imu0': '/imu/data',
}


def _to_image(obj) -> Image:
    """Convert rosbags deserialized object to sensor_msgs/Image."""
    msg = Image()
    h = _attr(obj, 'header')
    s = _attr(h, 'stamp') if h else None
    msg.header.stamp.sec = int(_attr(s, 'sec') or 0)
    msg.header.stamp.nanosec = int(_attr(s, 'nanosec') or 0)
    msg.header.frame_id = str(_attr(h, 'frame_id') or '')
    msg.height = int(_attr(obj, 'height') or 0)
    msg.width = int(_attr(obj, 'width') or 0)
    msg.encoding = str(_attr(obj, 'encoding') or '')
    msg.is_bigendian = int(_attr(obj, 'is_bigendian') or 0)
    msg.step = int(_attr(obj, 'step') or 0)
    data = _attr(obj, 'data')
    msg.data = bytes(data) if data is not None else b''
    return msg


def _to_imu(obj) -> Imu:
    """Convert rosbags deserialized object to sensor_msgs/Imu."""
    msg = Imu()
    h = _attr(obj, 'header')
    s = _attr(h, 'stamp') if h else None
    msg.header.stamp.sec = int(_attr(s, 'sec') or 0)
    msg.header.stamp.nanosec = int(_attr(s, 'nanosec') or 0)
    msg.header.frame_id = str(_attr(h, 'frame_id') or '')
    for name, field in [
        ('orientation', msg.orientation),
        ('angular_velocity', msg.angular_velocity),
        ('linear_acceleration', msg.linear_acceleration),
    ]:
        o = _attr(obj, name)
        if o is not None:
            field.x = float(_attr(o, 'x') or 0)
            field.y = float(_attr(o, 'y') or 0)
            field.z = float(_attr(o, 'z') or 0)
            if name == 'orientation':
                field.w = float(_attr(o, 'w') or 1)
    msg.orientation_covariance[0] = -1.0
    msg.angular_velocity_covariance[0] = -1.0
    msg.linear_acceleration_covariance[0] = -1.0
    return msg


def _attr(obj, name: str):
    """Get attribute from object (supports dict, object, TypedDict)."""
    if obj is None:
        return None
    if isinstance(obj, dict):
        return obj.get(name)
    return getattr(obj, name, None)


class BagPlayerNode(Node):
    """Republishes bag contents to EOS-SLAM topics."""

    def __init__(self) -> None:
        super().__init__('bag_player')
        self.declare_parameter(
            'bag_path', '',
            ParameterDescriptor(description='Path to ROS1 or ROS2 bag'),
        )
        self.declare_parameter(
            'rate', 1.0,
            ParameterDescriptor(description='Playback rate multiplier (1.0 = realtime)'),
        )

        bag_path = self.get_parameter('bag_path').get_parameter_value().string_value
        if not bag_path:
            raise ValueError('Parameter bag_path is required')
        self._bag_path = Path(bag_path).expanduser().resolve()
        self._rate = self.get_parameter('rate').get_parameter_value().double_value

        self._pub_map: dict[str, object] = {}
        self.get_logger().info(f'Playing bag: {self._bag_path} at {self._rate}x')

    def run(self) -> None:
        """Read bag and republish messages."""
        try:
            from rosbags.highlevel import AnyReader
            from rosbags.typesys import Stores, get_typestore
        except ImportError:
            self.get_logger().error('rosbags not installed. Run: pip install rosbags')
            return

        typestore = get_typestore(Stores.ROS2_HUMBLE)
        path = self._bag_path
        if path.suffix == '.bag':
            paths = [path]
        else:
            paths = [path]

        with AnyReader(paths, default_typestore=typestore) as reader:
            # Build connection -> publisher map for topics we care about
            connections_to_use = []
            conn_to_pub = {}
            for conn in reader.connections:
                out_topic = TOPIC_MAP.get(conn.topic)
                is_image = 'Image' in conn.msgtype and 'Compressed' not in conn.msgtype
                is_imu = 'Imu' in conn.msgtype
                if out_topic and (is_image or is_imu):
                    if out_topic not in self._pub_map:
                        if 'Image' in conn.msgtype:
                            self._pub_map[out_topic] = self.create_publisher(
                                Image, out_topic, 10
                            )
                        else:
                            self._pub_map[out_topic] = self.create_publisher(
                                Imu, out_topic, 10
                            )
                    connections_to_use.append(conn)
                    conn_to_pub[conn.id] = (out_topic, 'Image' in conn.msgtype)

            if not connections_to_use:
                self.get_logger().error(
                    f'No /cam0/image_raw, /cam1/image_raw, or /imu0 in bag. '
                    f'Found: {[c.topic for c in reader.connections]}'
                )
                return

            self.get_logger().info(f'Republishing {len(connections_to_use)} topics')
            start_wall = time.monotonic()
            start_bag = None
            count = 0

            for conn, timestamp, rawdata in reader.messages(connections=connections_to_use):
                if conn.id not in conn_to_pub:
                    continue
                out_topic, is_image = conn_to_pub[conn.id]
                pub = self._pub_map[out_topic]

                # Realtime pacing
                if start_bag is None:
                    start_bag = timestamp
                elapsed_bag_ns = timestamp - start_bag
                elapsed_wall = time.monotonic() - start_wall
                target_wall = (elapsed_bag_ns / 1e9) / self._rate
                sleep_time = target_wall - elapsed_wall
                if sleep_time > 0.001:
                    time.sleep(sleep_time)

                try:
                    msg_obj = reader.deserialize(rawdata, conn.msgtype)
                except Exception as e:
                    self.get_logger().warn(f'Deserialize failed: {e}')
                    continue

                if is_image:
                    ros_msg = _to_image(msg_obj)
                else:
                    ros_msg = _to_imu(msg_obj)

                pub.publish(ros_msg)
                count += 1
                if count % 1000 == 0:
                    self.get_logger().info(f'Published {count} messages')

            self.get_logger().info(f'Done. Published {count} messages')


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = BagPlayerNode()
        node.run()
    except (ValueError, RuntimeError) as e:
        import sys
        print(f'Error: {e}', file=sys.stderr)
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
