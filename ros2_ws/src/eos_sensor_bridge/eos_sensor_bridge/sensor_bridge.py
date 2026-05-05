import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_msgs.msg import TFMessage


SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
)

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
)

TOPIC_MAPPING = {
    '/scene_camera/color/image_raw':      {'out': '/eos/input/rgb',             'type': Image,             'qos': SENSOR_QOS},
    '/scene_camera/depth/image_rect_raw':  {'out': '/eos/input/depth',           'type': Image,             'qos': SENSOR_QOS},
    '/scene_camera/color/camera_info':     {'out': '/eos/input/camera_info',     'type': CameraInfo,        'qos': RELIABLE_QOS},
    '/scene_camera/depth/color/points':    {'out': '/eos/input/points',          'type': PointCloud2,       'qos': SENSOR_QOS},
    '/odom':                               {'out': '/eos/input/odom',            'type': Odometry,          'qos': RELIABLE_QOS},
    '/mid360_front':                       {'out': '/eos/input/lidar_front',     'type': PointCloud2,       'qos': SENSOR_QOS},
    '/mid360_back':                        {'out': '/eos/input/lidar_back',      'type': PointCloud2,       'qos': SENSOR_QOS},
}


class SensorBridgeNode(Node):
    def __init__(self):
        super().__init__('eos_sensor_bridge')

        self.declare_parameter('topic_overrides', [])
        self.declare_parameter('bridge_tf', True)
        self.declare_parameter('status_period', 5.0)

        self._relay_pubs = {}
        self._relay_subs = {}
        self._msg_counts = {}

        self._create_relay_chains()

        if self.get_parameter('bridge_tf').value:
            self._create_tf_relay()

        status_period = self.get_parameter('status_period').value
        self._status_timer = self.create_timer(status_period, self._publish_status)
        self._status_pub = self.create_publisher(
            PointStamped, '/eos/bridge/status', 10)

        self.get_logger().info(f'Sensor bridge active: {len(self._relay_subs)} topic pairs')
        for src, info in TOPIC_MAPPING.items():
            self.get_logger().info(f'  {src} -> {info["out"]}')

    def _create_relay_chains(self):
        topic_overrides = self.get_parameter('topic_overrides').value
        overrides = {}
        for entry in topic_overrides:
            if '=' in entry:
                src, dst = entry.split('=', 1)
                overrides[src.strip()] = dst.strip()

        for src_topic, config in TOPIC_MAPPING.items():
            out_topic = overrides.get(src_topic, config['out'])
            msg_type = config['type']
            qos = config['qos']

            pub = self.create_publisher(msg_type, out_topic, qos)
            self._relay_pubs[src_topic] = pub
            self._msg_counts[src_topic] = 0

            sub = self.create_subscription(
                msg_type, src_topic,
                self._make_callback(src_topic, out_topic, msg_type),
                qos)
            self._relay_subs[src_topic] = sub

            self.get_logger().debug(f'Relay: {src_topic} -> {out_topic}')

    def _make_callback(self, src, dst, msg_type):
        def callback(msg):
            self._relay_pubs[src].publish(msg)
            self._msg_counts[src] += 1
        return callback

    def _create_tf_relay(self):
        for tf_topic in ['/tf', '/tf_static']:
            pub = self.create_publisher(TFMessage, '/eos/' + tf_topic.lstrip('/'), 100)
            sub = self.create_subscription(
                TFMessage, tf_topic,
                lambda msg, p=pub: p.publish(msg),
                100)
            self.get_logger().info(f'TF relay: {tf_topic} -> /eos/{tf_topic.lstrip("/")}')

    def _publish_status(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'eos_bridge'
        total = sum(self._msg_counts.values())
        msg.point.x = float(total)
        self._status_pub.publish(msg)
        self.get_logger().debug(f'Bridge status: {total} total messages')

    def get_topic_stats(self):
        return dict(self._msg_counts)


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down sensor bridge')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
