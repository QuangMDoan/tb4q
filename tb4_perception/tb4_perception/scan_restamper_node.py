"""Re-publish /scan with the local PC's current clock time.

Workaround for unreliable Wi-Fi between Raspberry Pi and user PC: scan messages
can arrive seconds late, which breaks Nav2 TF lookups in costmaps and
collision_monitor. Re-stamping with rclpy.Clock.now() decouples those consumers
from network jitter. AMCL should keep using the original /scan so localization
remains physically correct.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanRestamper(Node):
    def __init__(self) -> None:
        super().__init__('scan_restamper')

        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_restamped')
        self.declare_parameter('max_age_sec', 2.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._max_age = float(self.get_parameter('max_age_sec').value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(LaserScan, output_topic, sensor_qos)
        self._sub = self.create_subscription(
            LaserScan, input_topic, self._cb, sensor_qos
        )

        self._dropped = 0
        self.get_logger().info(
            f'Re-stamping {input_topic} -> {output_topic} (drop if older than '
            f'{self._max_age:.2f}s)'
        )

    def _cb(self, msg: LaserScan) -> None:
        now = self.get_clock().now()
        msg_time_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age_sec = (now.nanoseconds - msg_time_ns) / 1e9

        if age_sec > self._max_age:
            self._dropped += 1
            if self._dropped % 20 == 1:
                self.get_logger().warn(
                    f'Dropping stale scan (age {age_sec:.2f}s > '
                    f'{self._max_age:.2f}s); total dropped={self._dropped}'
                )
            return

        msg.header.stamp = now.to_msg()
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ScanRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
