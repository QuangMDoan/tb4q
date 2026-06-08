"""Re-broadcast the most recent map->odom transform when the localizer goes silent.

Problem: over an unreliable Wi-Fi link the user-PC AMCL may stop receiving scans
from the robot for several seconds. While silent, AMCL stops publishing
map->odom, the TF cache entry expires, and every Nav2 lookup that crosses
map<->odom starts failing. The system then cannot recover even after scans
resume, because the BT, controller and planner can no longer resolve the robot
pose.

This node listens on /tf, caches the latest map->odom emitted by AMCL, and if
AMCL goes silent for longer than `silence_threshold_sec` re-broadcasts the
cached transform with a fresh local timestamp at `republish_rate_hz`. As soon
as AMCL resumes, the keepalive backs off and AMCL's own publications take over.

Fail-safe: if AMCL has been silent for longer than `max_extrapolation_sec`, the
keepalive stops re-broadcasting so downstream consumers fail loudly instead of
running on arbitrarily stale pose.
"""

from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


class TFKeepalive(Node):
    def __init__(self) -> None:
        super().__init__('tf_keepalive')

        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('silence_threshold_sec', 0.3)
        self.declare_parameter('republish_rate_hz', 10.0)
        self.declare_parameter('max_extrapolation_sec', 5.0)

        self._source = self.get_parameter('source_frame').value
        self._target = self.get_parameter('target_frame').value
        self._silence_threshold = float(
            self.get_parameter('silence_threshold_sec').value
        )
        self._max_extrap = float(self.get_parameter('max_extrapolation_sec').value)
        rate = float(self.get_parameter('republish_rate_hz').value)

        self._lock = Lock()
        self._cached = None
        self._last_amcl_ns = None
        self._fallback_active = False
        self._given_up = False

        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._broadcaster = TransformBroadcaster(self)
        self._tf_sub = self.create_subscription(
            TFMessage, '/tf', self._on_tf, tf_qos
        )
        self._timer = self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f'Watching {self._source}->{self._target}: silence>'
            f'{self._silence_threshold:.2f}s triggers re-broadcast at '
            f'{rate:.1f} Hz, max stale={self._max_extrap:.1f}s'
        )

    def _on_tf(self, msg: TFMessage) -> None:
        now_ns = self.get_clock().now().nanoseconds
        for t in msg.transforms:
            if t.header.frame_id != self._source or t.child_frame_id != self._target:
                continue
            with self._lock:
                self._cached = t
                self._last_amcl_ns = now_ns
                if self._fallback_active:
                    self.get_logger().info(
                        f'{self._source}->{self._target} resumed; '
                        'disabling keepalive'
                    )
                    self._fallback_active = False
                if self._given_up:
                    self._given_up = False
            return

    def _on_timer(self) -> None:
        with self._lock:
            cached = self._cached
            last_ns = self._last_amcl_ns

        if cached is None or last_ns is None:
            return

        now = self.get_clock().now()
        silence_sec = (now.nanoseconds - last_ns) / 1e9

        if silence_sec < self._silence_threshold:
            return

        if silence_sec > self._max_extrap:
            with self._lock:
                if not self._given_up:
                    self.get_logger().error(
                        f'{self._source}->{self._target} silent for '
                        f'{silence_sec:.2f}s > max {self._max_extrap:.2f}s; '
                        'stopping keepalive (downstream will fail-safe)'
                    )
                    self._given_up = True
                self._fallback_active = False
            return

        if not self._fallback_active:
            self.get_logger().warn(
                f'{self._source}->{self._target} silent for '
                f'{silence_sec:.2f}s; re-broadcasting cached transform'
            )
            with self._lock:
                self._fallback_active = True

        cached.header.stamp = now.to_msg()
        self._broadcaster.sendTransform(cached)


def main() -> None:
    rclpy.init()
    node = TFKeepalive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
