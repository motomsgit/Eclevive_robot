#!/usr/bin/env python3
"""
map→odom TF Publisher for nvblox Navigation

このノードは、nvbloxを使用したナビゲーションシステムにおいて、
map→odomのTF変換を配信します。

動作原理:
1. nvbloxは/nvblox_node/static_occupancy_gridをmapフレームで配信
2. ZED Visual Odometryはodom→zed_camera_linkのTFを配信
3. このノードがmap→odomのTFを配信し、TFツリーを完成させる

初期状態では恒等変換（map = odom）を配信し、
将来的にはSLAMのループクロージャなどで補正可能。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math

class MapOdomTfPublisher(Node):
    def __init__(self):
        super().__init__('map_odom_tf_publisher')

        # パラメータ
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('use_static_transform', True)  # 静的変換を使用

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_static_transform = self.get_parameter('use_static_transform').value

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 初期変換（恒等変換: map = odom）
        self.map_to_odom = TransformStamped()
        self.map_to_odom.header.frame_id = self.map_frame
        self.map_to_odom.child_frame_id = self.odom_frame
        self.map_to_odom.transform.translation.x = 0.0
        self.map_to_odom.transform.translation.y = 0.0
        self.map_to_odom.transform.translation.z = 0.0
        self.map_to_odom.transform.rotation.x = 0.0
        self.map_to_odom.transform.rotation.y = 0.0
        self.map_to_odom.transform.rotation.z = 0.0
        self.map_to_odom.transform.rotation.w = 1.0

        # Odometry subscriber（将来の拡張用）
        # 現在は静的変換のみだが、将来的にodomを監視して補正可能
        self.odom_sub = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_callback,
            10
        )

        # TF配信タイマー
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_tf)

        self.get_logger().info(
            f'map→odom TF Publisher started\n'
            f'  Map frame: {self.map_frame}\n'
            f'  Odom frame: {self.odom_frame}\n'
            f'  Publish rate: {self.publish_rate} Hz\n'
            f'  Mode: {"Static (Identity)" if self.use_static_transform else "Dynamic"}'
        )

    def odom_callback(self, msg: Odometry):
        """
        Odometry callback（将来の拡張用）

        現在は静的変換のみだが、将来的にはここで：
        - ループクロージャ検出
        - グローバル位置推定
        - ドリフト補正
        などを実装可能
        """
        pass

    def publish_tf(self):
        """map→odom TFを配信"""
        self.map_to_odom.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.map_to_odom)

def main(args=None):
    rclpy.init(args=args)
    node = MapOdomTfPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
