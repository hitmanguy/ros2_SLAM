#!/usr/bin/env python3
"""
Reads /odom (nav_msgs/Odometry) and publishes the
odom → base_footprint transform to /tf.

Why this exists: the ros_gz_bridge /tf bridge using
ignition.msgs.Pose_V is unreliable on many ROS 2 Humble
installs — the DiffDrive plugin's TF never arrives in ROS.
This node is the robust workaround: /odom always bridges
correctly, so we re-derive the TF from it here.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.br = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.get_logger().info('odom_tf_broadcaster started — publishing odom→base_footprint TF')

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()

        # Use the same timestamp as the odometry message
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        # Copy position
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Copy orientation
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
