#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GroundTruthRemapper(Node):
    def __init__(self):
        super().__init__('ground_truth_remapper')

        # Input: raw pose from Gazebo / PosePublisher
        self.sub = self.create_subscription(
            PoseStamped,
            '/model/my_robot/pose',   # <--- Updated to match launch file
            self.cb,
            10
        )

        # Output: cleaned ground-truth pose for ROS 2
        self.pub = self.create_publisher(
            PoseStamped,
            '/ground_truth_pose',
            10
        )

        # Choose whatever frame makes sense in your setup: "map", "odom", "world", ...
        self.target_frame = 'odom'

    def cb(self, msg: PoseStamped):
        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = self.target_frame
        out.pose = msg.pose
        self.pub.publish(out)

def main():
    rclpy.init()
    node = GroundTruthRemapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
