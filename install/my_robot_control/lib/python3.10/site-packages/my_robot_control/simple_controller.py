import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.cmd_pub.publish(Twist())  # safety stop on startup
        self.get_logger().info("Obstacle Avoidance Node Started!")

    def get_min(self, ranges):
        valid = [r for r in ranges if not math.isinf(r) and not math.isnan(r) and r > 0.0]
        return min(valid) if valid else None

    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        total  = len(ranges)  # should be 360

        front = ranges[160:200]
        right = ranges[60:120]
        left  = ranges[240:300]

        front_min = self.get_min(front)
        left_min  = self.get_min(left)
        right_min = self.get_min(right)

        cmd = Twist()

        if front_min is None:
            self.get_logger().info("No valid front readings. Moving forward cautiously.")
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0

        elif front_min < 0.5:
            self.get_logger().warn(f"Too close! ({front_min:.2f}m) Backing up!")
            cmd.linear.x = -0.2
            cmd.angular.z = 0.0

        elif front_min < 1.0:
            cmd.linear.x = 0.0
            left_clear  = left_min  if left_min  is not None else float('inf')
            right_clear = right_min if right_min is not None else float('inf')

            if left_clear >= right_clear:
                self.get_logger().info(
                    f"Obstacle at {front_min:.2f}m | Left:{left_clear:.2f}m Right:{right_clear:.2f}m → Turning LEFT")
                cmd.angular.z = 0.5
            else:
                self.get_logger().info(
                    f"Obstacle at {front_min:.2f}m | Left:{left_clear:.2f}m Right:{right_clear:.2f}m → Turning RIGHT")
                cmd.angular.z = -0.5

        else:
            self.get_logger().info(
                f"Path clear. Front: {front_min:.2f}m | Moving forward.")
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()