import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Publisher for robot velocity
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for LiDAR data
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
        )

        self.get_logger().info("Obstacle Avoidance Node Started! Looking for walls...")

    def scan_callback(self, msg):
        # The LiDAR publishes 360 samples. Index 0 is directly in front of the robot.
        # We slice the array to get the 20 degrees to the left and 20 degrees to the right.
        front_ranges = msg.ranges[-20:] + msg.ranges[:20]
        
        # Filter out 'inf' (infinity) and 'NaN' (not a number) values
        valid_ranges = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r)]

        cmd = Twist()

        # If we have valid readings and the closest wall is less than 1.0 meter away
        if len(valid_ranges) > 0 and min(valid_ranges) < 1.0:
            self.get_logger().info(f"Wall detected at {min(valid_ranges):.2f}m! Turning left...")
            cmd.linear.x = 0.0      # Stop moving forward
            cmd.angular.z = 0.5     # Spin counter-clockwise
        else:
            self.get_logger().info("Path clear. Moving forward.")
            cmd.linear.x = 0.5      # Move forward
            cmd.angular.z = 0.0     # Stop spinning

        # Send the command to the wheels
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()