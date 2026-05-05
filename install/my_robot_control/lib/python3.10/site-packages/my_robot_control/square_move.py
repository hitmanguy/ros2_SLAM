#!/usr/bin/env python3
"""
Square path controller for a mobile robot in ROS 2.

- Uses /odom to measure distance and heading.
- Publishes standard Twist messages on /cmd_vel.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    """Extract yaw (rotation around Z) from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Normalize angle to the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')

        # Parameters
        self.side_length = 1.0        # meters
        self.linear_speed = 0.2       # m/s

        # Coarse turn
        self.angular_speed_coarse = 0.4     # rad/s
        self.angle_coarse_thresh = math.radians(6.0)   # switch to fine control

        # Fine turn
        self.angular_speed_fine = 0.15      # rad/s
        self.angle_tolerance_fine = math.radians(0.5)  # final tolerance

        self.dist_tolerance = 0.01   # meters

        # Internal state
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.start_x = None
        self.start_y = None

        self.turn_start_yaw = None
        self.turn_target_yaw = None

        self.state = 'WAIT_FOR_ODOM'      # WAIT_FOR_ODOM, FORWARD, TURN_COARSE, TURN_FINE, STOP
        self.sides_completed = 0
        self.total_sides = 4

        # Publisher: Standard Twist
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            # '/odom',
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        # Control loop timer: 50 Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("SquareMover node started. Waiting for /odom...")

    def odom_callback(self, msg: Odometry):
        # Update current pose
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        # Initialize start pose once
        if self.state == 'WAIT_FOR_ODOM':
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.state = 'FORWARD'
            self.get_logger().info("Received first odom. Starting square motion.")

    def start_turn(self):
        """Initialize a new 90° turn from current yaw."""
        self.turn_start_yaw = self.current_yaw
        self.turn_target_yaw = normalize_angle(self.turn_start_yaw + math.pi / 2.0)
        self.state = 'TURN_COARSE'
        self.get_logger().info(
            f"Starting turn. Target yaw: {math.degrees(self.turn_target_yaw):.2f} deg"
        )

    def control_turn(self, coarse: bool) -> Twist:
        """Return a Twist command for coarse or fine turning."""
        twist = Twist()

        # Compute error to fixed target_yaw
        error = normalize_angle(self.turn_target_yaw - self.current_yaw)

        if coarse:
            # Coarse phase: get within angle_coarse_thresh
            if abs(error) < self.angle_coarse_thresh:
                # Done with coarse; switch to fine
                self.state = 'TURN_FINE'
                self.get_logger().info("Coarse turn done, switching to fine control.")
                return twist  # zero command this cycle

            k_ang = 1.2
            ang_cmd = k_ang * error
            # Saturate
            ang_cmd = max(min(ang_cmd, self.angular_speed_coarse), -self.angular_speed_coarse)
            twist.angular.z = ang_cmd
            return twist

        else:
            # Fine phase: precise turn to small tolerance
            if abs(error) < self.angle_tolerance_fine:
                # Turn completed
                twist.angular.z = 0.0
                self.sides_completed += 1
                self.get_logger().info(
                    f"Turn completed. Sides done: {self.sides_completed}/{self.total_sides}"
                )

                if self.sides_completed >= self.total_sides:
                    self.state = 'STOP'
                    self.get_logger().info("Square completed. Stopping.")
                else:
                    # Next side
                    self.start_x = self.current_x
                    self.start_y = self.current_y
                    self.state = 'FORWARD'
                return twist

            k_ang = 1.0
            ang_cmd = k_ang * error
            ang_cmd = max(min(ang_cmd, self.angular_speed_fine), -self.angular_speed_fine)
            twist.angular.z = ang_cmd
            return twist

    def control_loop(self):
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            # No odom yet
            return

        twist = Twist()

        if self.state == 'FORWARD':
            # Distance from the starting point of this side
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist >= self.side_length - self.dist_tolerance:
                # Stop and start a turn
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)

                self.start_turn()
                return
            else:
                # Drive straight
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0

        elif self.state == 'TURN_COARSE':
            twist = self.control_turn(coarse=True)

        elif self.state == 'TURN_FINE':
            twist = self.control_turn(coarse=False)

        elif self.state == 'STOP':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        else:
            # Unknown state, make sure we don't move
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish the raw Twist message
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SquareMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        # Final stop command
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()