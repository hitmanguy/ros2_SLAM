#!/usr/bin/env python3

"""
ROS 2 version of the noisy odometry node.

Original ROS 1 code:
Modified version of the original from: Team Leonard, University of Birmingham Intelligent Robotics 2018
"""

import math
import random
from random import gauss

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_srvs.srv import Empty

# sl = standard deviation of the linear velocity Gaussian noise
# sa = standard deviation of the angular velocity Gaussian noise
sl, sa = 0.1, 0.5


def multiply_quaternions(qa, qb):
    """
    Multiplies two quaternions to give the rotation of qb by qa.

    :Args:
       | qa (geometry_msgs.msg.Quaternion): rotation amount to apply to qb
       | qb (geometry_msgs.msg.Quaternion): to rotate by qa
    :Return:
       | (geometry_msgs.msg.Quaternion): qb rotated by qa.
    """
    combined = Quaternion()

    combined.w = (qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z)
    combined.x = (qa.x * qb.w + qa.w * qb.x + qa.y * qb.z - qa.z * qb.y)
    combined.y = (qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x)
    combined.z = (qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w)
    return combined


def rotateQuaternion(q_orig, yaw):
    """
    Converts a basic rotation about the z-axis (in radians) into the
    Quaternion notation required by ROS transform and pose messages.

    :Args:
       | q_orig (geometry_msgs.msg.Quaternion): to be rotated
       | yaw (double): rotate by this amount in radians
    :Return:
       | (geometry_msgs.msg.Quaternion) q_orig rotated yaw about the z axis
    """
    # Create a temporary Quaternion to represent the change in heading
    q_headingChange = Quaternion()

    p = 0.0
    y = yaw / 2.0
    r = 0.0

    sinp = math.sin(p)
    siny = math.sin(y)
    sinr = math.sin(r)
    cosp = math.cos(p)
    cosy = math.cos(y)
    cosr = math.cos(r)

    q_headingChange.x = sinr * cosp * cosy - cosr * sinp * siny
    q_headingChange.y = cosr * sinp * cosy + sinr * cosp * siny
    q_headingChange.z = cosr * cosp * siny - sinr * sinp * cosy
    q_headingChange.w = cosr * cosp * cosy + sinr * sinp * siny

    # Multiply new (heading-only) quaternion by the existing (pitch and bank)
    # quaternion. Order is important!
    return multiply_quaternions(q_headingChange, q_orig)


def getHeading(q):
    """
    Get the robot heading in radians from a Quaternion representation.

    :Args:
        | q (geometry_msgs.msg.Quaternion): an orientation about the z-axis
    :Return:
        | (double): Equivalent orientation about the z-axis in radians
    """
    yaw = math.atan2(
        2.0 * (q.x * q.y + q.w * q.z),
        q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z
    )
    return yaw


def simple_gaussian(odom):
    """
    Applies simple gaussian noise to current position and odometry readings.
    (Unused in closed_loop_problem but kept for completeness.)
    """
    sp, sr = 0.01, 0.008
    pos = odom.pose.pose.position
    odom.pose.pose.position = Point(gauss(pos.x, sp), gauss(pos.y, sp), gauss(pos.z, sp))
    rot = odom.pose.pose.orientation
    odom.pose.pose.orientation = Quaternion(
        gauss(rot.x, sr), gauss(rot.y, sr), gauss(rot.z, sr), gauss(rot.w, sr)
    )
    return odom


class NoisyOdomNode(Node):
    def __init__(self):
        super().__init__('noisy_odom')

        # Publisher and subscriber
        self.pub = self.create_publisher(Odometry, '/odom_noisy', 10)
        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10
        )

        # Shutdown service
        self.shutdown_srv = self.create_service(
            Empty,
            'noisy_odom/shutdown',
            self.shutdown_callback
        )

        # Internal state for closed-loop odom and shutdown flag
        self.cl_odom = None
        self.shutdown_flag = False

        # Timer to check for shutdown flag (similar to the while loop in ROS 1)
        self.shutdown_timer = self.create_timer(0.5, self.shutdown_timer_cb)

        self.get_logger().info("Started noisy odometry publisher node")

    def shutdown_timer_cb(self):
        if self.shutdown_flag:
            self.get_logger().info("Shutting down noisy odometry node...")
            # This triggers spin() to return
            rclpy.shutdown()

    def closed_loop_problem(self, odom: Odometry):
        """
        Using the linear and angular velocities extracted from each odometry
        message: add noise to these velocities and add them to the current
        fictional position to keep track independently of the positions
        reported by the odometry.
        """
        # If cl_odom is not defined, then it must be the first callback
        if self.cl_odom is None:
            self.cl_odom = odom
            return

        # Get velocities
        lv = odom.twist.twist.linear
        av = odom.twist.twist.angular

        # Compute dt from header timestamps (ROS 2 builtin_interfaces/Time)
        t = odom.header.stamp
        t_prev = self.cl_odom.header.stamp
        dt = float(t.sec - t_prev.sec) + 1e-9 * float(t.nanosec - t_prev.nanosec)
        if dt < 0.0:
            dt = 0.0

        # Add noise to velocities
        #lv = Vector3(gauss(lv.x, sl), gauss(lv.y, sl), lv.z)
        lv = Vector3(
            x=gauss(lv.x, sl),
            y=gauss(lv.y, sl),
            z=lv.z
        )
        #av = Vector3(av.x, av.y, gauss(av.z, abs(av.z) * sa if av.z != 0.0 else sa))
        av = Vector3(
            x=av.x,
            y=av.y,
            z=gauss(av.z, abs(av.z) * sa if av.z != 0.0 else sa)
        )

        # Apply velocities to orientation of last location
        cl_ori = self.cl_odom.pose.pose.orientation
        odom.pose.pose.orientation = rotateQuaternion(cl_ori, av.z * dt)
        odom.twist.twist.angular = av
        yaw = getHeading(odom.pose.pose.orientation) % (2.0 * math.pi)

        # Apply velocities to position of last location
        cl_pos = self.cl_odom.pose.pose.position
        fwd, drift = lv.x * dt, lv.y * dt
        c = math.cos(yaw)
        s = math.sin(yaw)
        odom.pose.pose.position.x = cl_pos.x + c * fwd + s * drift
        odom.pose.pose.position.y = cl_pos.y + s * fwd + c * drift
        odom.twist.twist.linear = lv

        # Update stored closed-loop odom
        self.cl_odom = odom

    def odometry_callback(self, odom: Odometry):
        self.get_logger().debug(
            f"Got perfect odom: {odom.pose.pose.position.x} {odom.pose.pose.position.y}"
        )
        # Add noise to the odometry
        self.closed_loop_problem(odom)
        # Republish
        self.pub.publish(odom)
        self.get_logger().debug(
            f"Pub noisy odom: {odom.pose.pose.position.x} {odom.pose.pose.position.y}"
        )

    def shutdown_callback(self, request, response):
        # Set shutdown flag
        self.get_logger().info("Shutdown service called; will shut down soon.")
        self.shutdown_flag = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NoisyOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.get_logger().info("Destroying node...")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()