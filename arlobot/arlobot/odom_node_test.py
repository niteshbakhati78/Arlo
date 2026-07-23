#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time

# === Physical Constants ===
WHEEL_DIAMETER = 0.1524  # meters
WHEEL_BASE = 0.3875     # meters

# Assuming your original configuration of (35, 36) was correct for your hardware
COUNTS_PER_REV_L = 35
COUNTS_PER_REV_R = 35

# === Calibration Factor ===
# Since the robot drifts RIGHT, the RIGHT wheel travels too far.
# We reduce the reported distance per tick for the RIGHT wheel in odometry.
# Start low (e.g., 0.99) and tune up or down until the robot tracks straight.
RIGHT_TICK_CALIBRATION_FACTOR = 1.2055

# === Derived Constants ===
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER

DISTANCE_PER_TICK_L = WHEEL_CIRCUMFERENCE / COUNTS_PER_REV_L
# Apply the calibration factor to the right wheel's distance calculation
DISTANCE_PER_TICK_R = (WHEEL_CIRCUMFERENCE / COUNTS_PER_REV_R) * RIGHT_TICK_CALIBRATION_FACTOR


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.br = TransformBroadcaster(self)
        self.last_left = 0
        self.last_right = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Subscriptions for encoder ticks
        self.left_sub = self.create_subscription(Int32, 'left_ticks', self.left_callback, 10)
        self.right_sub = self.create_subscription(Int32, 'right_ticks', self.right_callback, 10)
        self.left_ticks = 0
        self.right_ticks = 0

        # Timer for odometry update loop
        self.timer = self.create_timer(0.1, self.update_odom)
        self.get_logger().info("Odom Node Started with Right Wheel Calibration Factor: %.3f" % RIGHT_TICK_CALIBRATION_FACTOR)

    def left_callback(self, msg):
        self.left_ticks = msg.data

    def right_callback(self, msg):
        self.right_ticks = msg.data

    def update_odom(self):
        # Calculate tick differences since last update
        delta_left = (self.left_ticks - self.last_left)
        # delta_right is negative because your right encoder counts decrease for forward motion
        delta_right = -(self.right_ticks - self.last_right) 

        self.last_left = self.left_ticks
        self.last_right = self.right_ticks

        # Calculate linear distance travelled by each wheel
        d_left = delta_left * DISTANCE_PER_TICK_L
        d_right = delta_right * DISTANCE_PER_TICK_R
        
        # Differential Drive Kinematics
        d_center = (d_left + d_right) / 2.0
        delta_theta = (d_left - d_right) / WHEEL_BASE

        # Update pose
        self.theta += delta_theta
        # Simple integration using current angle (accurate for small dt)
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # Keep theta normalized (optional, but good practice)
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Orientation as quaternion (Yaw-only)
        q = Quaternion()
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q

        self.odom_pub.publish(odom)

        # Also publish the TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
