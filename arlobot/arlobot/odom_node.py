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

# === Constants ===
WHEEL_DIAMETER = 0.1524  # meters 0.1524
WHEEL_BASE = 0.3875     # meters 0.3937 
COUNTS_PER_REV_L = 35
COUNTS_PER_REV_R = 35
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
DISTANCE_PER_TICK_L = WHEEL_CIRCUMFERENCE / COUNTS_PER_REV_L
DISTANCE_PER_TICK_R = WHEEL_CIRCUMFERENCE / COUNTS_PER_REV_R
#DISTANCE_PER_TICK = 0.0137
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

        self.left_sub = self.create_subscription(Int32, 'left_ticks', self.left_callback, 10)
        self.right_sub = self.create_subscription(Int32, 'right_ticks', self.right_callback, 10)
        self.left_ticks = 0
        self.right_ticks = 0

        self.timer = self.create_timer(0.1, self.update_odom)
        self.get_logger().info("Odom Node Started")

    def left_callback(self, msg):
        self.left_ticks = msg.data

    def right_callback(self, msg):
        self.right_ticks = msg.data

    def update_odom(self):
        delta_left = (self.left_ticks - self.last_left)
        delta_right = -(self.right_ticks - self.last_right)

        self.last_left = self.left_ticks
        self.last_right = self.right_ticks

        d_left = delta_left * DISTANCE_PER_TICK_L
        d_right = delta_right * DISTANCE_PER_TICK_R
        d_center = (d_left + d_right) / 2.0
        delta_theta = (d_left - d_right) / WHEEL_BASE

        self.theta += delta_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # Orientation as quaternion
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

        # Also publish the TF
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
