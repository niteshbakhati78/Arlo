#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

# ==== Geometry & encoders ====
WHEEL_DIAMETER   = 0.1524   # m
WHEEL_BASE       = 0.3937   # m  (match motor_controller)
TICKS_PER_REV_L  = 35
TICKS_PER_REV_R  = 36
WHEEL_CIRC       = math.pi * WHEEL_DIAMETER
DIST_PER_TICK_L  = WHEEL_CIRC / float(TICKS_PER_REV_L)
DIST_PER_TICK_R  = WHEEL_CIRC / float(TICKS_PER_REV_R)

# Direction sign (set once if your wiring yields opposite)
SIGN_L = +1
SIGN_R = -1

DT = 0.02  # 50 Hz

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.br = TransformBroadcaster(self)

        self.x = 0.0; self.y = 0.0; self.th = 0.0
        self.left_ticks = 0; self.right_ticks = 0
        self.last_left = None; self.last_right = None

        self.create_subscription(Int32, 'left_ticks',  self.left_cb,  50)
        self.create_subscription(Int32, 'right_ticks', self.right_cb, 50)

        self.create_timer(DT, self.update)
        self.get_logger().info("Odom node @50Hz started")

    def left_cb(self, msg):  self.left_ticks  = int(msg.data)
    def right_cb(self, msg): self.right_ticks = int(msg.data)

    @staticmethod
    def to_quat(yaw):
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    def update(self):
        if self.last_left is None:
            self.last_left  = self.left_ticks
            self.last_right = self.right_ticks
            return

        dlt = SIGN_L * (self.left_ticks  - self.last_left)
        drt = SIGN_R * (self.right_ticks - self.last_right)
        self.last_left  = self.left_ticks
        self.last_right = self.right_ticks

        dL = dlt * DIST_PER_TICK_L
        dR = drt * DIST_PER_TICK_R

        dC  = 0.5 * (dL + dR)
        dTh = (dL - dR) / WHEEL_BASE

        self.th += dTh
        self.x  += dC * math.cos(self.th)
        self.y  += dC * math.sin(self.th)

        q = self.to_quat(self.th)
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
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
