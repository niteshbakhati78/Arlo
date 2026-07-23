#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import csv, time, math

class CalibrationTester(Node):
    def __init__(self):
        super().__init__('calibration_tester')

        # Parameters
        self.declare_parameter('test_type', 'straight')
        self.declare_parameter('duration', 5.0)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.4)
        self.declare_parameter('log_file', '/tmp/calibration_log.csv')

        self.test_type = self.get_parameter('test_type').value
        self.duration = float(self.get_parameter('duration').value)
        self.vx = float(self.get_parameter('linear_speed').value)
        self.wz = float(self.get_parameter('angular_speed').value)
        self.log_file = self.get_parameter('log_file').value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.sub_left = self.create_subscription(Int32, 'left_ticks', self.on_left, 10)
        self.sub_right = self.create_subscription(Int32, 'right_ticks', self.on_right, 10)

        self.left_ticks = 0
        self.right_ticks = 0
        self.start_time = None
        self.odom_x0 = self.odom_y0 = 0.0
        self.theta0 = 0.0
        self.records = []

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"Calibration test started ({self.test_type}). Logging to {self.log_file}")

    # --------- Callbacks ----------
    def on_left(self, msg): self.left_ticks = msg.data
    def on_right(self, msg): self.right_ticks = msg.data

    def on_odom(self, msg):
        if self.start_time is None:
            self.odom_x0 = msg.pose.pose.position.x
            self.odom_y0 = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self.theta0 = math.atan2(2*(q.w*q.z), 1 - 2*(q.z*q.z))
            return
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z), 1 - 2*(q.z*q.z))
        dx = msg.pose.pose.position.x - self.odom_x0
        dy = msg.pose.pose.position.y - self.odom_y0
        self.records.append({
            't': time.time() - self.start_time,
            'x': dx,
            'y': dy,
            'yaw': yaw - self.theta0,
            'left_ticks': self.left_ticks,
            'right_ticks': self.right_ticks
        })

    # --------- Main control loop ----------
    def control_loop(self):
        msg = Twist()

        # start timer once odometry initialized
        if self.start_time is None:
            self.start_time = time.time()

        elapsed = time.time() - self.start_time
        if elapsed < self.duration:
            if self.test_type == 'straight':
                msg.linear.x = self.vx
                msg.angular.z = 0.0
            elif self.test_type == 'rotate':
                msg.linear.x = 0.0
                msg.angular.z = self.wz
            self.cmd_pub.publish(msg)
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.write_csv()
            self.get_logger().info("Test complete. CSV written.")
            rclpy.shutdown()

    # --------- Save CSV ----------
    def write_csv(self):
        keys = ['t','x','y','yaw','left_ticks','right_ticks']
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.records)
        self.get_logger().info(f"Saved {len(self.records)} samples to {self.log_file}")

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.write_csv()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
