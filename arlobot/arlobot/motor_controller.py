#!/usr/bin/env python3
import sys
sys.path.insert(0, "/usr/local/share/pynq-venv/lib/python3.10/site-packages")
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynq import Overlay

# === Robot Parameters ===
WHEEL_BASE = 0.3937  # meters
MAX_SPEED = 0.3      # max linear speed in m/s (tune if needed)

# === Motor PWM Calibration (based on testing) ===
PWM_STOP = 50

PWM_FWD_MIN = 1650   # slow forward
PWM_FWD_MAX = 1800   # fast forward

PWM_REV_MIN = 1450   # slow reverse
PWM_REV_MAX = 1300   # fast reverse

def pulse_us_to_percent(pulse_us):
    return int((pulse_us / 20000.0) * 100)

def velocity_to_pwm(linear_vel):
    if abs(linear_vel) < 0.01:
        return PWM_STOP

    if linear_vel > 0:
        scale = min(linear_vel / MAX_SPEED, 1.0)
        pulse = PWM_FWD_MIN + (PWM_FWD_MAX - PWM_FWD_MIN) * scale
        return int(min(PWM_FWD_MAX, max(PWM_FWD_MIN, pulse)))

    else:
        scale = min(abs(linear_vel) / MAX_SPEED, 1.0)
        pulse = PWM_REV_MIN - (PWM_REV_MIN - PWM_REV_MAX) * scale
        return int(max(PWM_REV_MAX, min(PWM_REV_MIN, pulse)))

# === Load FPGA Overlay ===
overlay = Overlay("/home/ubuntu/arlo/kv260_v4.bit")
pwm_ip = overlay.motor_pwm_0

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("Motor Controller Node Started")

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        left_speed = linear - (angular * WHEEL_BASE / 2)
        right_speed = linear + (angular * WHEEL_BASE / 2)

        left_pwm_us = velocity_to_pwm(left_speed)
        right_pwm_us = velocity_to_pwm(right_speed)

        left_duty = pulse_us_to_percent(left_pwm_us)
        right_duty = pulse_us_to_percent(right_pwm_us)

        pwm_ip.write(0x00, left_duty)
        pwm_ip.write(0x04, right_duty)

        self.get_logger().info(f"PWM L: {left_pwm_us} µs ({left_duty}%) | PWM R: {right_pwm_us} µs ({right_duty}%)")

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
