#!/usr/bin/env python3
import sys
sys.path.insert(0, "/usr/local/share/pynq-venv/lib/python3.10/site-packages")

import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynq import Overlay

# ======== Robot Geometry ========
WHEEL_BASE = 0.3875     # meters (distance between wheels)
MAX_SPEED  = 0.3        # m/s (linear velocity for full PWM range)

# ======== PWM Microsecond Limits ========
PWM_STOP_US    = 1500   # Neutral (no motion)
PWM_FWD_START  = 1650   # Start forward
PWM_FWD_MAX    = 1800   # Max forward
PWM_REV_START  = 1450   # Start reverse
PWM_REV_MAX    = 1300   # Max reverse

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller_open_loop')

        # Load overlay and get IP handles
        try:
            ov = Overlay('/home/ubuntu/arlo/kv260_v4.bit')
            self.pwm_ip = ov.motor_pwm_0
            self.get_logger().info(" PWM overlay loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load overlay: {e}")
            raise

        # Subscribe to cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("Motor controller (open-loop) started.")
        self.stop_motors()

    # --- Conversion helpers ---
    def velocity_to_us(self, linear_vel):
        """Map linear velocity (m/s) to microsecond PWM value."""
        if abs(linear_vel) < 1e-3:
            return PWM_STOP_US

        scale = min(abs(linear_vel) / MAX_SPEED, 1.0)

        if linear_vel > 0:  # Forward
            return PWM_FWD_START + scale * (PWM_FWD_MAX - PWM_FWD_START)
        else:                # Reverse
            return PWM_REV_START - scale * (PWM_REV_START - PWM_REV_MAX)

    def set_motor_pwm(self, motor_id, pulse_us):
        """Write raw us pulse width to PWM IP."""
        addr = 0x00 if motor_id == 0 else 0x04
        self.pwm_ip.write(addr, int(pulse_us))

    def stop_motors(self):
        self.set_motor_pwm(0, PWM_STOP_US)
        self.set_motor_pwm(1, PWM_STOP_US)
        self.get_logger().info(" Motors stopped (1500 us neutral).")

    # --- Main callback ---
    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Compute left/right wheel linear speeds (m/s)
        v_l = linear - (angular * WHEEL_BASE / 2.0)
        v_r = linear + (angular * WHEEL_BASE / 2.0)

        # Convert to PWM µs and write
        pulse_l = self.velocity_to_us(v_l)
        pulse_r = self.velocity_to_us(v_r)

        self.set_motor_pwm(0, pulse_l)
        self.set_motor_pwm(1, pulse_r)

        self.get_logger().info(
            f"CMD_VEL: lin={linear:.3f} ang={angular:.3f} "
            f"=> L:{pulse_l:.1f}us R:{pulse_r:.1f}us"
        )

    def on_shutdown(self):
        self.stop_motors()

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
