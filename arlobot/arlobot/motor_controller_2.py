#!/usr/bin/env python3
import sys
sys.path.insert(0, "/usr/local/share/pynq-venv/lib/python3.10/site-packages")
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynq import Overlay
import time

# === Constants ===
TICKS_PER_REV_L = 35
TICKS_PER_REV_R = 36
PWM_PERIOD_US = 20000
PWM_NEUTRAL_US = 100  # Your experimentally tested neutral
TRIM_L_US = -5
TRIM_R_US = +5

# === Pulse range (from your working setup) ===
FWD_START_L = 1625
FWD_START_R = 1615
REV_START_L = 1480
REV_START_R = 1480
FWD_MAX_US = 1800
REV_MAX_US = 1400
MAX_PWM_US = FWD_MAX_US
MIN_PWM_US = REV_MAX_US

# === Helper functions ===
def pulse_us_to_percent(pulse_us):
    return int((pulse_us / PWM_PERIOD_US) * 100)

def signed32(val):
    val = val & 0xFFFFFFFF
    return val - 0x100000000 if val & 0x80000000 else val

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Load overlay and get IP handles
        overlay = Overlay('/home/ubuntu/arlo/kv260_v4.bit')
        self.pwm_ip = overlay.motor_pwm_0
        self.decoder_l = overlay.quadrature_decoder_0
        self.decoder_r = overlay.quadrature_decoder_1

        # ROS 2 subscriber
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Encoder tracking
        self.prev_ticks_l = self.read_encoder(0)
        self.prev_ticks_r = self.read_encoder(1)
        self.prev_time = time.time()

        # Start timer for RPM debug
        self.create_timer(0.5, self.debug_rpm)

        # Stop motors at init
        self.stop_motors()
        self.get_logger().info("Motor controller with RPM debug started.")

    def cmd_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z
        wheel_base = 0.3937

        v_l = linear - (angular * wheel_base / 2.0)
        v_r = linear + (angular * wheel_base / 2.0)

        pulse_l = self.velocity_to_pwm(v_l, side='L') + TRIM_L_US
        pulse_r = self.velocity_to_pwm(v_r, side='R') + TRIM_R_US

        self.set_motor_pwm(0, pulse_l)
        self.set_motor_pwm(1, pulse_r)

    def velocity_to_pwm(self, velocity, side='L'):
        if abs(velocity) < 1e-3:
            return PWM_NEUTRAL_US

        if velocity > 0:
            start = FWD_START_L if side == 'L' else FWD_START_R
            span = FWD_MAX_US - start
            pwm = start + span * velocity  # you can normalize velocity if needed
            return min(FWD_MAX_US, int(pwm))
        else:
            start = REV_START_L if side == 'L' else REV_START_R
            span = start - REV_MAX_US
            pwm = start - span * abs(velocity)
            return max(REV_MAX_US, int(pwm))

    def set_motor_pwm(self, motor_id, pulse_us):
        duty = pulse_us_to_percent(pulse_us)
        addr = 0x00 if motor_id == 0 else 0x04
        self.pwm_ip.write(addr, duty)

    def stop_motors(self):
        self.set_motor_pwm(0, PWM_NEUTRAL_US)
        self.set_motor_pwm(1, PWM_NEUTRAL_US)

    def read_encoder(self, motor_id):
        raw = self.decoder_l.mmio.read(0) if motor_id == 0 else self.decoder_r.mmio.read(0)
        return signed32(raw) if motor_id == 0 else -signed32(raw)

    def debug_rpm(self):
        now = time.time()
        dt = now - self.prev_time
        if dt < 0.01:
            return

        curr_l = self.read_encoder(0)
        curr_r = self.read_encoder(1)
        delta_l = curr_l - self.prev_ticks_l
        delta_r = curr_r - self.prev_ticks_r

        rpm_l = (delta_l / TICKS_PER_REV_L) / (dt / 60.0)
        rpm_r = (delta_r / TICKS_PER_REV_R) / (dt / 60.0)

        self.get_logger().info(f"RPM | Left: {rpm_l:.2f}, Right: {rpm_r:.2f} | ticks = ({delta_l}, {delta_r})")

        self.prev_ticks_l = curr_l
        self.prev_ticks_r = curr_r
        self.prev_time = now

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
