#!/usr/bin/env python3
import time
import math
import sys

# If you need this on your system, keep it; otherwise you can remove.
sys.path.insert(0, "/usr/local/share/pynq-venv/lib/python3.10/site-packages")

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynq import Overlay


# =========================
# Robot + Encoder Parameters
# =========================
WHEEL_BASE_M = 0.3937          # meters (your value)

WHEEL_DIAMETER_M = 0.1524      # 6 inches; change if needed
WHEEL_CIRC_M = math.pi * WHEEL_DIAMETER_M

CPR_LEFT = 34.0                # confirmed
CPR_RIGHT = 32.0               # confirmed

DIST_PER_TICK_L = WHEEL_CIRC_M / CPR_LEFT
DIST_PER_TICK_R = WHEEL_CIRC_M / CPR_RIGHT


# =========================
# Control Loop Parameters (from your best plot)
# =========================
DT = 0.05                       # 20 Hz control loop
WINDOW_SAMPLES = 12             # 0.6 s window

KP = 200.0
KI = 60.0
KD = 0.0                        # keep 0 for now (derivative is noisy with low CPR)

I_LIMIT = 0.20                  # integral clamp in (m/s)*s
DEADBAND_MPS = 0.02             # treat small cmds as stop

OUT_LIMIT_US = 250.0            # max PI correction in us


# =========================
# PWM Mapping / Limits
# =========================
PWM_STOP_US = 1500
PWM_MIN_US = 1300
PWM_MAX_US = 1800

# Feedforward mapping tuning knobs (same concept as your notebook)
MAX_SPEED_MPS = 0.80

PWM_FWD_MIN_US = 1525
PWM_FWD_MAX_US = 1780

PWM_REV_MIN_US = 1475
PWM_REV_MAX_US = 1400

# Rate limiting prevents sudden jumps
MAX_STEP_US_PER_CYCLE = 30.0


# =========================
# Helper functions
# =========================
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def signed32(v):
    v &= 0xFFFFFFFF
    return v - 0x100000000 if (v & 0x80000000) else v

def velocity_to_pulse_us(v_mps):
    """Open-loop feedforward mapping: v (m/s) -> PWM us."""
    v = clamp(v_mps, -MAX_SPEED_MPS, MAX_SPEED_MPS)

    if abs(v) < DEADBAND_MPS:
        return PWM_STOP_US

    if v > 0.0:
        scale = v / MAX_SPEED_MPS
        pulse = PWM_FWD_MIN_US + (PWM_FWD_MAX_US - PWM_FWD_MIN_US) * scale
        return int(clamp(pulse, PWM_FWD_MIN_US, PWM_FWD_MAX_US))
    else:
        scale = abs(v) / MAX_SPEED_MPS
        pulse = PWM_REV_MIN_US - (PWM_REV_MIN_US - PWM_REV_MAX_US) * scale
        return int(clamp(pulse, PWM_REV_MAX_US, PWM_REV_MIN_US))


class PIController:
    """PI: error (m/s) -> correction (us)."""
    def __init__(self, kp, ki, i_limit, out_limit_us):
        self.kp = kp
        self.ki = ki
        self.i_limit = abs(i_limit)
        self.out_limit = abs(out_limit_us)
        self.i = 0.0

    def reset(self):
        self.i = 0.0

    def update(self, e, dt):
        if dt <= 1e-6:
            return 0.0
        self.i += e * dt
        self.i = clamp(self.i, -self.i_limit, self.i_limit)
        u = self.kp * e + self.ki * self.i
        return clamp(u, -self.out_limit, self.out_limit)


class WindowSpeedEstimator:
    """Computes v over WINDOW_SAMPLES*DT to avoid quantization."""
    def __init__(self, dist_per_tick, window_samples):
        self.dist_per_tick = dist_per_tick
        self.window_samples = int(window_samples)
        self.hist = []

    def reset(self):
        self.hist = []

    def update(self, ticks, dt):
        self.hist.append(ticks)
        if len(self.hist) > self.window_samples:
            old = self.hist[-self.window_samples - 1]
            dt_w = self.window_samples * dt
            return ((ticks - old) * self.dist_per_tick) / dt_w
        return 0.0


# =========================
# ROS 2 Node
# =========================
class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller_pid")

        # Load overlay + IPs
        self.overlay = Overlay("/home/ubuntu/arlo/kv260_v4.bit")
        self.pwm_ip = self.overlay.motor_pwm_0
        self.dec0 = self.overlay.quadrature_decoder_0
        self.dec1 = self.overlay.quadrature_decoder_1

        # Controllers + estimators
        self.pi_l = PIController(KP, KI, I_LIMIT, OUT_LIMIT_US)
        self.pi_r = PIController(KP, KI, I_LIMIT, OUT_LIMIT_US)

        self.est_l = WindowSpeedEstimator(DIST_PER_TICK_L, WINDOW_SAMPLES)
        self.est_r = WindowSpeedEstimator(DIST_PER_TICK_R, WINDOW_SAMPLES)

        # command state
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.last_cmd_time = time.time()

        # last PWM for rate limiting
        self.last_pwm_l = PWM_STOP_US
        self.last_pwm_r = PWM_STOP_US

        # ticks baseline
        self.prev_time = time.time()

        # subscriptions
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)

        # control loop timer
        self.timer = self.create_timer(DT, self.control_loop)

        # Safety: stop if cmd_vel goes stale
        self.cmd_timeout_s = 0.5

        # init: stop and reset
        self.stop_all()
        self.reset_encoders()
        self.get_logger().info("Closed-loop motor controller started (PI + windowed speed).")

    def reset_encoders(self):
        # your IP behavior: write 0 resets position
        self.dec0.mmio.write(0, 0)
        self.dec1.mmio.write(0, 0)
        self.est_l.reset()
        self.est_r.reset()

    def stop_all(self):
        self.pwm_ip.write(0x00, PWM_STOP_US)
        self.pwm_ip.write(0x04, PWM_STOP_US)
        self.last_pwm_l = PWM_STOP_US
        self.last_pwm_r = PWM_STOP_US
        self.pi_l.reset()
        self.pi_r.reset()

    def read_ticks_left(self):
        return signed32(self.dec0.mmio.read(0))

    def read_ticks_right(self):
        # match your convention: right inverted
        return -signed32(self.dec1.mmio.read(0))

    def cmd_vel_cb(self, msg: Twist):
        self.v_cmd = float(msg.linear.x)
        self.w_cmd = float(msg.angular.z)
        self.last_cmd_time = time.time()

    def rate_limit(self, new, old):
        step = clamp(new - old, -MAX_STEP_US_PER_CYCLE, MAX_STEP_US_PER_CYCLE)
        return int(old + step)

    def control_loop(self):
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now
        if dt <= 1e-6:
            dt = DT

        # timeout safety
        if (now - self.last_cmd_time) > self.cmd_timeout_s:
            self.stop_all()
            return

        # wheel velocity setpoints from cmd_vel
        v = self.v_cmd
        w = self.w_cmd
        v_l_cmd = v - (w * WHEEL_BASE_M / 2.0)
        v_r_cmd = v + (w * WHEEL_BASE_M / 2.0)

        # deadband stop
        if abs(v_l_cmd) < DEADBAND_MPS and abs(v_r_cmd) < DEADBAND_MPS:
            self.stop_all()
            return

        # measure wheel speeds
        ticks_l = self.read_ticks_left()
        ticks_r = self.read_ticks_right()

        v_l_meas = self.est_l.update(ticks_l, DT)
        v_r_meas = self.est_r.update(ticks_r, DT)

        # feedforward base PWM
        base_l = velocity_to_pulse_us(v_l_cmd)
        base_r = velocity_to_pulse_us(v_r_cmd)

        # PI correction
        e_l = v_l_cmd - v_l_meas
        e_r = v_r_cmd - v_r_meas

        u_l = self.pi_l.update(e_l, dt)
        u_r = self.pi_r.update(e_r, dt)

        pwm_l = int(clamp(base_l + round(u_l), PWM_MIN_US, PWM_MAX_US))
        pwm_r = int(clamp(base_r + round(u_r), PWM_MIN_US, PWM_MAX_US))

        # rate limit
        pwm_l = self.rate_limit(pwm_l, self.last_pwm_l)
        pwm_r = self.rate_limit(pwm_r, self.last_pwm_r)

        self.last_pwm_l = pwm_l
        self.last_pwm_r = pwm_r

        # write µs directly (IMPORTANT!)
        self.pwm_ip.write(0x00, pwm_l)
        self.pwm_ip.write(0x04, pwm_r)

        # optional: throttle logs (don’t spam)
        # self.get_logger().info(f"vL cmd/meas {v_l_cmd:.2f}/{v_l_meas:.2f} pwm {pwm_l} | "
        #                        f"vR cmd/meas {v_r_cmd:.2f}/{v_r_meas:.2f} pwm {pwm_r}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_all()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
