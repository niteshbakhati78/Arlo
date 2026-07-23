#!/usr/bin/env python3
import sys
sys.path.insert(0, "/usr/local/share/pynq-venv/lib/python3.10/site-packages")
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from pynq import Overlay
import time

# === Load FPGA Overlay ===
overlay = Overlay("/home/ubuntu/arlo/kv260_v4.bit")
encoder_left = overlay.quadrature_decoder_0
encoder_right = overlay.quadrature_decoder_1

# === Helper Functions ===
def signed32(val):
    val = val & 0xFFFFFFFF
    return val - 0x100000000 if val & 0x80000000 else val

def clamp_int32(val):
    return max(-2147483648, min(2147483647, int(val)))

class EncoderLogger(Node):
    def __init__(self):
        super().__init__('encoder_logger')
        self.left_pub = self.create_publisher(Int32, 'left_ticks', 50)
        self.right_pub = self.create_publisher(Int32, 'right_ticks', 50)

        # Offsets captured once at startup to make counts start at 0
        self.left_offset = None
        self.right_offset = None
        self.zeroed_logged = False

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Encoder Logger Node Started (auto-zero on start)")

    def timer_callback(self):
        try:
            raw_left = clamp_int32(signed32(encoder_left.read(0x00)))
            raw_right = clamp_int32(signed32(encoder_right.read(0x00)))

            # Capture offsets the first time we read, so published ticks start at 0
            if self.left_offset is None or self.right_offset is None:
                self.left_offset = raw_left
                self.right_offset = raw_right
                if not self.zeroed_logged:
                    self.get_logger().info(
                        f"Encoders zeroed. Offsets -> left:{self.left_offset} right:{self.right_offset}"
                    )
                    self.zeroed_logged = True

            # Apply offsets so each launch starts from 0
            left_ticks = clamp_int32(raw_left - self.left_offset)
            right_ticks = clamp_int32(raw_right - self.right_offset)

            self.left_pub.publish(Int32(data=left_ticks))
            self.right_pub.publish(Int32(data=right_ticks))

            self.get_logger().info(f"Encoder Ticks | Left: {left_ticks} | Right: {right_ticks}")
        except Exception as e:
            self.get_logger().error(f"Error reading encoders: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
