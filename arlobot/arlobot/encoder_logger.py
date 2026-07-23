#!/usr/bin/env python3
import sys
sys.path.insert(0, "/usr/local/share/pynq-venv/lib/python3.10/site-packages")
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from pynq import Overlay
import time

# === Load FPGA Overlay ===
#bitfile = "/home/ubuntu/overlays/design_for_kv260.bit"
#hwhfile = "/home/ubuntu/overlays/design_for_kv260.hwh"
#overlay = Overlay(bitfile_name=bitfile)
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
        self.left_pub = self.create_publisher(Int32, 'left_ticks', 10)
        self.right_pub = self.create_publisher(Int32, 'right_ticks', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Encoder Logger Node Started")

    def timer_callback(self):
        try:
            left_ticks = clamp_int32(signed32(encoder_left.read(0x00)))
            right_ticks = clamp_int32(signed32(encoder_right.read(0x00)))

            left_msg = Int32()
            left_msg.data = left_ticks
            self.left_pub.publish(left_msg)

            right_msg = Int32()
            right_msg.data = right_ticks
            self.right_pub.publish(right_msg)

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
