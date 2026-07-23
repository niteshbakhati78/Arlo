#!/usr/bin/env python3
#import rclpy
#from rclpy.node import Node
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
#import cv2

#class CameraPublisher(Node):
    #def __init__(self):
        #super().__init__('camera_publisher')
        #self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        #self.bridge = CvBridge()

        # GStreamer pipeline: captures MJPEG and decodes it to RGB
        #pipeline = (
         #   "v4l2src device=/dev/video0 ! "
          #  "image/jpeg,width=640,height=480,framerate=15/1 ! "
           # "jpegdec ! videoconvert ! appsink"
        #)
        # GStreamer pipeline: requests RAW video and lets videoconvert handle it
        #pipeline = (
           # "v4l2src device=/dev/video0 ! "
          #  "video/x-raw,format=YUYV,width=640,height=480,framerate=15/1 ! "
         #   "videoconvert ! appsink"
        #)

        #self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        #self.timer = self.create_timer(0.066, self.publish_frame)  # ~15 fps
       # self.get_logger().info("Camera Publisher started")
        

#    def publish_frame(self):
 #       ret, frame = self.cap.read()
  #      if not ret:
   #         self.get_logger().warn("Failed to read frame from camera")
#            return
 #       msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
  #      self.publisher_.publish(msg)

#def main():
 #   rclpy.init()
  #  node = CameraPublisher()
   # rclpy.spin(node)
    #node.destroy_node()
    #rclpy.shutdown()

#if __name__ == '__main__':
 #   main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # === 1. FORCE 640x480 MJPG @ 5FPS ===
        # This uses the most efficient (MJPG) mode at a low, stable
        # framerate that your hardware supports (5fps).
        self.get_logger().info("Forcing camera to 640x480 MJPG @ 5fps via v4l2-ctl...")
        v4l2_cmd = [
            'v4l2-ctl',
            '-d', '/dev/video0',
            '--set-fmt-video=width=640,height=480,pixelformat=MJPG',
            '--set-parm=5'  # 5 frames per second
        ]
        try:
            subprocess.run(v4l2_cmd, check=True)
            self.get_logger().info("v4l2-ctl command successful.")
        except Exception as e:
            self.get_logger().error(f"!!! FAILED to set camera hardware settings: {e}")
            return # Don't continue if this fails
        # === END OF CONFIG ===

        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()

        # Open camera using the V4L2 backend in "read-only" mode
        self.get_logger().info("Opening pre-configured camera (V4L2 backend)...")
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2) 
        
        if not self.cap.isOpened():
            self.get_logger().error("!!! FAILED TO OPEN CAMERA 0 with V4L2 backend !!!")
            return

        # === 2. ALL 'self.cap.set(...)' COMMANDS ARE REMOVED ===
        # We MUST NOT try to set anything via OpenCV.
        # === END OF FIX ===

        # === 3. MATCH TIMER TO 5FPS ===
        timer_period = 0.2  # (1.0 / 5fps = 0.2s)
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Camera Publisher started (True Read-Only Mode @ {1.0/timer_period:.1f}Hz).")
        self.get_logger().info(f"  -> (Ignoring) OpenCV reports settings as: {int(w)}x{int(h)}")


    def publish_frame(self):
        try:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().warn("Failed to read frame (ret=False or frame=None)")
                return
        except Exception as e:
            self.get_logger().error(f"Error during self.cap.read(): {e}")
            return

        # 'cv2.read()' auto-decodes the MJPG stream into a BGR frame.
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()