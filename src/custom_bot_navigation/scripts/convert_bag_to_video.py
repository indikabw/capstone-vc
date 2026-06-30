#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import subprocess
import threading
import time

class VideoConverter(Node):
    def __init__(self, output_filename, topic):
        super().__init__('video_converter')
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.video_writer = None
        self.output_filename = output_filename
        self.get_logger().info(f'Waiting for frames on {topic}...')
        self.last_frame_time = time.time()

    def listener_callback(self, msg):
        self.last_frame_time = time.time()
        # Convert ROS2 Image message to OpenCV format
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge exception: {e}")
            return
            
        # Initialize VideoWriter dynamically with the correct resolution on the first frame
        if self.video_writer is None:
            height, width, _ = cv_img.shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Codec for MP4
            self.video_writer = cv2.VideoWriter(self.output_filename, fourcc, 15.0, (width, height))
            self.get_logger().info(f"Initialized video file '{self.output_filename}' with resolution {width}x{height}")

        self.video_writer.write(cv_img)

    def close(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info('Released video writer resource.')

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 convert_bag_to_video.py <path_to_bag> <output.mp4>")
        sys.exit(1)
        
    bag_path = sys.argv[1]
    output_path = sys.argv[2]
    topic = '/playback/destination_camera/image_raw'

    rclpy.init()
    node = VideoConverter(output_path, topic)
    
    # Start the bag playback in a separate process
    print(f"Starting playback of bag: {bag_path}")
    bag_process = subprocess.Popen(['ros2', 'bag', 'play', bag_path, '--remap', '/destination_camera/image_raw:=/playback/destination_camera/image_raw'])
    
    # Spin the node to process frames
    try:
        while bag_process.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            # If no frames for 5 seconds and playback is still somehow running but stuck, we could break
            # but usually bag_process.poll() is sufficient.
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        # Give it a tiny bit of time to process any remaining queued messages
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        node.close()
        node.destroy_node()
        rclpy.shutdown()
        if bag_process.poll() is None:
            bag_process.terminate()
        print(f"Conversion complete. Video saved to {output_path}")

if __name__ == '__main__':
    main()
