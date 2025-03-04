#!/usr/bin/env python3
import os
import glob
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import numpy as np
if np.__version__.startswith("2."):
    print("WARNING: Detected NumPy version 2.x. cv_bridge may not be compatible. "
          "Either downgrade to numpy<2 (e.g., pip install 'numpy<2') or rebuild cv_bridge "
          "with pybind11>=2.12.")

try:
    from cv_bridge import CvBridge
except Exception as e:
    print("Error importing cv_bridge:", e)
    print("Ensure that cv_bridge is built with a compatible NumPy version or "
          "rebuild it using pybind11>=2.12.")
    exit(1)


class ImagePlaybackNode(Node):
    def __init__(self):
        super().__init__("image_playback_node")

        # Declare and get parameters.
        self.declare_parameter("image_dir", "/path/to/images")
        self.declare_parameter("publish_rate", 10.0)  # images per second
        self.declare_parameter("image_topic", "/cam0/image_raw")
        self.declare_parameter("encoding", "mono8")  # use "bgr8" if your images are color

        self.image_dir = self.get_parameter("image_dir").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.image_topic = self.get_parameter("image_topic").value
        self.encoding = self.get_parameter("encoding").value

        # Set up publisher and cv_bridge.
        self.publisher_ = self.create_publisher(Image, self.image_topic, 10)
        self.bridge = CvBridge()

        # Find image files in the directory.
        #self.image_files = sorted(glob.glob(os.path.join(self.image_dir, "*.png")))
        # Sort by number in filename
        self.image_files = sorted(glob.glob(os.path.join(self.image_dir, "*.png")), key=lambda x: int(os.path.basename(x).split('.')[0]))
        if not self.image_files:
            self.get_logger().error(f"No images found in directory: {self.image_dir}")
            rclpy.shutdown()
            return

        self.current_index = 0
        self.get_logger().info(f"Found {len(self.image_files)} images in {self.image_dir}")

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def timer_callback(self):
        if self.current_index < len(self.image_files):
            image_path = self.image_files[self.current_index]
            cv_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
            if cv_image is None:
                self.get_logger().error(f"Failed to read image: {image_path}")
                self.current_index += 1
                return

            # Convert the OpenCV image to a ROS Image message.
            try:
                msg = self.bridge.cv2_to_imgmsg(cv_image, encoding=self.encoding)
            except Exception as e:
                self.get_logger().error(f"cv_bridge conversion failed for {image_path}: {e}")
                self.current_index += 1
                return

            # Set header information.
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"

            self.publisher_.publish(msg)
            self.get_logger().info(f"Published image: {os.path.basename(image_path)}")
            self.current_index += 1
        else:
            self.get_logger().info("Finished publishing all images. Shutting down node.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImagePlaybackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
