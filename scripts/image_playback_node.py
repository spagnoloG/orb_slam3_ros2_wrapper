#!/usr/bin/env python3
import os
import glob

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImagePlaybackNode(Node):
    def __init__(self):
        super().__init__("image_playback_node")

        self.declare_parameter("image_dir", "/path/to/images")
        self.declare_parameter("publish_rate", 10.0)  # images per second
        self.declare_parameter("image_topic", "/cam0/image_raw")
        self.declare_parameter("encoding", "mono8")  # or "bgr8" for color

        self.image_dir = self.get_parameter("image_dir").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.image_topic = self.get_parameter("image_topic").value
        self.encoding = self.get_parameter("encoding").value

        self.publisher_ = self.create_publisher(Image, self.image_topic, 10)
        self.bridge = CvBridge()

        self.image_files = sorted(glob.glob(os.path.join(self.image_dir, "*.png")))
        if not self.image_files:
            self.get_logger().error(f"No images found in directory: {self.image_dir}")
            rclpy.shutdown()

        self.current_index = 0

        # Create a timer to publish images at the specified rate.
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(
            f"Found {len(self.image_files)} images in {self.image_dir}"
        )

    def timer_callback(self):
        if self.current_index < len(self.image_files):
            image_path = self.image_files[self.current_index]
            cv_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
            if cv_image is None:
                self.get_logger().error(f"Failed to read image: {image_path}")
                self.current_index += 1
                return

            # Convert to ROS Image message using cv_bridge.
            try:
                msg = self.bridge.cv2_to_imgmsg(cv_image, encoding=self.encoding)
            except Exception as e:
                self.get_logger().error(
                    f"cv_bridge conversion failed for {image_path}: {e}"
                )
                self.current_index += 1
                return

            # Set header stamp and frame id.
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"

            self.publisher_.publish(msg)
            self.get_logger().info(f"Published image: {os.path.basename(image_path)}")
            self.current_index += 1
        else:
            self.get_logger().info(
                "Finished publishing all images. Shutting down node."
            )
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImagePlaybackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
