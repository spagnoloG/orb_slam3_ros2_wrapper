#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class TrajectoryServer(Node):
    def __init__(self):
        super().__init__("trajectory_server")
        self.subscription = self.create_subscription(
            PoseStamped, "/orb_slam3/camera_pose", self.pose_callback, 10
        )
        self.publisher = self.create_publisher(Path, "/reconstructed_path", 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = "world"

        self.timer = self.create_timer(0.1, self.timer_callback)

    def pose_callback(self, msg: PoseStamped):
        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(msg)

    def timer_callback(self):
        self.publisher.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
