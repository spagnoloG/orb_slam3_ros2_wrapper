import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("orb_slam3_ros_wrapper")

    return LaunchDescription(
        [
            Node(
                package="orb_slam3_ros_wrapper",
                executable="orb_slam3_ros_wrapper_mono_inertial",
                name="orb_slam3_mono_inertial",
                output="screen",
                remappings=[
                    ("/camera/image_raw", "/camera/fisheye1/image_raw"),
                    ("/imu", "/camera/imu"),
                ],
                parameters=[
                    {
                        "voc_file": os.path.join(pkg_share, "config", "ORBvoc.txt"),
                        "settings_file": os.path.join(
                            pkg_share, "config", "Realsense_T265.yaml"
                        ),
                        "world_frame_id": "world",
                        "cam_frame_id": "camera",
                        "enable_pangolin": True,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(pkg_share, "config", "orb_slam3_with_imu.rviz"),
                ],
            ),
            Node(
                package="orb_slam3_ros_wrapper",
                executable="trajectory_server",
                name="trajectory_server_orb_slam3",
                output="screen",
            ),
        ]
    )
