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
                executable="orb_slam3_ros_wrapper_mono",
                name="orb_slam3_mono",
                output="screen",
                remappings=[("/camera/image_raw", "/cam0/image_raw")],
                parameters=[
                    {
                        "voc_file": os.path.join(pkg_share, "config", "ORBvoc.txt"),
                        "settings_file": os.path.join(
                            pkg_share, "config", "dji_mini_3_calibration.yaml"
                        ),
                        "world_frame_id": "world",
                        "cam_frame_id": "camera",
                        "enable_pangolin": True,
                        "world_roll": 0.0,
                        "world_pitch": 0.0,
                        "world_yaw": 0.0,
                        "video_file": "/home/gasper/Datasets/UAV-LOC-DATASET/FRI-110m-sunny.mp4",
                        "stride": 6
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
                    os.path.join(pkg_share, "config", "orb_slam3_no_imu.rviz"),
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
