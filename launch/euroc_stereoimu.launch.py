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
                executable="orb_slam3_ros_wrapper_stereo_inertial",
                name="orb_slam3_stereo_inertial",
                output="screen",
                remappings=[
                    ("/camera/left/image_raw", "/cam0/image_raw"),
                    ("/camera/right/image_raw", "/cam1/image_raw"),
                    ("/imu", "/imu0"),
                ],
                parameters=[
                    {
                        "voc_file": os.path.join(pkg_share, "config", "ORBvoc.txt"),
                        "settings_file": os.path.join(
                            pkg_share, "config", "EuRoC.yaml"
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
                package="hector_trajectory_server",
                executable="hector_trajectory_server",
                name="trajectory_server_orb_slam3",
                namespace="orb_slam3_ros",
                output="screen",
                parameters=[
                    {
                        "target_frame_name": "/world",
                        "source_frame_name": "/camera",
                        "trajectory_update_rate": 20.0,
                        "trajectory_publish_rate": 20.0,
                    }
                ],
            ),
        ]
    )
