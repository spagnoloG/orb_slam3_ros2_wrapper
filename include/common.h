#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <Eigen/Dense>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "image_transport/image_transport.hpp"

#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_field.hpp"

// Message filters (ROS2 version)
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

// ORB-SLAM3-specific libraries. (Assume include path set via CMakeLists.txt)
#include "System.h"
#include "ImuTypes.h"

// Global variables (defined in common.cc)
extern ORB_SLAM3::System::eSensor sensor_type;
extern std::string world_frame_id, cam_frame_id, imu_frame_id;
extern Sophus::SE3f Tc0w;

// Instead of ros::Publisher, we use shared pointers to rclcpp publishers.
extern rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub;

// Function prototypes – note that we pass the node pointer (rclcpp::Node::SharedPtr) instead of a NodeHandle.
void setup_ros_publishers(
    const rclcpp::Node::SharedPtr &node,
    image_transport::ImageTransport &it,
    const Eigen::Vector3d &rpy_rad = Eigen::Vector3d::Zero());

void publish_ros_camera_pose(Sophus::SE3f, rclcpp::Time);
void publish_ros_tracked_mappoints(std::vector<ORB_SLAM3::MapPoint*>, rclcpp::Time);
void publish_ros_tf_transform(Sophus::SE3f, const std::string &, const std::string &, rclcpp::Time);

// Helper to convert a Sophus::SE3f into a TransformStamped for tf2.
geometry_msgs::msg::TransformStamped SE3f_to_transform_stamped(Sophus::SE3f, const std::string &frame_id, const std::string &child_frame_id, rclcpp::Time stamp);

sensor_msgs::msg::PointCloud2 tracked_mappoints_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*>, rclcpp::Time);

#endif // COMMON_H

