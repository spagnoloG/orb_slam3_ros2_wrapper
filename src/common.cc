#include "common.h"

// Global variable definitions
ORB_SLAM3::System::eSensor sensor_type;
std::string world_frame_id, cam_frame_id, imu_frame_id;
Sophus::SE3f Tc0w = Sophus::SE3f();

// Global ROS2 publishers (set up in setup_ros_publishers)
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub;

// Global transform broadcaster pointer (created with the node)
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

void setup_ros_publishers(const rclcpp::Node::SharedPtr &node, image_transport::ImageTransport &it, const Eigen::Vector3d &rpy_rad)
{
    pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("orb_slam3/camera_pose", 1);
    map_points_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("orb_slam3/map_points", 1);

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    if (!rpy_rad.isZero(0))
    {
        Eigen::AngleAxisf AngleR(rpy_rad(0), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf AngleP(rpy_rad(1), Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf AngleY(rpy_rad(2), Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf qRPY = AngleR * AngleP * AngleY;
        Eigen::Matrix3f RotRPY = qRPY.matrix();
        Tc0w = Sophus::SE3f(RotRPY, Eigen::Vector3f::Zero());
        RCLCPP_INFO(node->get_logger(), "World frame will be rotated by RPY (in that order) %f %f %f (rad)",
                    rpy_rad[0], rpy_rad[1], rpy_rad[2]);
    }
}

void publish_ros_camera_pose(Sophus::SE3f Twc_SE3f, rclcpp::Time msg_time)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = world_frame_id;
    pose_msg.header.stamp = msg_time;

    pose_msg.pose.position.x = Twc_SE3f.translation().x();
    pose_msg.pose.position.y = Twc_SE3f.translation().y();
    pose_msg.pose.position.z = Twc_SE3f.translation().z();

    auto quat = Twc_SE3f.unit_quaternion();
    pose_msg.pose.orientation.w = quat.w();
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();

    pose_pub->publish(pose_msg);
}

void publish_ros_tf_transform(Sophus::SE3f Twc_SE3f, const std::string &frame_id, const std::string &child_frame_id, rclcpp::Time msg_time)
{
    auto transform_stamped = SE3f_to_transform_stamped(Twc_SE3f, frame_id, child_frame_id, msg_time);
    tf_broadcaster->sendTransform(transform_stamped);
}

void publish_ros_tracked_mappoints(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time)
{
    sensor_msgs::msg::PointCloud2 cloud = tracked_mappoints_to_pointcloud(map_points, msg_time);
    map_points_pub->publish(cloud);
}

geometry_msgs::msg::TransformStamped SE3f_to_transform_stamped(Sophus::SE3f T_SE3f, const std::string &frame_id, const std::string &child_frame_id, rclcpp::Time stamp)
{
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = stamp;
    transformStamped.header.frame_id = frame_id;
    transformStamped.child_frame_id = child_frame_id;

    transformStamped.transform.translation.x = T_SE3f.translation().x();
    transformStamped.transform.translation.y = T_SE3f.translation().y();
    transformStamped.transform.translation.z = T_SE3f.translation().z();

    auto quat = T_SE3f.unit_quaternion();
    transformStamped.transform.rotation.w = quat.w();
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();

    return transformStamped;
}

sensor_msgs::msg::PointCloud2 tracked_mappoints_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time)
{
    const int num_channels = 3; // x, y, z

    if (map_points.empty())
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = msg_time;
    cloud.header.frame_id = world_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z" };

    for (int i = 0; i < num_channels; i++)
    {
        sensor_msgs::msg::PointField field;
        field.name = channel_id[i];
        field.offset = i * sizeof(float);
        field.count = 1;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[i] = field;
    }

    cloud.data.resize(cloud.row_step * cloud.height);
    unsigned char *cloud_data_ptr = cloud.data.data();

    for (size_t i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {
            Eigen::Vector3f pMPw = map_points[i]->GetWorldPos();
            if (sensor_type == ORB_SLAM3::System::MONOCULAR || sensor_type == ORB_SLAM3::System::STEREO)
            {
                Sophus::SE3f Tc0mp(Eigen::Matrix3f::Identity(), pMPw);
                Sophus::SE3f Twmp = Tc0w.inverse() * Tc0mp;
                pMPw = Twmp.translation();
            }
            float data_array[num_channels] = { pMPw.x(), pMPw.y(), pMPw.z() };
            memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
        }
    }
    return cloud;
}
