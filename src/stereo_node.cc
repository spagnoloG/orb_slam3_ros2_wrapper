#include "common.h"
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

class ImageGrabber
{
public:
  ImageGrabber(ORB_SLAM3::System* pSLAM) : mpSLAM(pSLAM) {}

  void GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr &msgLeft,
                  const sensor_msgs::msg::Image::ConstSharedPtr &msgRight)
  {
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
      cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Stereo"), "cv_bridge exception: %s", e.what());
      return;
    }
    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
      cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Stereo"), "cv_bridge exception: %s", e.what());
      return;
    }
    Sophus::SE3f Tcc0 = mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image,
                      cv_ptrLeft->header.stamp.sec + cv_ptrLeft->header.stamp.nanosec*1e-9);
    Sophus::SE3f Twc = (Tcc0 * Tc0w).inverse();
    rclcpp::Time msg_time(cv_ptrLeft->header.stamp.sec, cv_ptrLeft->header.stamp.nanosec, RCL_ROS_TIME);

    publish_ros_camera_pose(Twc, msg_time);
    publish_ros_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);
    publish_ros_tracked_mappoints(mpSLAM->GetTrackedMapPoints(), msg_time);
  }

  ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("Stereo");

  node->declare_parameter<std::string>("voc_file", "file_not_set");
  node->declare_parameter<std::string>("settings_file", "file_not_set");
  node->declare_parameter<std::string>("world_frame_id", "map");
  node->declare_parameter<std::string>("cam_frame_id", "camera");
  node->declare_parameter<bool>("enable_pangolin", true);

  std::string voc_file = node->get_parameter("voc_file").as_string();
  std::string settings_file = node->get_parameter("settings_file").as_string();
  if (voc_file == "file_not_set" || settings_file == "file_not_set")
  {
    RCLCPP_ERROR(node->get_logger(), "Please provide voc_file and settings_file in the launch file");
    rclcpp::shutdown();
    return 1;
  }

  world_frame_id = node->get_parameter("world_frame_id").as_string();
  cam_frame_id = node->get_parameter("cam_frame_id").as_string();
  bool enable_pangolin = node->get_parameter("enable_pangolin").as_bool();

  Eigen::Vector3d rpy_rad(0, 0, 0);
  node->declare_parameter<double>("world_roll", 0.0);
  node->declare_parameter<double>("world_pitch", 0.0);
  node->declare_parameter<double>("world_yaw", 0.0);
  rpy_rad(0) = node->get_parameter("world_roll").as_double();
  rpy_rad(1) = node->get_parameter("world_pitch").as_double();
  rpy_rad(2) = node->get_parameter("world_yaw").as_double();

  sensor_type = ORB_SLAM3::System::STEREO;
  ORB_SLAM3::System SLAM(voc_file, settings_file, sensor_type, enable_pangolin);
  ImageGrabber igb(&SLAM);
	

  message_filters::Subscriber<sensor_msgs::msg::Image> left_sub(node.get(), "/camera/left/image_raw", rmw_qos_profile_default);
  message_filters::Subscriber<sensor_msgs::msg::Image> right_sub(node.get(), "/camera/right/image_raw", rmw_qos_profile_default);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), left_sub, right_sub);
  sync.registerCallback(std::bind(&ImageGrabber::GrabStereo, &igb, std::placeholders::_1, std::placeholders::_2));

  image_transport::ImageTransport it(node);
  setup_ros_publishers(node, it, rpy_rad);

  rclcpp::spin(node);

  SLAM.Shutdown();
  rclcpp::shutdown();

  return 0;
}

