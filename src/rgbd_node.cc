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

  void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr &msgRGB,
                const sensor_msgs::msg::Image::ConstSharedPtr &msgD)
  {
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RGBD"), "cv_bridge exception: %s", e.what());
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
      cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RGBD"), "cv_bridge exception: %s", e.what());
      return;
    }

    Sophus::SE3f Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image,
                      cv_ptrRGB->header.stamp.sec + cv_ptrRGB->header.stamp.nanosec*1e-9);
    Sophus::SE3f Twc = Tcw.inverse();
    rclcpp::Time msg_time(cv_ptrRGB->header.stamp.sec, cv_ptrRGB->header.stamp.nanosec, RCL_ROS_TIME);

    publish_ros_camera_pose(Twc, msg_time);
    publish_ros_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);
    publish_ros_tracked_mappoints(mpSLAM->GetTrackedMapPoints(), msg_time);
  }

  ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("RGBD");

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

  sensor_type = ORB_SLAM3::System::RGBD;
  ORB_SLAM3::System SLAM(voc_file, settings_file, sensor_type, enable_pangolin);

  ImageGrabber igb(&SLAM);

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub(node.get(), "/camera/rgb/image_raw", rmw_qos_profile_default);
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub(node.get(), "/camera/depth_registered/image_raw", rmw_qos_profile_default);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), rgb_sub, depth_sub);
  sync.registerCallback(std::bind(&ImageGrabber::GrabRGBD, &igb, std::placeholders::_1, std::placeholders::_2));

  image_transport::ImageTransport it(node);
  setup_ros_publishers(node, it, Eigen::Vector3d::Zero());

  rclcpp::spin(node);

  SLAM.Shutdown();
  rclcpp::shutdown();

  return 0;
}

