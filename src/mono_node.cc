#include "common.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"


class ImageGrabber {
public:
  ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}

  void GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    // Log message to confirm that the callback is triggered.
    RCLCPP_INFO(rclcpp::get_logger("Mono"), "Received an image message");

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("Mono"), "cv_bridge exception: %s", e.what());
      return;
    }

    Sophus::SE3f Tcc0 = mpSLAM->TrackMonocular(
        cv_ptr->image,
        cv_ptr->header.stamp.sec + cv_ptr->header.stamp.nanosec * 1e-9);

    // Log the output transformation matrix from ORB_SLAM3.
    Eigen::Matrix<float, 4, 4> Tcc0_mat = Tcc0.matrix();
    //RCLCPP_INFO(rclcpp::get_logger("Mono"),
    //            "Tcc0 matrix:\n[%f, %f, %f, %f]\n[%f, %f, %f, %f]\n[%f, %f, %f, %f]\n[%f, %f, %f, %f]",
    //            Tcc0_mat(0,0), Tcc0_mat(0,1), Tcc0_mat(0,2), Tcc0_mat(0,3),
    //            Tcc0_mat(1,0), Tcc0_mat(1,1), Tcc0_mat(1,2), Tcc0_mat(1,3),
    //            Tcc0_mat(2,0), Tcc0_mat(2,1), Tcc0_mat(2,2), Tcc0_mat(2,3),
    //            Tcc0_mat(3,0), Tcc0_mat(3,1), Tcc0_mat(3,2), Tcc0_mat(3,3));

    // Calculate camera pose. Make sure that Tc0w is defined and properly initialized.
    Sophus::SE3f Twc = (Tcc0 * Tc0w).inverse();
    //RCLCPP_INFO(rclcpp::get_logger("Mono"), "Computed camera pose (Twc) from Tcc0 and Tc0w.");

    // Create a ROS time stamp from the image header.
    rclcpp::Time msg_time(cv_ptr->header.stamp.sec,
                          cv_ptr->header.stamp.nanosec, RCL_ROS_TIME);

    publish_ros_camera_pose(Twc, msg_time);
    publish_ros_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);
    publish_ros_tracked_mappoints(mpSLAM->GetTrackedMapPoints(), msg_time);
  }

  ORB_SLAM3::System *mpSLAM;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("Mono");

  // Declare and retrieve parameters.
  node->declare_parameter<std::string>("voc_file", "file_not_set");
  node->declare_parameter<std::string>("settings_file", "file_not_set");
  node->declare_parameter<std::string>("world_frame_id", "map");
  node->declare_parameter<std::string>("cam_frame_id", "camera");
  node->declare_parameter<bool>("enable_pangolin", true);

  std::string voc_file = node->get_parameter("voc_file").as_string();
  std::string settings_file = node->get_parameter("settings_file").as_string();
  if (voc_file == "file_not_set" || settings_file == "file_not_set") {
    RCLCPP_ERROR(node->get_logger(), "Please provide voc_file and settings_file in the launch file");
    rclcpp::shutdown();
    return 1;
  }

  // Retrieve frame IDs.
  world_frame_id = node->get_parameter("world_frame_id").as_string();
  cam_frame_id = node->get_parameter("cam_frame_id").as_string();
  bool enable_pangolin = node->get_parameter("enable_pangolin").as_bool();

  // Retrieve world orientation parameters.
  Eigen::Vector3d rpy_rad(0, 0, 0);
  node->declare_parameter<double>("world_roll", 0.0);
  node->declare_parameter<double>("world_pitch", 0.0);
  node->declare_parameter<double>("world_yaw", 0.0);
  rpy_rad(0) = node->get_parameter("world_roll").as_double();
  rpy_rad(1) = node->get_parameter("world_pitch").as_double();
  rpy_rad(2) = node->get_parameter("world_yaw").as_double();

  // Set sensor type to MONOCULAR.
  sensor_type = ORB_SLAM3::System::MONOCULAR;
  ORB_SLAM3::System SLAM(voc_file, settings_file, sensor_type, enable_pangolin);
  ImageGrabber igb(&SLAM);

  // Create subscription (topic remapping should be configured via the launch file)
  auto image_sub = node->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 1,
      std::bind(&ImageGrabber::GrabImage, &igb, std::placeholders::_1));

  // Set up additional ROS publishers (assumes setup_ros_publishers is implemented elsewhere)
  image_transport::ImageTransport it(node);
  setup_ros_publishers(node, it, rpy_rad);

  rclcpp::spin(node);

  SLAM.Shutdown();
  rclcpp::shutdown();

  return 0;
}