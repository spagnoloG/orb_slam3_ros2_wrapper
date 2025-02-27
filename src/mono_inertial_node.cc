#include "common.h"
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <thread>
#include <chrono>

using std::placeholders::_1;

class ImuGrabber
{
public:
  ImuGrabber() {}
  void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
  {
    std::lock_guard<std::mutex> lock(mBufMutex);
    imuBuf.push(imu_msg);
  }
  std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf;
  std::mutex mBufMutex;
};

class ImageGrabber
{
public:
  ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb) : mpSLAM(pSLAM), mpImuGb(pImuGb) {}

  void GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mBufMutex);
    if (!img0Buf.empty())
      img0Buf.pop();
    img0Buf.push(msg);
  }

  cv::Mat GetImage(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(img_msg, "mono8");
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Mono_Inertial"), "cv_bridge exception: %s", e.what());
    }
    return cv_ptr->image.clone();
  }

  void SyncWithImu()
  {
    while (rclcpp::ok())
    {
      if (!img0Buf.empty() && !mpImuGb->imuBuf.empty())
      {
        cv::Mat im;
        double tIm = img0Buf.front()->header.stamp.sec + img0Buf.front()->header.stamp.nanosec * 1e-9;
        {
          std::lock_guard<std::mutex> lock(mBufMutex);
          im = GetImage(img0Buf.front());
        }
        rclcpp::Time msg_time(img0Buf.front()->header.stamp.sec,
                              img0Buf.front()->header.stamp.nanosec, RCL_ROS_TIME);
        {
          std::lock_guard<std::mutex> lock(mBufMutex);
          img0Buf.pop();
        }

        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
          std::lock_guard<std::mutex> lock(mpImuGb->mBufMutex);
          while (!mpImuGb->imuBuf.empty() &&
                 (mpImuGb->imuBuf.front()->header.stamp.sec + mpImuGb->imuBuf.front()->header.stamp.nanosec * 1e-9) <= tIm)
          {
            double t = mpImuGb->imuBuf.front()->header.stamp.sec + mpImuGb->imuBuf.front()->header.stamp.nanosec * 1e-9;
            cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x,
                            mpImuGb->imuBuf.front()->linear_acceleration.y,
                            mpImuGb->imuBuf.front()->linear_acceleration.z);
            cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x,
                            mpImuGb->imuBuf.front()->angular_velocity.y,
                            mpImuGb->imuBuf.front()->angular_velocity.z);
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
            mpImuGb->imuBuf.pop();
          }
        }
        Sophus::SE3f Tcw = mpSLAM->TrackMonocular(im, tIm, vImuMeas);
        Sophus::SE3f Twc = Tcw.inverse();

        publish_ros_camera_pose(Twc, msg_time);
        publish_ros_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);
        publish_ros_tracked_mappoints(mpSLAM->GetTrackedMapPoints(), msg_time);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  std::queue<sensor_msgs::msg::Image::ConstSharedPtr> img0Buf;
  std::mutex mBufMutex;
  ORB_SLAM3::System* mpSLAM;
  ImuGrabber *mpImuGb;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("Mono_Inertial");

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

  sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
  ORB_SLAM3::System SLAM(voc_file, settings_file, sensor_type, enable_pangolin);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM, &imugb);

  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 1000, std::bind(&ImuGrabber::GrabImu, &imugb, _1));
  auto image_sub = node->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 100, std::bind(&ImageGrabber::GrabImage, &igb, _1));

  image_transport::ImageTransport it(node);
  setup_ros_publishers(node, it, Eigen::Vector3d::Zero());

  std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

  rclcpp::spin(node);

  SLAM.Shutdown();
  rclcpp::shutdown();
  sync_thread.join();

  return 0;
}
