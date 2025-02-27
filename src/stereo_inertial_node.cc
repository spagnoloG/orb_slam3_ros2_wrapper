#include "common.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <thread>

class ImuGrabber {
public:
  ImuGrabber() {}
  void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg) {
    std::lock_guard<std::mutex> lock(mBufMutex);
    imuBuf.push(imu_msg);
  }
  std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf;
  std::mutex mBufMutex;
};

class ImageGrabber {
public:
  ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb)
      : mpSLAM(pSLAM), mpImuGb(pImuGb) {}

  void GrabImageLeft(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg) {
    std::lock_guard<std::mutex> lock(mBufMutexLeft);
    if (!imgLeftBuf.empty())
      imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
  }

  void GrabImageRight(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg) {
    std::lock_guard<std::mutex> lock(mBufMutexRight);
    if (!imgRightBuf.empty())
      imgRightBuf.pop();
    imgRightBuf.push(img_msg);
  }

  cv::Mat GetImage(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(img_msg, "mono8");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("Stereo_Inertial"),
                   "cv_bridge exception: %s", e.what());
    }
    return cv_ptr->image.clone();
  }

  void SyncWithImu() {
    const double maxTimeDiff = 0.01;
    while (rclcpp::ok()) {
      cv::Mat imLeft, imRight;
      double tImLeft = 0, tImRight = 0;
      if (!imgLeftBuf.empty() && !imgRightBuf.empty() &&
          !mpImuGb->imuBuf.empty()) {
        tImLeft = imgLeftBuf.front()->header.stamp.sec +
                  imgLeftBuf.front()->header.stamp.nanosec * 1e-9;
        tImRight = imgRightBuf.front()->header.stamp.sec +
                   imgRightBuf.front()->header.stamp.nanosec * 1e-9;

        {
          std::lock_guard<std::mutex> lock(mBufMutexRight);
          while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1) {
            imgRightBuf.pop();
            tImRight = imgRightBuf.front()->header.stamp.sec +
                       imgRightBuf.front()->header.stamp.nanosec * 1e-9;
          }
        }

        {
          std::lock_guard<std::mutex> lock(mBufMutexLeft);
          while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1) {
            imgLeftBuf.pop();
            tImLeft = imgLeftBuf.front()->header.stamp.sec +
                      imgLeftBuf.front()->header.stamp.nanosec * 1e-9;
          }
        }

        if (fabs(tImLeft - tImRight) > maxTimeDiff)
          continue;
        double latestImuTime =
            mpImuGb->imuBuf.back()->header.stamp.sec +
            mpImuGb->imuBuf.back()->header.stamp.nanosec * 1e-9;
        if (tImLeft > latestImuTime)
          continue;

        rclcpp::Time msg_time(imgLeftBuf.front()->header.stamp.sec,
                              imgLeftBuf.front()->header.stamp.nanosec,
                              RCL_ROS_TIME);
        {
          std::lock_guard<std::mutex> lock(mBufMutexLeft);
          imLeft = GetImage(imgLeftBuf.front());
          imgLeftBuf.pop();
        }
        {
          std::lock_guard<std::mutex> lock(mBufMutexRight);
          imRight = GetImage(imgRightBuf.front());
          imgRightBuf.pop();
        }

        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
          std::lock_guard<std::mutex> lock(mpImuGb->mBufMutex);
          while (!mpImuGb->imuBuf.empty() &&
                 (mpImuGb->imuBuf.front()->header.stamp.sec +
                  mpImuGb->imuBuf.front()->header.stamp.nanosec * 1e-9) <=
                     tImLeft) {
            double t = mpImuGb->imuBuf.front()->header.stamp.sec +
                       mpImuGb->imuBuf.front()->header.stamp.nanosec * 1e-9;
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
        Sophus::SE3f Tcw =
            mpSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
        Sophus::SE3f Twc = Tcw.inverse();
        publish_ros_camera_pose(Twc, msg_time);
        publish_ros_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);
        publish_ros_tracked_mappoints(mpSLAM->GetTrackedMapPoints(), msg_time);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  std::queue<sensor_msgs::msg::Image::ConstSharedPtr> imgLeftBuf, imgRightBuf;
  std::mutex mBufMutexLeft, mBufMutexRight;
  ORB_SLAM3::System *mpSLAM;
  ImuGrabber *mpImuGb;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("Stereo_Inertial");

  node->declare_parameter<std::string>("voc_file", "file_not_set");
  node->declare_parameter<std::string>("settings_file", "file_not_set");
  node->declare_parameter<std::string>("world_frame_id", "world");
  node->declare_parameter<std::string>("cam_frame_id", "camera");
  node->declare_parameter<bool>("enable_pangolin", true);

  std::string voc_file = node->get_parameter("voc_file").as_string();
  std::string settings_file = node->get_parameter("settings_file").as_string();
  if (voc_file == "file_not_set" || settings_file == "file_not_set") {
    RCLCPP_ERROR(
        node->get_logger(),
        "Please provide voc_file and settings_file in the launch file");
    rclcpp::shutdown();
    return 1;
  }

  world_frame_id = node->get_parameter("world_frame_id").as_string();
  cam_frame_id = node->get_parameter("cam_frame_id").as_string();
  bool enable_pangolin = node->get_parameter("enable_pangolin").as_bool();

  sensor_type = ORB_SLAM3::System::IMU_STEREO;
  ORB_SLAM3::System SLAM(voc_file, settings_file, sensor_type, enable_pangolin);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM, &imugb);

  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 1000,
      std::bind(&ImuGrabber::GrabImu, &imugb, std::placeholders::_1));
  auto img_left_sub = node->create_subscription<sensor_msgs::msg::Image>(
      "/camera/left/image_raw", 100,
      std::bind(&ImageGrabber::GrabImageLeft, &igb, std::placeholders::_1));
  auto img_right_sub = node->create_subscription<sensor_msgs::msg::Image>(
      "/camera/right/image_raw", 100,
      std::bind(&ImageGrabber::GrabImageRight, &igb, std::placeholders::_1));

  image_transport::ImageTransport it(node);
  setup_ros_publishers(node, it, Eigen::Vector3d::Zero());

  std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

  rclcpp::spin(node);

  sync_thread.join();
  rclcpp::shutdown();

  return 0;
}
